const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

class Element {
  constructor(tag) {
    this.tagName = tag.toUpperCase();
    this.children = [];
    this.style = {};
    this.dataset = {};
    this.className = "";
    this.textContent = "";
    this.disabled = false;
    this.listeners = {};
  }
  append(...nodes) { this.children.push(...nodes); }
  appendChild(node) { this.children.push(node); return node; }
  addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }
  dispatch(type, event = {}) {
    const payload = {
      preventDefault() {}, pointerId: 7, clientX: 0, clientY: 0,
      target: this, ...event,
    };
    if (typeof this[`on${type}`] === "function") this[`on${type}`](payload);
    for (const fn of this.listeners[type] || []) fn(payload);
  }
  setAttribute(name, value) { this[name] = String(value); }
  querySelectorAll(selector) { return findAll(this, node => node.tagName === selector.toUpperCase()); }
  get firstChild() { return this.children[0]; }
  getBoundingClientRect() { return { left: 0, top: 0, width: 200, height: 200 }; }
  setPointerCapture() {}
  releasePointerCapture() {}
}

const documentListeners = {};
const windowListeners = {};
const document = {
  hidden: false,
  createElement: tag => new Element(tag),
  addEventListener(type, fn) { (documentListeners[type] ||= []).push(fn); },
};
const window = {
  addEventListener(type, fn) { (windowListeners[type] ||= []).push(fn); },
};
const intervals = [];
const sandbox = {
  document, window,
  setInterval(fn) { intervals.push(fn); return intervals.length; },
  clearInterval() {},
};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(process.argv[2], "utf8") +
  "\nglobalThis.remotePanel = Panels.remote;", sandbox);

const sent = [];
const disconnect = [];
const acknowledgements = [];
const pageChanges = [];
const body = new Element("div");
const ctx = {
  cfg: { limits: { max_linear_velocity: .5, simulation_max_linear_velocity: 2,
                   max_angular_velocity: 1.2 } },
  send: payload => sent.push(payload),
  onDisconnect: fn => disconnect.push(fn),
  onAck: fn => acknowledgements.push(fn),
  onPageChange: fn => pageChanges.push(fn),
};
const panel = sandbox.remotePanel.render(body, ctx);

const walk = find(body, node => node.textContent === "Start travel (trot)");
const greeting = find(body, node => node.textContent === "Greeting");
const stop = find(body, node => node.textContent === "STOP MOVEMENT");
const save = find(body, node => node.textContent === "Save room scan");
const scanName = find(body, node => node["aria-label"] === "Room scan name");
const speedLimit = find(body, node => node["aria-label"] === "Maximum driving speed");
const sticks = findAll(body, node => node.className.includes("joystick-pad"));
assert.ok(walk && greeting && stop && save && scanName && speedLimit,
  "remote actions and safe scan name input should be visible");
assert.equal(sticks.length, 2, "two touch/mouse joysticks should be rendered");
assert.match(sticks[0]["aria-label"], /forward.*lateral/i);
assert.match(sticks[1]["aria-label"], /turn/i);
assert.equal(speedLimit.value, .5, "remote drive starts at the conservative hardware limit");

walk.onclick();
greeting.onclick();
assert.deepEqual(plain(sent.slice(0, 2)), [
  { action: "gait", gait: "trot", enable: true },
  { action: "greeting" },
]);
scanName.value = "office-east";
save.onclick();
assert.deepEqual(plain(sent.at(-1)), { action: "save_map", name: "office-east" });
acknowledgements[0]({ action: "save_map", ok: true, message: "saved",
  data: { manifest_path: "/maps/office-east/manifest.json" } });
assert.match(flatten(body).join(" "), /office-east\/manifest\.json/);

// Left stick: right and up requests lateral and forward motion. Pointer release
// is the deadman and must explicitly command zero velocity.
sticks[0].dispatch("pointerdown", { clientX: 180, clientY: 20 });
assert.equal(sent.at(-1).action, "cmd_vel");
assert.ok(sent.at(-1).vx > 0);
assert.ok(sent.at(-1).vy < 0);
sticks[0].dispatch("pointercancel");
assert.deepEqual(plain(sent.at(-1)), { action: "cmd_vel", vx: 0, vy: 0, wz: 0 });

// Each pad owns its pointer so two-finger control remains independent.
sticks[0].dispatch("pointerdown", { pointerId: 11, clientX: 100, clientY: 20 });
sticks[1].dispatch("pointerdown", { pointerId: 12, clientX: 20, clientY: 100 });
sticks[0].dispatch("pointerup", { pointerId: 11 });
assert.notEqual(sent.at(-1).wz, 0, "releasing translation must preserve the turn pointer");
sticks[1].dispatch("pointerup", { pointerId: 12 });
assert.deepEqual(plain(sent.at(-1)), { action: "cmd_vel", vx: 0, vy: 0, wz: 0 });

// Keyboard drive works for laptops, ignores key repeat, and releases to zero.
fire(documentListeners.keydown, { code: "KeyW", target: { tagName: "BODY" }, repeat: false });
assert.ok(sent.at(-1).vx > 0);
fire(documentListeners.keyup, { code: "KeyW", target: { tagName: "BODY" } });
assert.deepEqual(plain(sent.at(-1)), { action: "cmd_vel", vx: 0, vy: 0, wz: 0 });

// Losing browser focus, hiding the tab, disconnecting, and the stop button are
// independent deadman paths. Each must publish a zero command.
for (const action of [
  () => fire(windowListeners.blur, {}),
  () => { document.hidden = true; fire(documentListeners.visibilitychange, {}); },
  () => disconnect[0](),
  () => pageChanges[0]("remote", "dashboard"),
  () => stop.onclick(),
]) {
  sent.length = 0;
  action();
  assert.deepEqual(plain(sent.at(-1)), { action: "cmd_vel", vx: 0, vy: 0, wz: 0 });
}

panel.update({
  state: "MOVING", base: { vel: [.8, .1, 0], omega: [0, 0, .2], pos: [1, 2, .3] },
  controller: { gait: "walk", mode: "IMPEDANCE" },
  safety: { estop: false, watchdog_ok: true, max_util: .42, max_temp: 47.5,
            most_loaded: "FL_kfe_joint", hottest: "RR_kfe_joint" },
  simulation: { active: true, backend: "mujoco", world: "house" },
  camera: { available: true },
});
const allText = flatten(body).join(" ");
assert.match(allText, /MOVING/);
assert.match(allText, /0\.81 m\/s/);
assert.match(allText, /42%/);
assert.match(allText, /47\.5 °C/);
assert.match(allText, /mujoco \/ house/);
assert.equal(sticks[0].dataset.linearLimit, "2");
assert.equal(speedLimit.max, 2);
assert.equal(speedLimit.value, .5, "simulation telemetry must not silently raise drive speed");
speedLimit.value = 1;
speedLimit.dispatch("input");
sticks[0].dispatch("pointerdown", { pointerId: 20, clientX: 100, clientY: 0 });
assert.equal(sent.at(-1).vx, 1, "1 m/s needs an explicit operator adjustment");

function fire(list = [], event) {
  for (const fn of list) fn({ preventDefault() {}, repeat: false, ...event });
}
function plain(value) { return JSON.parse(JSON.stringify(value)); }
function find(root, predicate) { return findAll(root, predicate)[0]; }
function findAll(root, predicate) {
  const found = [];
  const visit = node => {
    if (predicate(node)) found.push(node);
    for (const child of node.children || []) visit(child);
  };
  visit(root);
  return found;
}
function flatten(root) {
  const values = [root.textContent || ""];
  for (const child of root.children || []) values.push(...flatten(child));
  return values;
}
