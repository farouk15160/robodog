const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");
class Element {
  constructor(tag) {
    this.tagName = tag; this.children = []; this.style = {};
    this.textContent = ""; this.classList = { toggle() {} };
  }
  append(...nodes) { this.children.push(...nodes); }
  appendChild(node) { this.children.push(node); return node; }
  addEventListener() {}
}
const sandbox = { document: { createElement: tag => new Element(tag) }, confirm: () => true };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(process.argv[2], "utf8") +
  "\nglobalThis.gaitPanel = Panels.gait;", sandbox);
const sent = [];
const ctx = { cfg: { limits: { max_linear_velocity: .5,
  simulation_max_linear_velocity: 2, max_angular_velocity: 1.2 } },
  send: command => sent.push(command), onDisconnect() {} };
const body = new Element("div");
const panel = sandbox.gaitPanel.render(body, ctx);
const descendants = element => [element, ...element.children.flatMap(descendants)];
const forward = descendants(body).find(element => element.id === "vx");
panel.update({ simulation: { active: true }, controller: { gait: "stand" } });
assert.equal(Number(forward.max), 2);
assert.equal(Number(forward.min), -2);
assert.match(descendants(body).map(node => node.textContent).join(" "), /experimental simulation/i);
forward.value = "2";
forward.onchange();
assert.equal(sent.at(-1).vx, 2);
panel.update({ simulation: { active: false }, controller: { gait: "stand" } });
assert.equal(Number(forward.max), .5);
assert.equal(Number(forward.value), .5);
assert.match(descendants(body).map(node => node.textContent).join(" "), /hardware limit/i);
