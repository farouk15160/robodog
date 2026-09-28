const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

// Minimal DOM adapter: test the real panel render/update interface, no browser
// globals other than the elements this panel actually uses.
class Element {
  constructor(tag) {
    this.tagName = tag;
    this.children = [];
    this.style = {};
    this.textContent = "";
  }
  append(...nodes) { this.children.push(...nodes); }
  appendChild(node) { this.children.push(node); return node; }
  get firstChild() { return this.children[0]; }
}
const sandbox = { document: { createElement: tag => new Element(tag) } };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(process.argv[2], "utf8") +
  "\nglobalThis.jointsPanel = Panels.joints;", sandbox);
const names = ["FL", "FR", "RL", "RR"].flatMap(leg =>
  ["haa", "hfe", "kfe"].map(axis => `${leg}_${axis}_joint`));
const actuators = Object.fromEntries(names.map(name => [name, {
  ratio: name.includes("kfe") ? 2 : 1,
  efficiency: name.includes("kfe") ? .95 : 1,
  torque_gain: name.includes("kfe") ? 1.9 : 1,
  continuous_torque_nm: name.includes("kfe") ? 15.2 : 8,
  continuous_limit_basis: "Stall rating; installed cooling unverified",
  peak_torque_nm: name.includes("kfe") ? 68.4 : 36,
}]));
const body = new Element("div");
const ctx = {
  info: { joints: names, joint_actuators: actuators },
  cfg: { thresholds: { torque_warn: .7, torque_alarm: .95,
    temperature_warn_c: 70, temperature_alarm_c: 85, tracking_error_warn_rad: .05 } },
};
const panel = sandbox.jointsPanel.render(body, ctx);
const joints = names.map(name => ({ name, pos: 0, pos_cmd: 0, err: 0, vel: 0,
  eff: name.includes("kfe") ? -15.2 : 8, cur: 4.5, temp: 71.2, util: 1,
  mode: "IMPEDANCE", faults: [], enabled: true }));
// The protocol names identify joints; reordering packets must not relabel a leg.
panel.update({ joints: [...joints].reverse(), simulation: { active: true } });
const table = body.children.find(node => node.tagName === "table");
const rows = table.children[1].children;
assert.equal(rows.length, 12);
assert.match(table.children[0].innerHTML, /joint.*N·m/);
assert.match(table.children[0].innerHTML, /motor.*N·m/);
assert.equal(rows[2].children[0].textContent, "FL_kfe");
assert.equal(rows[2].children[5].textContent, "-15.20");
assert.equal(rows[2].children[6].textContent, "-8.00");
assert.equal(rows[2].children[7].textContent, "15.2 / 68.4");
assert.equal(rows[2].children[8].children[0].textContent, "100%");
assert.equal(rows[2].children[10].textContent, "71.2");
assert.match(body.children[0].textContent, /thermal estimate/i);
assert.match(body.children[0].textContent, /uncalibrated/i);
assert.match(body.children[0].textContent, /Knee belt 2:1, 95% assumed efficiency/);
assert.match(table.children[0].innerHTML, /cont limit \/ peak/);
assert.match(body.children[0].textContent, /continuous limit.*stall rating/i);
assert.match(body.children[0].textContent, /τ\/Kt estimate/i);
assert.match(body.children[0].textContent, /high-current saturation/i);
assert.match(body.children[0].textContent, /not measured phase\/battery\/iq/i);
assert.match(body.children[0].textContent, /copper-loss model/i);
panel.update({ joints, simulation: { active: false } });
assert.match(body.children[0].textContent, /reported motor temperature.*thermal estimate/i);
