const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

class Element {
  constructor(tag) { this.tagName = tag; this.children = []; this.style = {}; this.textContent = ""; }
  append(...nodes) { this.children.push(...nodes); }
  appendChild(node) { this.children.push(node); return node; }
  get firstChild() { return this.children[0]; }
  querySelectorAll(tag) { return this.children.flatMap(c =>
    [...(c.tagName === tag ? [c] : []), ...c.querySelectorAll(tag)]); }
}
const content = node => [node.textContent, ...node.children.map(content)].join(" ");
const sandbox = { document: { createElement: tag => new Element(tag) } };
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(process.argv[2], "utf8") +
  "\nglobalThis.panels = Panels;", sandbox);
const names = ["FL_haa_joint", "FL_hfe_joint", "FL_kfe_joint"];
const ctx = { info: {
  robot: "robodog", actuator: "ROBSTRIDE06", actuator_revision: "2026-09-17",
  joints: names, mass_kg: 19.719271,
  geometry: { body_box_m: [.384, .22, .12], nominal_footprint_m: { length: .582, width: .289002 },
    thigh_length_m: .192, shank_length_m: .195621 },
  performance: { stall_continuous_torque_nm: 8, rated_torque_nm: 11, peak_torque_nm: 36,
    rated_speed_rpm: 100, vendor_curve: { heat_sink_dimensions_mm: [200, 200] } },
  electrical: { rated_voltage_v: 48, supply_voltage_v: 44.4, torque_constant_nm_per_arms: 1.1 },
  mass_budget: { cad_export_kg: 7.549431, actuators_kg: 7.452, electronics_payload_kg: 4.452 },
  joint_actuators: { FL_kfe_joint: { ratio: 2, efficiency: .95 } },
}, cfg: { thresholds: { tracking_error_warn_rad: .05, temperature_warn_c: 70, temperature_alarm_c: 85 } } };
const body = new Element("div");
assert.ok(sandbox.panels.jointstats, "A separate rolling statistics panel must exist");
const panel = sandbox.panels.jointstats.render(body, ctx);
const sample = name => ({ name, kp: 55, kd: 2.5, eff_cmd: -3, vel_cmd: .25,
  motor_vel_rad_s: 2, mechanical_power_w: -5, stats: {
    window_s: 20, covered_s: 7.5, samples: 376,
    joint_torque_rms_nm: 9.5, joint_torque_peak_nm: 19,
    motor_torque_rms_nm: 5, motor_torque_peak_nm: 10,
    motor_speed_at_peak_rad_s: Math.PI, above_continuous_s: .42,
    above_vendor_rotating_s: 0, temperature_max_c: 72.1,
    tracking_error_rms_rad: .04, mechanical_power_mean_w: -2.5,
  } });
panel.update({ joints: [...names].reverse().map(sample), diagnostics: {
  window_s: 20, covered_s: 7.5, samples: 376, sample_rate_hz: 50,
  clock: "simulation time", torque_source: "MuJoCo applied actuator torque",
  stats_basis: "Received telemetry; sampled peaks, not physics-step peaks",
  resets: 0, dropped_samples: 0, gaps: 0,
} });
const tables = body.querySelectorAll("table");
assert.equal(tables.length, 2);
const rows = tables[0].querySelectorAll("tbody")[0].children;
assert.equal(rows.length, 3);
assert.equal(rows[2].children[0].textContent, "FL_kfe");
assert.equal(rows[2].children[1].textContent, "5.00");
assert.equal(rows[2].children[2].textContent, "10.00");
assert.equal(rows[2].children[3].textContent, "9.50");
assert.equal(rows[2].children[4].textContent, "19.00");
assert.equal(rows[2].children[5].textContent, "30.0");
assert.equal(rows[2].children[6].textContent, "0.42");
assert.equal(rows[2].children[7].textContent, "0.00");
assert.match(content(body), /7\.50.*20.*376.*50\.0/);
assert.match(content(body), /MuJoCo applied actuator torque/);
assert.match(content(body), /sampled peaks, not physics-step peaks/);
assert.match(content(body), /time-weighted/i);
assert.match(tables[0].children[0].innerHTML, /11 N·m/);
assert.match(tables[1].children[0].innerHTML, /feedforward/i);
assert.match(content(body), /feedforward.*not total.*torque/i);
const details = tables[1].querySelectorAll("tbody")[0].children[2].children;
assert.equal(details[1].textContent, "72.1");
assert.equal(details[2].textContent, "0.040");
assert.equal(details[3].textContent, "-2.50");
assert.equal(details[4].textContent, "55.0");
assert.equal(details[5].textContent, "2.50");
assert.equal(details[6].textContent, "-3.00");

panel.update({ joints: [sample(names[0])], diagnostics: { samples: 2 } });
assert.equal(rows[2].children[1].textContent, "--", "Missing joints must not retain stale RMS");

// A new/backend-reset sample has no elapsed duration; unknown is never zero RMS.
panel.update({ joints: names.map(name => ({ name })), diagnostics: {} });
assert.equal(rows[2].children[1].textContent, "--");
assert.equal(rows[2].children[5].textContent, "--");
assert.equal(details[1].textContent, "--");
assert.match(content(body), /waiting for.*statistics/i);

const robotBody = new Element("div");
sandbox.panels.robot.render(robotBody, ctx);
assert.match(content(robotBody), /19\.719 kg/);
assert.match(content(robotBody), /384 × 220 × 120 mm/);
assert.match(content(robotBody), /582 × 289 mm/);
assert.match(content(robotBody), /200 × 200 mm/);
assert.match(content(robotBody), /8\.0.*11\.0.*36\.0/);
assert.match(content(robotBody), /cooling.*unverified/i);

const stateBody = new Element("div");
const statePanel = sandbox.panels.state.render(stateBody, ctx);
statePanel.update({ state: "MOVING", base: { height: .3, pos: [1, 2, .3],
  quat: [0, 0, 0, 1], vel: [.3, .4, 0], omega: [0, 0, .1] },
  joints: names.map(name => ({ name, enabled: true })),
  feet: [{ contact: true }, { contact: true }, { contact: false }, { contact: false }],
  power: { watts: 40, voltage: 44.4 } });
assert.match(content(stateBody), /1\.000, 2\.000, 0\.300 m/);
assert.match(content(stateBody), /0\.50 m\/s/);
assert.match(content(stateBody), /2 \/ 4/);
statePanel.update({ state: "MOVING", base: { height: null, pos: [null, null, null],
  quat: [null, null, null, null], vel: [null, null, null], omega: [null, null, null] },
  feet: [], joints: [], power: { watts: null, voltage: null } });
assert.match(content(stateBody), /-- mm/);
assert.match(content(stateBody), /speed -- m\/s/);
assert.match(content(stateBody), /-- \/ -- \/ --°/);
assert.doesNotMatch(content(stateBody), /0\.00 m\/s|NaN/);
