/* Panel registry.
 *
 * Every panel is {render(body, ctx)} -> optional {update(state, ctx)}.
 * `render` runs once when the page builds itself from /api/config; `update`
 * runs on every telemetry frame. Keeping the two separate is what lets the UI
 * redraw at 20 Hz without rebuilding the DOM, which is the difference between
 * a readable table and a flickering one.
 *
 * Adding a panel: write it here, then add an entry to
 * robodog_web/config/web.yaml. No other file changes.
 */
const Panels = {};

/* ---------- small helpers ---------- */
const el = (tag, cls, text) => {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
};
const fmt = (v, d = 3) => (v === undefined || v === null || Number.isNaN(v))
  ? "--" : Number(v).toFixed(d);
const deg = (r) => fmt(r * 180 / Math.PI, 1);
const pct = (v) => fmt(v * 100, 0) + "%";

function kv(pairs) {
  const dl = el("dl", "kv");
  for (const [k, v] of pairs) { dl.appendChild(el("dt", null, k)); dl.appendChild(el("dd", null, v)); }
  return dl;
}
function setKv(dl, values) {
  const dds = dl.querySelectorAll("dd");
  values.forEach((v, i) => { if (dds[i]) dds[i].textContent = v; });
}
function level(value, warn, alarm) {
  if (value >= alarm) return "alarm";
  if (value >= warn) return "warn";
  return "ok";
}
function bar(cls) {
  const b = el("div", "bar " + (cls || "")); b.appendChild(el("i")); return b;
}
function setBar(b, frac, cls) {
  b.firstChild.style.width = Math.max(0, Math.min(1, frac)) * 100 + "%";
  b.className = "bar " + (cls || "");
}

/* =========================== EMERGENCY STOP =========================== */
Panels.estop = {
  render(body, ctx) {
    const stop = el("button", "btn danger estop-big", "EMERGENCY STOP");
    const clear = el("button", "btn wide", "Clear E-Stop");
    const status = el("div", "badge idle", "--");
    const why = el("div", "muted");
    const enable = el("button", "btn", "Enable joints");
    const disable = el("button", "btn", "Disable joints");
    stop.onclick = () => ctx.send({ action: "estop", reason: "web GUI operator" });
    clear.onclick = () => ctx.send({ action: "clear_estop" });
    enable.onclick = () => ctx.send({ action: "enable", enable: true });
    disable.onclick = () => ctx.send({ action: "enable", enable: false });
    const wrap = el("div", "stack");
    wrap.append(stop, status, why, clear, el("div", "btnrow"));
    wrap.lastChild.append(enable, disable);
    body.appendChild(wrap);
    return {
      update(s) {
        const sf = s.safety;
        status.textContent = sf.latched ? "LATCHED" : (sf.estop ? "ENGAGED" : "CLEAR");
        status.className = "badge " + (sf.estop ? "alarm" : "ok");
        why.textContent = sf.estop ? (sf.source || "") : "";
        clear.disabled = !sf.latched;
      }
    };
  }
};

/* ============================ ROBOT STATE ============================ */
Panels.state = {
  render(body) {
    const badge = el("div", "badge idle", "--");
    badge.style.fontSize = "15px";
    const dl = kv([["height", "--"], ["vx", "--"], ["vy", "--"], ["yaw rate", "--"],
                   ["roll", "--"], ["pitch", "--"], ["power", "--"], ["bus", "--"]]);
    const wrap = el("div", "stack"); wrap.append(badge, dl); body.appendChild(wrap);
    return {
      update(s) {
        badge.textContent = s.state;
        badge.className = "badge " + ({ FAULT: "alarm", ESTOP: "alarm", IDLE: "idle",
                                        MOVING: "ok", STANDING: "ok", READY: "ok" }[s.state] || "idle");
        const q = s.base.quat;
        const roll = Math.atan2(2 * (q[3] * q[0] + q[1] * q[2]), 1 - 2 * (q[0] ** 2 + q[1] ** 2));
        const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (q[3] * q[1] - q[2] * q[0]))));
        setKv(dl, [fmt(s.base.height * 1000, 0) + " mm",
                   fmt(s.base.vel[0], 2) + " m/s", fmt(s.base.vel[1], 2) + " m/s",
                   fmt(s.base.omega[2], 2) + " rad/s",
                   deg(roll) + "°", deg(pitch) + "°",
                   fmt(s.power.watts, 1) + " W", fmt(s.power.voltage, 1) + " V"]);
      }
    };
  }
};

/* ============================= CONTROLLER ============================= */
Panels.controller = {
  render(body) {
    const dl = kv([["active", "--"], ["mode", "--"], ["gait", "--"], ["pose", "--"],
                   ["loop", "--"], ["jitter", "--"], ["trajectory", "--"]]);
    const prog = bar("ok");
    const wrap = el("div", "stack"); wrap.append(dl, prog); body.appendChild(wrap);
    return {
      update(s) {
        const c = s.controller, sf = s.safety;
        setKv(dl, [c.active, c.mode, c.gait || "--", c.pose || "--",
                   fmt(c.rate_hz, 0) + " Hz",
                   fmt(sf.loop_jitter * 1000, 2) + " ms",
                   c.traj_active ? pct(c.traj_progress) : "idle"]);
        setBar(prog, c.traj_active ? c.traj_progress : 0, "ok");
      }
    };
  }
};

/* =============================== SAFETY =============================== */
Panels.safety = {
  render(body, ctx) {
    const dl = kv([["max temp", "--"], ["hottest", "--"], ["max torque", "--"],
                   ["most loaded", "--"], ["clamps", "--"], ["watchdog", "--"],
                   ["cmd age", "--"]]);
    const tbar = bar(), qbar = bar();
    const faults = el("div", "btnrow");
    const wrap = el("div", "stack");
    wrap.append(dl, el("div", "muted", "torque"), qbar,
                el("div", "muted", "temperature"), tbar, faults);
    body.appendChild(wrap);
    return {
      update(s) {
        const sf = s.safety, th = ctx.cfg.thresholds;
        setKv(dl, [fmt(sf.max_temp, 1) + " °C", sf.hottest || "--",
                   pct(sf.max_util), sf.most_loaded || "--", sf.clamps,
                   sf.watchdog_ok ? "ok" : "STALE",
                   fmt(sf.command_age * 1000, 0) + " ms"]);
        setBar(qbar, sf.max_util, level(sf.max_util, th.torque_warn, th.torque_alarm));
        const tfrac = (sf.max_temp - 20) / (th.temperature_alarm_c - 20);
        setBar(tbar, tfrac, level(sf.max_temp, th.temperature_warn_c, th.temperature_alarm_c));
        faults.textContent = "";
        if (!sf.faults.length) faults.appendChild(el("span", "badge ok", "no faults"));
        else sf.faults.forEach(f => faults.appendChild(el("span", "badge alarm", f)));
      }
    };
  }
};

/* ================================ POSES ================================ */
Panels.pose = {
  render(body, ctx) {
    const row = el("div", "btnrow");
    (ctx.info.poses || []).forEach(p => {
      const b = el("button", "btn", p);
      b.onclick = () => ctx.send({ action: "pose", pose: p });
      row.appendChild(b);
    });
    const note = el("div", "muted",
      "moves every joint on a minimum-jerk profile, speed-limited");
    const wrap = el("div", "stack"); wrap.append(row, note); body.appendChild(wrap);
    return {
      update(s) {
        [...row.children].forEach(b =>
          b.classList.toggle("on", b.textContent === s.controller.pose));
      }
    };
  }
};

/* =========================== GAIT + VELOCITY =========================== */
Panels.gait = {
  render(body, ctx) {
    const lim = ctx.cfg.limits;
    const gaits = ["stand", "walk", "trot", "pace", "bound"];
    const row = el("div", "btnrow");
    let active = "stand";
    gaits.forEach(g => {
      const b = el("button", "btn", g);
      b.onclick = () => {
        if (lim.require_confirm_for_gait && g !== "stand" &&
            !confirm(`Start the '${g}' gait?\n\nThe robot will begin stepping.`)) return;
        active = g;
        ctx.send({ action: "gait", gait: g, enable: true });
      };
      row.appendChild(b);
    });

    const mk = (label, id, max, step, unit) => {
      const s = el("div", "slider");
      const inp = el("input"); inp.type = "range"; inp.min = -max; inp.max = max;
      inp.step = step; inp.value = 0; inp.id = id;
      const out = el("output", null, "0.00 " + unit);
      inp.oninput = () => { out.textContent = Number(inp.value).toFixed(2) + " " + unit; };
      s.append(el("label", null, label), inp, out);
      return { node: s, input: inp, output: out };
    };
    const vx = mk("forward", "vx", lim.max_linear_velocity, 0.02, "m/s");
    const vy = mk("lateral", "vy", lim.max_linear_velocity, 0.02, "m/s");
    const wz = mk("turn", "wz", lim.max_angular_velocity, 0.05, "rad/s");
    const stop = el("button", "btn wide", "zero velocity");
    const zero = () => {
      [vx, vy, wz].forEach(s => { s.input.value = 0; s.input.oninput(); });
      ctx.send({ action: "cmd_vel", vx: 0, vy: 0, wz: 0 });
    };
    stop.onclick = zero;
    const push = () => ctx.send({
      action: "cmd_vel", vx: +vx.input.value, vy: +vy.input.value, wz: +wz.input.value });
    [vx, vy, wz].forEach(s => s.input.onchange = push);
    // Hold-to-drive: releasing the mouse anywhere returns the robot to zero,
    // so a dropped connection or a slipped click cannot leave it walking.
    [vx, vy, wz].forEach(s => s.input.addEventListener("pointerup", push));

    const wrap = el("div", "stack");
    wrap.append(row, vx.node, vy.node, wz.node, stop);
    body.appendChild(wrap);
    ctx.onDisconnect(zero);
    return {
      update(s) {
        active = s.controller.gait || active;
        [...row.children].forEach(b => b.classList.toggle("on", b.textContent === active));
      }
    };
  }
};

/* =============================== JOINTS =============================== */
Panels.joints = {
  render(body, ctx) {
    const table = el("table");
    const head = el("thead");
    head.innerHTML = "<tr><th>joint</th><th>pos</th><th>cmd</th><th>err</th>" +
      "<th>vel</th><th>torque</th><th>load</th><th>curr</th><th>temp</th>" +
      "<th>mode</th><th>status</th></tr>";
    const tbody = el("tbody");
    table.append(head, tbody);
    body.appendChild(table);
    const rows = ctx.info.joints.map((name, i) => {
      const tr = el("tr");
      if (i % 3 === 0 && i > 0) tr.className = "leg-sep";
      const cells = [];
      for (let c = 0; c < 11; c++) { const td = el("td"); tr.appendChild(td); cells.push(td); }
      cells[0].textContent = name.replace("_joint", "");
      const lbar = bar(); cells[6].appendChild(lbar);
      tbody.appendChild(tr);
      return { cells, lbar };
    });
    return {
      update(s) {
        const th = ctx.cfg.thresholds;
        s.joints.forEach((j, i) => {
          const { cells, lbar } = rows[i];
          cells[1].textContent = fmt(j.pos, 3);
          cells[2].textContent = fmt(j.pos_cmd, 3);
          cells[3].textContent = fmt(j.err, 3);
          cells[3].style.color = Math.abs(j.err) > th.tracking_error_warn_rad
            ? "var(--warn)" : "";
          cells[4].textContent = fmt(j.vel, 2);
          cells[5].textContent = fmt(j.eff, 2);
          setBar(lbar, j.util, level(j.util, th.torque_warn, th.torque_alarm));
          cells[7].textContent = fmt(j.cur, 2);
          cells[8].textContent = fmt(j.temp, 1);
          cells[8].style.color = j.temp >= th.temperature_alarm_c ? "var(--alarm)"
            : j.temp >= th.temperature_warn_c ? "var(--warn)" : "";
          cells[9].textContent = j.mode;
          cells[10].textContent = "";
          if (j.faults.length) cells[10].appendChild(el("span", "badge alarm", j.faults[0]));
          else cells[10].appendChild(el("span", "badge " + (j.enabled ? "ok" : "idle"),
                                        j.enabled ? "on" : "off"));
        });
      }
    };
  }
};

/* ================================ FEET ================================ */
Panels.feet = {
  render(body, ctx) {
    const table = el("table");
    table.innerHTML = "<thead><tr><th>foot</th><th>contact</th><th>force</th>" +
      "<th>x</th><th>y</th><th>z</th></tr></thead>";
    const tbody = el("tbody"); table.appendChild(tbody);
    const rows = (ctx.info.legs || []).map(() => {
      const tr = el("tr");
      const cells = [];
      for (let c = 0; c < 6; c++) { const td = el("td"); tr.appendChild(td); cells.push(td); }
      tbody.appendChild(tr); return cells;
    });
    const total = el("div", "muted");
    body.append(table, total);
    return {
      update(s) {
        let sum = 0;
        s.feet.forEach((f, i) => {
          const c = rows[i]; if (!c) return;
          c[0].textContent = f.name;
          c[1].textContent = "";
          c[1].appendChild(el("span", "badge " + (f.contact ? "ok" : "idle"),
                              f.contact ? "down" : "swing"));
          c[2].textContent = fmt(f.force, 1) + " N";
          c[3].textContent = fmt(f.pos[0], 3);
          c[4].textContent = fmt(f.pos[1], 3);
          c[5].textContent = fmt(f.pos[2], 3);
          sum += f.force;
        });
        const w = (ctx.info.mass_kg || 10) * 9.81;
        total.textContent = `total ${fmt(sum, 1)} N of ${fmt(w, 1)} N weight `
          + `(${pct(sum / w)})`;
      }
    };
  }
};

/* =============================== CAMERA =============================== */
Panels.camera = {
  render(body) {
    const frame = el("div", "videoframe");
    const placeholder = el("div", null, "waiting for camera…");
    frame.appendChild(placeholder);
    const dl = kv([["backend", "--"], ["resolution", "--"], ["encoding", "--"],
                   ["frame", "--"]]);
    const wrap = el("div", "stack"); wrap.append(frame, dl); body.appendChild(wrap);
    let attached = false;
    return {
      update(s) {
        const c = s.camera || {};
        setKv(dl, [s.simulation.active ? "simulated" : "hardware",
                   c.width ? `${c.width}×${c.height}` : "--",
                   c.encoding || "--", c.frame || "--"]);
        if (c.available && !attached) {
          attached = true;
          const img = el("img");
          // Cache-buster: some browsers will reuse a previous MJPEG response.
          img.src = "/stream/color.mjpg?t=" + Date.now();
          img.onerror = () => {
            attached = false; frame.textContent = "";
            frame.appendChild(el("div", null,
              "no JPEG encoder on the robot — install Pillow to stream video"));
          };
          frame.textContent = ""; frame.appendChild(img);
        }
      }
    };
  }
};

/* ============================= SIMULATION ============================= */
Panels.simulation = {
  render(body) {
    const dl = kv([["backend", "--"], ["world", "--"], ["sim time", "--"],
                   ["real-time", "--"], ["timestep", "--"], ["steps", "--"]]);
    const rtf = bar();
    const wrap = el("div", "stack"); wrap.append(dl, rtf); body.appendChild(wrap);
    return {
      update(s) {
        const m = s.simulation;
        if (!m.active) {
          setKv(dl, ["hardware", "—", "—", "—", "—", "—"]);
          setBar(rtf, 0, "idle");
          return;
        }
        setKv(dl, [m.backend, m.world || "--", fmt(m.sim_time, 1) + " s",
                   fmt(m.rtf, 2) + "×", fmt(m.timestep * 1000, 2) + " ms",
                   m.steps]);
        setBar(rtf, m.rtf, m.rtf >= 0.9 ? "ok" : m.rtf >= 0.5 ? "warn" : "alarm");
      }
    };
  }
};

/* =============================== ERRORS =============================== */
Panels.errors = {
  render(body) {
    const list = el("div", "events");
    const none = el("div", "muted", "no events");
    body.append(list, none);
    return {
      update(s, ctx, events) {
        list.textContent = "";
        const all = events || [];
        none.style.display = all.length ? "none" : "";
        all.forEach(e => {
          const row = el("div", "event " + e.kind);
          const t = el("time", null, new Date(e.t * 1000).toLocaleTimeString());
          row.append(t, el("span", "badge " + (e.kind === "fault" ? "alarm" : "idle"), e.kind),
                     el("span", null, e.text));
          list.appendChild(row);
        });
      }
    };
  }
};

/* ============================= JOINT TEST ============================= */
Panels.jointtest = {
  render(body, ctx) {
    const lim = ctx.cfg.limits;
    if (!lim.enable_joint_jog) {
      body.appendChild(el("div", "disabled-note",
        "Joint jogging is disabled. It bypasses the gait layer and can place a "
        + "leg where the body cannot support it. Set limits.enable_joint_jog "
        + "in robodog_web/config/web.yaml to enable."));
      return null;
    }
    const sel = el("select");
    ctx.info.joints.forEach(j => sel.appendChild(new Option(j.replace("_joint", ""), j)));
    const minus = el("button", "btn", "− step");
    const plus = el("button", "btn", "+ step");
    const readout = el("div", "mid", "--");
    minus.onclick = () => ctx.send({ action: "jog", joint: sel.value, delta: -lim.manual_joint_step_rad });
    plus.onclick = () => ctx.send({ action: "jog", joint: sel.value, delta: +lim.manual_joint_step_rad });
    const rowb = el("div", "btnrow"); rowb.append(minus, plus);
    const wrap = el("div", "stack"); wrap.append(sel, rowb, readout);
    body.appendChild(wrap);
    return {
      update(s) {
        const i = ctx.info.joints.indexOf(sel.value);
        if (i >= 0) {
          const j = s.joints[i];
          readout.textContent = `${fmt(j.pos, 4)} rad  (${deg(j.pos)}°)  `
            + `τ ${fmt(j.eff, 2)} N·m`;
        }
      }
    };
  }
};
