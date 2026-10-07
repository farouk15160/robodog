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
const fmt = (v, d = 3) => (v === undefined || v === null || !Number.isFinite(Number(v)))
  ? "--" : Number(v).toFixed(d);
const finite = v => typeof v === "number" && Number.isFinite(v);
const scaled = (v, factor, digits) => fmt(finite(v) ? v * factor : null, digits);
const deg = r => scaled(r, 180 / Math.PI, 1);
const pct = v => finite(v) ? fmt(v * 100, 0) + "%" : "--";

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
  b.firstChild.style.width = Math.max(0, Math.min(1, finite(frac) ? frac : 0)) * 100 + "%";
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
    const dl = kv([["height", "--"], ["position xyz", "--"], ["speed", "--"],
                   ["velocity xyz", "--"], ["roll / pitch / yaw", "--"], ["yaw rate", "--"],
                   ["feet touching", "--"], ["joints enabled", "--"],
                   ["est electrical power", "--"], ["assumed bus", "--"]]);
    const wrap = el("div", "stack"); wrap.append(badge, dl); body.appendChild(wrap);
    return {
      update(s) {
        badge.textContent = s.state;
        badge.className = "badge " + ({ FAULT: "alarm", ESTOP: "alarm", IDLE: "idle",
                                        MOVING: "ok", STANDING: "ok", READY: "ok" }[s.state] || "idle");
        const q = s.base.quat;
        const validRotation = q.length === 4 && q.every(finite) && Math.hypot(...q) > 0;
        const roll = validRotation ? Math.atan2(2 * (q[3] * q[0] + q[1] * q[2]), 1 - 2 * (q[0] ** 2 + q[1] ** 2)) : null;
        const pitch = validRotation ? Math.asin(Math.max(-1, Math.min(1, 2 * (q[3] * q[1] - q[2] * q[0])))) : null;
        const yaw = validRotation ? Math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)) : null;
        const speed = s.base.vel.length === 3 && s.base.vel.every(finite) ? Math.hypot(...s.base.vel) : null;
        setKv(dl, [scaled(s.base.height, 1000, 0) + " mm",
                   s.base.pos.map(v => fmt(v)).join(", ") + " m",
                   fmt(speed, 2) + " m/s",
                   s.base.vel.map(v => fmt(v, 2)).join(", ") + " m/s",
                   [roll, pitch, yaw].map(deg).join(" / ") + "°",
                   fmt(s.base.omega[2], 2) + " rad/s",
                   `${s.feet.filter(f => f.contact).length} / ${s.feet.length}`,
                   `${s.joints.filter(j => j.enabled).length} / ${s.joints.length}`,
                   fmt(s.power.watts, 1) + " W", fmt(s.power.voltage, 1) + " V"]);
      }
    };
  }
};

/* ============================= ROBOT MODEL ============================= */
Panels.robot = {
  render(body, ctx) {
    const info = ctx.info, g = info.geometry || {}, p = info.performance || {};
    const e = info.electrical || {}, mass = info.mass_budget || {};
    const knee = info.joint_actuators?.FL_kfe_joint;
    const mm = values => values?.length ? values.map(v => fmt(v * 1000, 0)).join(" × ") + " mm" : "--";
    const heatSink = p.vendor_curve?.heat_sink_dimensions_mm;
    const dl = kv([
      ["model / motors", `${info.robot || "--"} / ${info.joints.length} × ${info.actuator || "--"}`],
      ["working mass / weight", `${fmt(info.mass_kg)} kg / ${fmt(info.mass_kg * 9.81, 1)} N`],
      ["body collision box", mm(g.body_box_m)],
      ["nominal footprint", g.nominal_footprint_m ? mm([g.nominal_footprint_m.length, g.nominal_footprint_m.width]) : "--"],
      ["thigh / shank", `${fmt(g.thigh_length_m * 1000, 1)} / ${fmt(g.shank_length_m * 1000, 1)} mm`],
      ["motor stall / rated / peak", `${fmt(p.stall_continuous_torque_nm, 1)} / ${fmt(p.rated_torque_nm, 1)} / ${fmt(p.peak_torque_nm, 1)} N·m`],
      ["rated rotating point", `${fmt(p.rated_speed_rpm, 0)} rpm; ${heatSink ? heatSink.join(" × ") + " mm heat sink" : "see motor specification"}`],
      ["knee belt / efficiency", knee ? `${fmt(knee.ratio, 0)}:1 / ${pct(knee.efficiency)} assumed` : "--"],
      ["nominal bus / motor rated", `${fmt(e.supply_voltage_v, 1)} / ${fmt(e.rated_voltage_v, 1)} V`],
      ["output torque constant", `${fmt(e.torque_constant_nm_per_arms, 2)} N·m/A rms`],
      ["CAD export / motors", `${fmt(mass.cad_export_kg)} / ${fmt(mass.actuators_kg)} kg`],
      ["electronics / payload", `${fmt(mass.electronics_payload_kg)} kg`],
    ]);
    const wrap = el("div", "stack");
    wrap.append(dl, el("div", "muted", "Configured model values; payload placement and installed motor cooling remain unverified. The rotating rating is conditional; it is not a continuous stall allowance."),
      el("div", "muted", info.actuator_revision || "Motor specification revision unavailable."));
    body.append(wrap);
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
                   scaled(sf.loop_jitter, 1000, 2) + " ms",
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
                   ["most loaded", "--"], ["aggregate safety clamps", "--"], ["watchdog", "--"],
                   ["cmd age", "--"]]);
    const tbar = bar(), qbar = bar();
    const faults = el("div", "btnrow");
    const wrap = el("div", "stack");
    wrap.append(dl, el("div", "muted", "torque"), qbar,
                el("div", "muted", "temperature"), tbar, faults,
                el("div", "muted", "Clamps are the cumulative safety-controller count across all joints, not per-joint actuator clipping."));
    body.appendChild(wrap);
    return {
      update(s) {
        const sf = s.safety, th = ctx.cfg.thresholds;
        setKv(dl, [fmt(sf.max_temp, 1) + " °C", sf.hottest || "--",
                   pct(sf.max_util), sf.most_loaded || "--", sf.clamps,
                   sf.watchdog_ok ? "ok" : "STALE",
                   scaled(sf.command_age, 1000, 0) + " ms"]);
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

/* ============================ REMOTE CONTROL ============================ */
Panels.remote = {
  render(body, ctx) {
    const limits = ctx.cfg.limits;
    let serverLinearLimit = Number(limits.max_linear_velocity);
    let linearLimit = Math.min(.5, serverLinearLimit);
    let turnLimit = Number(limits.max_angular_velocity);
    const drive = { leftX: 0, leftY: 0, turnX: 0, keys: new Set(), pointers: new Set() };

    const feedback = kv([
      ["state / gait", "--"], ["speed / yaw rate", "--"],
      ["position xyz", "--"], ["maximum joint load", "--"],
      ["hottest motor", "--"], ["backend / world", "--"],
      ["mapping", "waiting for telemetry"],
    ]);
    feedback.className += " remote-feedback";
    const video = el("div", "videoframe remote-video");
    const image = el("img");
    image.src = "/api/video";
    image.alt = "Live robot camera";
    video.append(image);

    const makeStick = (label, kind) => {
      let pointerId = null;
      const pad = el("div", "joystick-pad");
      pad.setAttribute("role", "application");
      pad.setAttribute("tabindex", "0");
      pad.setAttribute("aria-label", label);
      pad.dataset.kind = kind;
      const knob = el("span", "joystick-knob");
      pad.append(knob);
      const move = event => {
        event.preventDefault();
        const rect = pad.getBoundingClientRect();
        const x = Math.max(-1, Math.min(1, (event.clientX - rect.left - rect.width / 2) /
          (rect.width * .36)));
        const y = Math.max(-1, Math.min(1, (event.clientY - rect.top - rect.height / 2) /
          (rect.height * .36)));
        knob.style.transform = `translate(${x * 70}%, ${y * 70}%)`;
        if (kind === "translation") {
          drive.leftX = x; drive.leftY = y;
        } else {
          drive.turnX = x;
        }
        publishVelocity();
      };
      const release = (event, publish = true) => {
        if (event && (pointerId === null || event.pointerId !== pointerId)) return;
        if (event) event.preventDefault();
        if (kind === "translation") { drive.leftX = 0; drive.leftY = 0; }
        else drive.turnX = 0;
        knob.style.transform = "translate(0, 0)";
        const captured = pointerId;
        if (captured !== null) drive.pointers.delete(captured);
        pointerId = null;
        if (captured !== null) {
          try { pad.releasePointerCapture?.(captured); } catch { /* already released */ }
        }
        if (publish) publishVelocity();
      };
      pad.addEventListener("pointerdown", event => {
        if (pointerId !== null) return;
        pointerId = event.pointerId;
        drive.pointers.add(pointerId);
        pad.setPointerCapture?.(pointerId); move(event);
      });
      pad.addEventListener("pointermove", event => {
        if (event.pointerId === pointerId) move(event);
      });
      ["pointerup", "pointercancel", "lostpointercapture"].forEach(type =>
        pad.addEventListener(type, release));
      return { pad, release };
    };
    const translation = makeStick("Forward and lateral joystick", "translation");
    const turning = makeStick("Turn joystick", "turn");
    const sticks = el("div", "joystick-grid remote-sticks");
    const stickColumn = (title, stick, hint) => {
      const column = el("div", "joystick-column");
      column.append(el("strong", null, title), stick.pad, el("div", "muted", hint));
      return column;
    };
    sticks.append(stickColumn("Move", translation, "drag: forward / lateral"),
      stickColumn("Turn", turning, "drag left / right"));

    const currentVelocity = () => {
      let vx = -drive.leftY * linearLimit;
      let vy = -drive.leftX * linearLimit;
      let wz = -drive.turnX * turnLimit;
      if (drive.keys.has("KeyW") || drive.keys.has("ArrowUp")) vx += linearLimit;
      if (drive.keys.has("KeyS") || drive.keys.has("ArrowDown")) vx -= linearLimit;
      if (drive.keys.has("KeyA")) vy += linearLimit;
      if (drive.keys.has("KeyD")) vy -= linearLimit;
      if (drive.keys.has("ArrowLeft") || drive.keys.has("KeyQ")) wz += turnLimit;
      if (drive.keys.has("ArrowRight") || drive.keys.has("KeyE")) wz -= turnLimit;
      return {
        vx: Math.max(-linearLimit, Math.min(linearLimit, vx)),
        vy: Math.max(-linearLimit, Math.min(linearLimit, vy)),
        wz: Math.max(-turnLimit, Math.min(turnLimit, wz)),
      };
    };
    function publishVelocity() {
      const value = currentVelocity();
      ctx.send({ action: "cmd_vel", vx: value.vx, vy: value.vy, wz: value.wz });
    }
    const zero = () => {
      drive.keys.clear(); drive.leftX = 0; drive.leftY = 0; drive.turnX = 0;
      drive.pointers.clear();
      translation.release(null, false); turning.release(null, false);
      ctx.send({ action: "cmd_vel", vx: 0, vy: 0, wz: 0 });
    };

    const driveCodes = new Set(["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE",
      "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"]);
    document.addEventListener("keydown", event => {
      if (!driveCodes.has(event.code) || event.repeat ||
          ["INPUT", "SELECT", "TEXTAREA"].includes(event.target?.tagName) ||
          (ctx.isPageActive && !ctx.isPageActive("remote"))) return;
      event.preventDefault(); drive.keys.add(event.code); publishVelocity();
    });
    document.addEventListener("keyup", event => {
      if (!driveCodes.has(event.code) || !drive.keys.has(event.code)) return;
      event.preventDefault(); drive.keys.delete(event.code); publishVelocity();
    });
    window.addEventListener("blur", zero);
    document.addEventListener("visibilitychange", () => { if (document.hidden) zero(); });
    ctx.onDisconnect(zero);
    ctx.onPageChange?.((previous, next) => { if (previous === "remote" && next !== "remote") zero(); });
    setInterval(() => {
      if ((drive.pointers.size || drive.keys.size) &&
          (!ctx.isPageActive || ctx.isPageActive("remote"))) publishVelocity();
    }, 100);

    const speed = el("div", "slider remote-speed");
    const speedLabel = el("label", null, "max speed");
    const speedLimit = el("input");
    speedLimit.type = "range"; speedLimit.min = .1; speedLimit.max = serverLinearLimit;
    speedLimit.step = .1; speedLimit.value = linearLimit;
    speedLimit.setAttribute("aria-label", "Maximum driving speed");
    const speedOutput = el("output", null, `${fmt(linearLimit, 2)} m/s / ${fmt(turnLimit, 2)} rad/s`);
    speedLimit.oninput = () => {
      linearLimit = Math.max(.1, Math.min(serverLinearLimit, Number(speedLimit.value)));
      speedOutput.textContent = `${fmt(linearLimit, 2)} m/s / ${fmt(turnLimit, 2)} rad/s`;
    };
    speed.append(speedLabel, speedLimit, speedOutput);

    const actions = el("div", "btnrow remote-actions");
    const walk = el("button", "btn primary", "Start travel (trot)");
    const stand = el("button", "btn", "Stand");
    const greeting = el("button", "btn", "Greeting");
    const stop = el("button", "btn danger", "STOP MOVEMENT");
    walk.onclick = () => ctx.send({ action: "gait", gait: "trot", enable: true });
    stand.onclick = () => { ctx.send({ action: "gait", gait: "stand", enable: true }); zero(); };
    greeting.onclick = () => ctx.send({ action: "greeting" });
    stop.onclick = () => { ctx.send({ action: "gait", gait: "stand", enable: true }); zero(); };
    actions.append(walk, stand, greeting, stop);

    const scan = el("div", "scan-save");
    const scanName = el("input");
    scanName.type = "text"; scanName.maxLength = 64; scanName.placeholder = "optional scan name";
    scanName.setAttribute("aria-label", "Room scan name");
    scanName.setAttribute("pattern", "[A-Za-z0-9][A-Za-z0-9._-]{0,63}");
    const save = el("button", "btn", "Save room scan");
    const saveStatus = el("div", "muted scan-status", "Saves the accumulated point cloud and OctoMap.");
    save.onclick = () => {
      const name = String(scanName.value || "").trim();
      if (name && !/^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/.test(name)) {
        saveStatus.textContent = "Use 1–64 letters, numbers, dot, underscore, or dash.";
        saveStatus.className = "scan-status alarm-text";
        return;
      }
      save.disabled = true; saveStatus.textContent = "Saving room scan…";
      saveStatus.className = "muted scan-status";
      ctx.send({ action: "save_map", name });
    };
    ctx.onAck?.(ack => {
      if (ack.action !== "save_map") return;
      save.disabled = false;
      saveStatus.textContent = ack.ok
        ? `Saved: ${ack.data?.manifest_path || ack.message}` : `Save failed: ${ack.message}`;
      saveStatus.className = `scan-status ${ack.ok ? "ok-text" : "alarm-text"}`;
    });
    scan.append(scanName, save);

    const keyboard = el("div", "muted remote-help",
      "Laptop: W/S forward, A/D lateral, Q/E or ←/→ turn. Hold keys or a joystick to move; release, focus loss, hidden tab, or disconnect commands zero velocity.");
    const layout = el("div", "remote-layout");
    layout.append(actions, speed, video, sticks, keyboard, scan, saveStatus, feedback);
    body.append(layout);

    return {
      update(s) {
        serverLinearLimit = s.simulation.active
          ? Number(limits.simulation_max_linear_velocity ?? limits.max_linear_velocity)
          : Number(limits.max_linear_velocity);
        speedLimit.max = serverLinearLimit;
        if (Number(speedLimit.value) > serverLinearLimit) speedLimit.value = serverLinearLimit;
        linearLimit = Math.min(serverLinearLimit, Number(speedLimit.value));
        translation.pad.dataset.linearLimit = String(serverLinearLimit);
        turnLimit = Number(limits.max_angular_velocity);
        speedOutput.textContent = `${fmt(linearLimit, 2)} m/s / ${fmt(turnLimit, 2)} rad/s`;
        const velocity = s.base.vel?.every(finite) ? Math.hypot(...s.base.vel) : null;
        const mapping = s.mapping || {};
        setKv(feedback, [
          `${s.state} / ${s.controller.gait || "--"}`,
          `${fmt(velocity, 2)} m/s / ${fmt(s.base.omega?.[2], 2)} rad/s`,
          (s.base.pos || []).map(v => fmt(v, 2)).join(", ") + " m",
          `${pct(s.safety.max_util)} · ${s.safety.most_loaded || "--"}`,
          `${fmt(s.safety.max_temp, 1)} °C · ${s.safety.hottest || "--"}`,
          `${s.simulation.active ? s.simulation.backend : "hardware"} / ${s.simulation.world || "--"}`,
          mapping.status || mapping.message || (mapping.active ? "building map" : "telemetry unavailable"),
        ]);
        const locked = s.safety.estop || !s.safety.watchdog_ok;
        walk.disabled = locked; greeting.disabled = locked;
        if (locked && (drive.pointers.size || drive.keys.size)) zero();
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

    const rangeNote = el("div", "muted");
    const wrap = el("div", "stack");
    wrap.append(row, rangeNote, vx.node, vy.node, wz.node, stop);
    body.appendChild(wrap);
    ctx.onDisconnect(zero);
    return {
      update(s) {
        const simulation = s.simulation.active === true;
        const maximum = simulation
          ? (lim.simulation_max_linear_velocity ?? lim.max_linear_velocity)
          : lim.max_linear_velocity;
        for (const slider of [vx, vy]) {
          if (Number(slider.input.max) === maximum) continue;
          slider.input.min = -maximum;
          slider.input.max = maximum;
          slider.input.value = Math.max(-maximum, Math.min(maximum, Number(slider.input.value)));
          slider.input.oninput();
        }
        rangeNote.textContent = simulation
          ? `Experimental simulation range ±${fmt(maximum, 2)} m/s; achievable speed must be tested.`
          : `Hardware limit ±${fmt(maximum, 2)} m/s.`;
        active = s.controller.gait || active;
        [...row.children].forEach(b => b.classList.toggle("on", b.textContent === active));
      }
    };
  }
};

/* =============================== JOINTS =============================== */
Panels.joints = {
  render(body, ctx) {
    const note = el("div", "muted joint-note");
    const table = el("table");
    const head = el("thead");
    head.innerHTML = "<tr><th>joint</th><th>pos rad</th><th>cmd rad</th><th>err rad</th>" +
      "<th>vel rad/s</th><th>joint N·m</th><th>motor N·m</th>" +
      "<th>cont limit / peak N·m</th><th>load % limit</th><th>est phase A rms</th><th>temp °C</th>" +
      "<th>mode</th><th>status</th></tr>";
    const tbody = el("tbody");
    table.append(head, tbody);
    body.append(note, table);
    const rows = Object.fromEntries(ctx.info.joints.map((name, i) => {
      const tr = el("tr");
      if (i % 3 === 0 && i > 0) tr.className = "leg-sep";
      const cells = [];
      for (let c = 0; c < 13; c++) { const td = el("td"); tr.appendChild(td); cells.push(td); }
      cells[0].textContent = name.replace("_joint", "");
      const rating = ctx.info.joint_actuators?.[name];
      cells[7].textContent = rating
        ? `${fmt(rating.continuous_torque_nm, 1)} / ${fmt(rating.peak_torque_nm, 1)}` : "--";
      const load = el("span");
      const lbar = bar(); cells[8].append(load, lbar);
      tbody.appendChild(tr);
      return [name, { cells, lbar, load, rating }];
    }));
    return {
      update(s) {
        const th = ctx.cfg.thresholds;
        const knee = ctx.info.joint_actuators?.FL_kfe_joint;
        const continuousBasis = knee?.continuous_limit_basis || "configured limit";
        const belt = knee ? `Knee belt ${fmt(knee.ratio, 0)}:1, ` +
          `${pct(knee.efficiency)} assumed efficiency. ` : "";
        note.textContent = "Signed joint and motor-output torque; limits at the joint. " +
          belt + `Continuous limit: ${continuousBasis}. Load is instantaneous. ` +
          "Current: instantaneous τ/Kt estimate of phase-equivalent A rms, not a time-window RMS; excludes high-current saturation; not measured phase/battery/iq. " +
          "Temperature: " + (s.simulation.active
            ? "thermal estimate (uncalibrated), not measured; copper-loss model only."
            : "max(reported motor temperature, thermal estimate); observer is uncalibrated.");
        s.joints.forEach(j => {
          if (!rows[j.name]) return;
          const { cells, lbar, load, rating } = rows[j.name];
          cells[1].textContent = fmt(j.pos, 3);
          cells[2].textContent = fmt(j.pos_cmd, 3);
          cells[3].textContent = fmt(j.err, 3);
          cells[3].style.color = Math.abs(j.err) > th.tracking_error_warn_rad
            ? "var(--warn)" : "";
          cells[4].textContent = fmt(j.vel, 2);
          cells[5].textContent = fmt(j.eff, 2);
          const motorTorque = "motor_eff_nm" in j ? j.motor_eff_nm
            : rating && finite(j.eff) && finite(rating.torque_gain) && rating.torque_gain > 0
              ? j.eff / rating.torque_gain : null;
          cells[6].textContent = fmt(motorTorque, 2);
          load.textContent = pct(j.util);
          setBar(lbar, j.util, level(j.util, th.torque_warn, th.torque_alarm));
          cells[9].textContent = fmt(j.cur, 2);
          cells[10].textContent = fmt(j.temp, 1);
          cells[10].style.color = j.temp >= th.temperature_alarm_c ? "var(--alarm)"
            : j.temp >= th.temperature_warn_c ? "var(--warn)" : "";
          cells[11].textContent = j.mode;
          cells[12].textContent = "";
          if (j.faults.length) cells[12].appendChild(el("span", "badge alarm", j.faults[0]));
          else cells[12].appendChild(el("span", "badge " + (j.enabled ? "ok" : "idle"),
                                        j.enabled ? "on" : "off"));
        });
      }
    };
  }
};

/* ======================= ROLLING JOINT DIAGNOSTICS ======================= */
function jointTable(names, columns, className) {
  const table = el("table", className), head = el("thead"), tbody = el("tbody");
  // Headers are fixed UI strings; every telemetry value below uses textContent.
  head.innerHTML = "<tr>" + columns.map(c => `<th>${c}</th>`).join("") + "</tr>";
  const rows = Object.fromEntries(names.map((name, i) => {
    const row = el("tr", i > 0 && i % 3 === 0 ? "leg-sep" : "");
    const cells = columns.map(() => el("td", null, "--"));
    cells[0].textContent = name.replace("_joint", "");
    row.append(...cells); tbody.append(row);
    return [name, cells];
  }));
  table.append(head, tbody);
  return { table, rows };
}

Panels.jointstats = {
  render(body, ctx) {
    const status = el("div", "diagnostic-status", "Waiting for rolling statistics…");
    const source = el("div", "muted"), health = el("div", "muted");
    const torque = jointTable(ctx.info.joints, ["joint", "motor RMS N·m", "motor |peak| N·m",
      "joint RMS N·m", "joint |peak| N·m", "motor rpm @ |peak|",
      "above cont. s", "above 11 N·m s"], "joint-statistics");
    const details = jointTable(ctx.info.joints, ["joint", "window max °C", "error RMS rad",
      "mean mech. W", "live Kp N·m/rad", "live Kd N·m·s/rad", "live feedforward N·m",
      "live cmd rad/s", "coverage s / samples"], "joint-control-details");
    const expand = el("details", "joint-details");
    const scroll = el("div", "table-scroll"); scroll.append(details.table);
    expand.append(el("summary", null, "Temperature history, tracking and live controller commands"),
      el("div", "muted joint-note", "Temperature is the reported observer/feedback value, not a calibrated prediction. Mechanical power is signed joint torque × speed; negative means mechanical absorption, not measured battery regeneration. Feedforward is not total requested or applied torque; Kp/Kd add feedback torque."), scroll);
    const torqueScroll = el("div", "table-scroll"); torqueScroll.append(torque.table);
    body.append(status, source, health,
      el("div", "muted joint-note", "Time-weighted RMS and exposure use received telemetry. Absolute peaks can miss short impacts between samples. Exposure compares motor-output torque with the configured continuous limit and the conditional 11 N·m rotating reference. Motor speed at peak is signed. These rolling values do not establish thermal endurance."),
      torqueScroll, expand);
    return {
      update(s) {
        const d = s.diagnostics || {};
        status.textContent = d.samples
          ? `${fmt(d.covered_s, 2)} / ${fmt(d.window_s, 0)} s covered · ${d.samples} samples · ${fmt(d.sample_rate_hz, 1)} Hz · ${d.clock || "unknown clock"}`
          : "Waiting for rolling statistics…";
        source.textContent = `${d.torque_source || "Torque source unavailable"}. ${d.stats_basis || "Received telemetry; sampled peaks, not physics-step peaks"}.`;
        health.textContent = `Window resets ${d.resets ?? "--"} · rejected samples ${d.dropped_samples ?? "--"} · sampling gaps ${d.gaps ?? "--"}`;
        const joints = Object.fromEntries(s.joints.map(j => [j.name, j]));
        for (const name of ctx.info.joints) {
          const j = joints[name] || {};
          const t = torque.rows[name], c = details.rows[name];
          const st = j.stats || {};
          const rpm = st.motor_speed_at_peak_rad_s == null ? null : st.motor_speed_at_peak_rad_s * 60 / (2 * Math.PI);
          [st.motor_torque_rms_nm, st.motor_torque_peak_nm, st.joint_torque_rms_nm,
            st.joint_torque_peak_nm, rpm, st.above_continuous_s, st.above_vendor_rotating_s]
            .forEach((value, i) => { t[i + 1].textContent = fmt(value, i === 4 ? 1 : 2); });
          [st.temperature_max_c, st.tracking_error_rms_rad, st.mechanical_power_mean_w,
            j.kp, j.kd, j.eff_cmd, j.vel_cmd].forEach((value, i) => {
              c[i + 1].textContent = fmt(value, i === 1 ? 3 : (i === 0 || i === 3 ? 1 : 2));
            });
          c[8].textContent = `${fmt(st.covered_s, 2)} / ${st.samples ?? "--"}`;
          c[1].style.color = st.temperature_max_c >= ctx.cfg.thresholds.temperature_alarm_c ? "var(--alarm)"
            : st.temperature_max_c >= ctx.cfg.thresholds.temperature_warn_c ? "var(--warn)" : "";
          c[2].style.color = st.tracking_error_rms_rad > ctx.cfg.thresholds.tracking_error_warn_rad ? "var(--warn)" : "";
        }
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
