/* robodog web GUI -- bootstrap, transport and render loop.
 *
 * The page knows nothing about which panels exist: it fetches /api/config,
 * looks each panel id up in the Panels registry and renders what it finds.
 * Reordering the UI, or turning a panel off for an operator who should not see
 * it, is a change to robodog_web/config/web.yaml alone.
 */
(() => {
  "use strict";

  const state = {
    cfg: null, info: null, ws: null, live: [],
    seq: 0, lastFrame: 0, disconnectHooks: [], retry: 500,
  };

  const $ = (id) => document.getElementById(id);

  /* ------------------------------ transport ------------------------------ */
  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    state.ws = ws;

    ws.onopen = () => {
      state.retry = 500;
      setLink(true, "connected");
    };
    ws.onclose = () => {
      setLink(false, `disconnected — retrying in ${(state.retry / 1000).toFixed(1)} s`);
      state.disconnectHooks.forEach(fn => { try { fn(); } catch (e) { /* ignore */ } });
      setTimeout(connect, state.retry);
      // Back off to at most 5 s: a robot that is rebooting should not be hit
      // with a reconnect every 500 ms for minutes.
      state.retry = Math.min(state.retry * 1.6, 5000);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      if (msg.type === "state") {
        state.seq = msg.seq;
        state.lastFrame = performance.now();
        render(msg.data, msg.events || []);
      } else if (msg.type === "info") {
        state.info = msg.data;
      } else if (msg.type === "ack" && !msg.ok) {
        toast(`${msg.action}: ${msg.message}`);
      }
    };
  }

  function send(payload) {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
      toast("not connected");
      return;
    }
    state.ws.send(JSON.stringify({ type: "cmd", ...payload }));
  }

  function setLink(up, text) {
    const dot = $("link-dot");
    dot.className = "dot " + (up ? "on" : "off");
    $("foot-link").textContent = text;
  }

  function toast(text) {
    const f = $("foot-link");
    const prev = f.textContent;
    f.textContent = text;
    f.style.color = "var(--alarm)";
    setTimeout(() => { f.textContent = prev; f.style.color = ""; }, 4000);
  }

  /* -------------------------------- build -------------------------------- */
  async function boot() {
    const [cfg, info] = await Promise.all([
      fetch("/api/config").then(r => r.json()),
      fetch("/api/info").then(r => r.json()),
    ]);
    state.cfg = cfg;
    state.info = info;
    $("brand-sub").textContent =
      `${info.joints.length}-DOF · ${info.actuator} · ${info.mass_kg.toFixed(1)} kg`;
    $("foot-proto").textContent = `protocol v${cfg.protocol}`;

    const ctx = {
      cfg, info, send,
      onDisconnect: (fn) => state.disconnectHooks.push(fn),
    };
    const grid = $("grid");
    const tpl = $("tpl-panel");

    for (const spec of cfg.panels) {
      const impl = Panels[spec.id];
      const node = tpl.content.firstElementChild.cloneNode(true);
      node.classList.add("w" + (spec.width || 1));
      node.querySelector(".ptitle").textContent = spec.title || spec.id;
      const body = node.querySelector(".pbody");
      grid.appendChild(node);
      if (!impl) {
        body.appendChild(Object.assign(document.createElement("div"),
          { className: "muted", textContent: `no renderer for panel '${spec.id}'` }));
        continue;
      }
      try {
        const live = impl.render(body, ctx);
        if (live && live.update) state.live.push({ spec, live, node });
      } catch (err) {
        body.textContent = `panel '${spec.id}' failed: ${err.message}`;
        body.className += " muted";
      }
    }

    $("btn-estop-top").onclick = () => send({ action: "estop", reason: "web GUI toolbar" });
    $("btn-theme").onclick = () => {
      const root = document.documentElement;
      const next = root.dataset.theme === "light" ? "dark" : "light";
      root.dataset.theme = next;
      try { localStorage.setItem("robodog-theme", next); } catch { /* private mode */ }
    };
    try {
      const saved = localStorage.getItem("robodog-theme");
      if (saved) document.documentElement.dataset.theme = saved;
    } catch { /* private mode: keep the default */ }

    // Space is the fastest key to hit in a hurry.
    document.addEventListener("keydown", (e) => {
      if (e.code === "Space" && e.target.tagName !== "INPUT"
          && e.target.tagName !== "SELECT") {
        e.preventDefault();
        send({ action: "estop", reason: "web GUI keyboard" });
      }
    });

    connect();
    setInterval(watchdog, 500);
  }

  /* -------------------------------- render ------------------------------- */
  function render(data, events) {
    $("top-state").textContent = data.state;
    $("top-state").className = "badge " +
      ({ FAULT: "alarm", ESTOP: "alarm", IDLE: "idle" }[data.state] || "ok");
    $("top-controller").textContent = data.controller.active;
    $("top-rate").textContent = data.controller.rate_hz.toFixed(0) + " Hz";
    $("top-backend").textContent = data.simulation.active
      ? data.simulation.backend : "hardware";
    $("foot-seq").textContent = `frame ${state.seq}`;

    const ctx = { cfg: state.cfg, info: state.info, send };
    for (const { spec, live, node } of state.live) {
      try {
        live.update(data, ctx, events);
        node.querySelector(".pnote").textContent = "";
      } catch (err) {
        node.querySelector(".pnote").textContent = "render error";
        // One broken panel must not stop the rest of the page updating.
        console.error(`panel '${spec.id}':`, err);
      }
    }
  }

  /* Telemetry can stop arriving while the socket still looks open -- the
     control node may have died. Surface that rather than leaving stale
     numbers on screen looking authoritative. */
  function watchdog() {
    if (!state.lastFrame) return;
    const age = (performance.now() - state.lastFrame) / 1000;
    if (age > 2.0 && state.ws && state.ws.readyState === WebSocket.OPEN) {
      setLink(false, `no telemetry for ${age.toFixed(1)} s`);
      document.querySelectorAll(".panel").forEach(p => p.style.opacity = 0.5);
    } else if (age <= 2.0) {
      document.querySelectorAll(".panel").forEach(p => p.style.opacity = 1);
      if (state.ws && state.ws.readyState === WebSocket.OPEN) setLink(true, "connected");
    }
  }

  boot().catch(err => {
    document.body.innerHTML =
      `<pre style="padding:24px;color:#ff5f56">failed to start: ${err.message}\n\n`
      + `Is robodog_web_server running?</pre>`;
  });
})();
