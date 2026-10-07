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
    seq: 0, lastFrame: 0, disconnectHooks: [], ackHooks: [], pageHooks: [], retry: 500,
    page: "dashboard",
  };

  const $ = (id) => document.getElementById(id);

  /* React Native uses this page as a same-origin command transport.  The
   * bridge exists only inside our WebView and only on an explicitly opted-in
   * URL; an ordinary browser never gets this message listener. */
  const nativeBridge = createNativeBridge();

  function createNativeBridge() {
    let optedIn = false;
    try {
      optedIn = new URLSearchParams(location.search).get("native_bridge") === "1";
    } catch { /* an invalid URL disables the bridge */ }
    const channel = window.ReactNativeWebView;
    if (!optedIn || !channel || typeof channel.postMessage !== "function") return null;

    let pending = {};
    let lastConnection = null;
    const post = message => {
      try {
        channel.postMessage(JSON.stringify({ type: "robodog.bridge", ...message }));
      } catch { /* a closing native view cannot receive feedback */ }
    };
    const error = (message, id) => post({ event: "error", ...(id ? { id } : {}), message });

    const receive = event => {
      const decoded = decodeNativeCommand(event && event.data);
      if (!decoded.ok) {
        error(decoded.message, decoded.id);
        return;
      }
      const { id, payload } = decoded;
      if (!send(payload)) {
        error("robot command socket is not connected", id);
        return;
      }
      const actionPending = pending[payload.action] || [];
      pending = { ...pending, [payload.action]: [...actionPending, id] };
    };
    // react-native-webview dispatches on document on Android and window on iOS.
    document.addEventListener("message", receive);
    window.addEventListener("message", receive);

    return Object.freeze({
      ready(cfg, info) {
        post({ event: "ready", version: 1, limits: cfg.limits, info });
      },
      connection(connected, message) {
        const key = `${connected}:${message}`;
        if (key === lastConnection) return;
        lastConnection = key;
        post({ event: "connection", connected, message });
      },
      ack(message) {
        const actionPending = pending[message.action] || [];
        const id = actionPending[0];
        pending = { ...pending, [message.action]: actionPending.slice(1) };
        post({ event: "ack", ...(id ? { id } : {}),
          action: message.action, ok: message.ok === true,
          message: String(message.message || ""),
          ...(message.data && typeof message.data === "object" ? { data: message.data } : {}) });
      },
      state(message) {
        post({ event: "state", seq: message.seq, t: message.t,
          state: message.data, events: Array.isArray(message.events) ? message.events : [] });
      },
    });
  }

  function decodeNativeCommand(raw) {
    if (typeof raw !== "string") return { ok: false, message: "message data must be JSON text" };
    let envelope;
    try { envelope = JSON.parse(raw); } catch { return { ok: false, message: "malformed JSON" }; }
    if (!plainObject(envelope)) return { ok: false, message: "message must be a JSON object" };
    const id = typeof envelope.id === "string" ? envelope.id : undefined;
    if (!exactKeys(envelope, ["id", "payload", "type"]) || envelope.type !== "robodog.command") {
      return { ok: false, id, message: "invalid command envelope" };
    }
    if (!id || !/^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$/.test(id)) {
      return { ok: false, message: "invalid command id" };
    }
    const payload = envelope.payload;
    if (!plainObject(payload) || typeof payload.action !== "string") {
      return { ok: false, id, message: "invalid command payload" };
    }

    if (payload.action === "cmd_vel") {
      const values = [payload.vx, payload.vy, payload.wz];
      if (!exactKeys(payload, ["action", "vx", "vy", "wz"])
          || values.some(value => typeof value !== "number" || !Number.isFinite(value))) {
        return { ok: false, id, message: "cmd_vel requires finite vx, vy and wz" };
      }
      return { ok: true, id, payload: {
        action: "cmd_vel", vx: payload.vx, vy: payload.vy, wz: payload.wz,
      } };
    }
    if (payload.action === "stand" && exactKeys(payload, ["action"])) {
      return { ok: true, id, payload: { action: "gait", gait: "stand", enable: true } };
    }
    if (payload.action === "gait") {
      const known = ["stand", "walk", "trot", "pace", "bound"];
      if (!exactKeys(payload, ["action", "enable", "gait"])
          || !known.includes(payload.gait) || typeof payload.enable !== "boolean") {
        return { ok: false, id, message: "invalid gait command" };
      }
      return { ok: true, id, payload: {
        action: "gait", gait: payload.gait, enable: payload.enable,
      } };
    }
    if (payload.action === "greeting" && exactKeys(payload, ["action"])) {
      return { ok: true, id, payload: { action: "greeting" } };
    }
    if (payload.action === "estop") {
      const withReason = exactKeys(payload, ["action", "reason"])
        && typeof payload.reason === "string"
        && /^[\x20-\x7E]{1,160}$/.test(payload.reason);
      if (!exactKeys(payload, ["action"]) && !withReason) {
        return { ok: false, id, message: "invalid emergency-stop reason" };
      }
      return { ok: true, id, payload: {
        action: "estop", reason: withReason ? payload.reason : "React Native controller",
      } };
    }
    return { ok: false, id, message: "command action is not allowed" };
  }

  function plainObject(value) {
    return value !== null && typeof value === "object" && !Array.isArray(value);
  }

  function exactKeys(value, expected) {
    const actual = Object.keys(value).sort();
    return actual.length === expected.length
      && expected.every((key, index) => actual[index] === key);
  }

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
        if (nativeBridge) {
          nativeBridge.state(msg);
          return;
        }
        render(msg.data, msg.events || []);
      } else if (msg.type === "info") {
        state.info = msg.data;
      } else if (msg.type === "ack") {
        if (nativeBridge) {
          nativeBridge.ack(msg);
          return;
        }
        $("command-status").textContent = `${msg.action}: ${msg.message}`;
        $("command-status").style.color = msg.ok ? "var(--accent)" : "var(--alarm)";
        if (!msg.ok) toast(`${msg.action}: ${msg.message}`);
        state.ackHooks.forEach(fn => { try { fn(msg); } catch (e) { /* isolate panels */ } });
      }
    };
  }

  function send(payload) {
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
      if (!nativeBridge) toast("not connected");
      return false;
    }
    state.ws.send(JSON.stringify({ type: "cmd", ...payload }));
    return true;
  }

  function setLink(up, text) {
    if (nativeBridge) {
      nativeBridge.connection(up, text);
      return;
    }
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
    if (nativeBridge) {
      nativeBridge.ready(cfg, info);
      connect();
      setInterval(watchdog, 500);
      return;
    }
    $("brand-sub").textContent =
      `${info.joints.length}-DOF · ${info.actuator} · ${info.mass_kg.toFixed(1)} kg`;
    $("foot-proto").textContent = `protocol v${cfg.protocol}`;

    const ctx = {
      cfg, info, send,
      onDisconnect: (fn) => state.disconnectHooks.push(fn),
      onAck: (fn) => state.ackHooks.push(fn),
      onPageChange: (fn) => state.pageHooks.push(fn),
      isPageActive: (page) => state.page === page,
    };
    const grid = $("grid");
    const tpl = $("tpl-panel");

    for (const spec of cfg.panels) {
      const impl = Panels[spec.id];
      const node = tpl.content.firstElementChild.cloneNode(true);
      node.classList.add("w" + (spec.width || 1));
      node.dataset.page = spec.page || "dashboard";
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

    const pageFromLocation = () => location.pathname === "/remote" ? "remote" : "dashboard";
    const selectPage = (page) => {
      const previous = state.page;
      state.page = page;
      document.body.dataset.page = page;
      document.querySelectorAll(".panel[data-page]").forEach(panel => {
        panel.hidden = panel.dataset.page !== page;
      });
      document.querySelectorAll(".page-tab").forEach(button => {
        const selected = button.dataset.page === page;
        button.classList.toggle("on", selected);
        button.setAttribute("aria-pressed", String(selected));
      });
      if (previous !== page) state.pageHooks.forEach(fn => {
        try { fn(previous, page); } catch (e) { /* isolate panels */ }
      });
    };
    document.querySelectorAll(".page-tab").forEach(button => {
      button.onclick = () => {
        const page = button.dataset.page;
        history.pushState(null, "", page === "remote" ? "/remote" : "/");
        selectPage(page);
      };
    });
    window.addEventListener("popstate", () => selectPage(pageFromLocation()));
    selectPage(pageFromLocation());

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
      if (nativeBridge) return;
      document.querySelectorAll(".panel").forEach(p => p.style.opacity = 0.5);
    } else if (age <= 2.0) {
      if (nativeBridge) {
        if (state.ws && state.ws.readyState === WebSocket.OPEN) setLink(true, "connected");
        return;
      }
      document.querySelectorAll(".panel").forEach(p => p.style.opacity = 1);
      if (state.ws && state.ws.readyState === WebSocket.OPEN) setLink(true, "connected");
    }
  }

  boot().catch(err => {
    if (nativeBridge) {
      nativeBridge.connection(false, `failed to start: ${err.message}`);
      return;
    }
    document.body.innerHTML =
      `<pre style="padding:24px;color:#ff5f56">failed to start: ${err.message}\n\n`
      + `Is robodog_web_server running?</pre>`;
  });
})();
