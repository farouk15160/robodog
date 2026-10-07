const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

const source = fs.readFileSync(process.argv[2], "utf8");

async function load(search = "?native_bridge=1", withNative = true) {
  const nativeMessages = [];
  const listeners = { window: {}, document: {} };
  const sockets = [];

  class FakeWebSocket {
    static OPEN = 1;
    constructor(url) {
      this.url = url;
      this.readyState = 0;
      this.sent = [];
      sockets.push(this);
    }
    send(value) { this.sent.push(JSON.parse(value)); }
    close() { this.readyState = 3; }
  }

  const addListener = group => (type, fn) => {
    (listeners[group][type] ||= []).push(fn);
  };
  const window = { addEventListener: addListener("window") };
  if (withNative) {
    window.ReactNativeWebView = {
      postMessage(value) { nativeMessages.push(JSON.parse(value)); },
    };
  }
  const document = {
    body: { innerHTML: "" },
    addEventListener: addListener("document"),
    // Any bridge-only attempt to construct the normal GUI should fail the test.
    getElementById() { throw new Error("bridge-only mode touched the dashboard DOM"); },
  };
  const sandbox = {
    window, document,
    location: { search, protocol: "http:", host: "robot.local", pathname: "/remote" },
    history: { pushState() {} },
    URLSearchParams,
    WebSocket: FakeWebSocket,
    performance: { now: () => 10 },
    fetch: async path => ({
      json: async () => path === "/api/config"
        ? { protocol: 1, panels: [], limits: { max_linear_velocity: 0.5 } }
        : { model: "RoboDog", joints: [] },
    }),
    setTimeout() { return 1; },
    setInterval() { return 1; },
    console,
  };
  vm.createContext(sandbox);
  vm.runInContext(source, sandbox);
  await new Promise(resolve => setImmediate(resolve));
  await new Promise(resolve => setImmediate(resolve));
  return { nativeMessages, listeners, sockets };
}

function emit(target, type, event) {
  for (const listener of target[type] || []) listener(event);
}

(async () => {
  // Both the explicit URL opt-in and the native WebView object are mandatory.
  const noFlag = await load("", true);
  assert.equal(noFlag.listeners.document.message, undefined);
  const noNative = await load("?native_bridge=1", false);
  assert.equal(noNative.listeners.document.message, undefined);

  const bridge = await load();
  assert.equal(bridge.sockets.length, 1);
  assert.equal(bridge.sockets[0].url, "ws://robot.local/ws");
  assert.deepEqual(bridge.nativeMessages[0], {
    type: "robodog.bridge", event: "ready", version: 1,
    limits: { max_linear_velocity: 0.5 }, info: { model: "RoboDog", joints: [] },
  });

  const socket = bridge.sockets[0];
  socket.readyState = 1;
  socket.onopen();
  assert.deepEqual(bridge.nativeMessages.at(-1), {
    type: "robodog.bridge", event: "connection", connected: true,
    message: "connected",
  });

  const command = (id, payload) => emit(bridge.listeners.document, "message", {
    data: JSON.stringify({ type: "robodog.command", id, payload }),
  });
  command("move-1", { action: "cmd_vel", vx: 0.4, vy: -0.1, wz: 0.2 });
  command("stand-1", { action: "stand" });
  command("trot-1", { action: "gait", gait: "trot", enable: true });
  command("hello-1", { action: "greeting" });
  command("stop-1", { action: "estop", reason: "Mobile emergency stop" });
  assert.deepEqual(socket.sent, [
    { type: "cmd", action: "cmd_vel", vx: 0.4, vy: -0.1, wz: 0.2 },
    { type: "cmd", action: "gait", gait: "stand", enable: true },
    { type: "cmd", action: "gait", gait: "trot", enable: true },
    { type: "cmd", action: "greeting" },
    { type: "cmd", action: "estop", reason: "Mobile emergency stop" },
  ]);

  // Reject malformed envelopes, unsafe fields, non-finite velocity, unknown
  // gaits/actions, and non-string postMessage data without touching transport.
  const sentBeforeInvalid = socket.sent.length;
  const invalid = [
    "not JSON",
    JSON.stringify({ type: "wrong", id: "x", payload: { action: "greeting" } }),
    JSON.stringify({ type: "robodog.command", id: "x", payload: {
      action: "cmd_vel", vx: 1e400, vy: 0, wz: 0,
    } }),
    JSON.stringify({ type: "robodog.command", id: "x", payload: {
      action: "cmd_vel", vx: 0, vy: 0, wz: 0, admin: true,
    } }),
    JSON.stringify({ type: "robodog.command", id: "x", payload: {
      action: "gait", gait: "moonwalk", enable: true,
    } }),
    JSON.stringify({ type: "robodog.command", id: "x", payload: { action: "jog" } }),
    JSON.stringify({ type: "robodog.command", id: "x", payload: {
      action: "estop", reason: "bad\nreason",
    } }),
  ];
  for (const data of invalid) emit(bridge.listeners.window, "message", { data });
  emit(bridge.listeners.window, "message", { data: {
    type: "robodog.command", id: "x", payload: { action: "greeting" },
  } });
  assert.equal(socket.sent.length, sentBeforeInvalid);
  assert.ok(bridge.nativeMessages.slice(-8).every(message => message.event === "error"));

  socket.onmessage({ data: JSON.stringify({
    type: "ack", action: "cmd_vel", ok: true, message: "simulation",
  }) });
  assert.deepEqual(bridge.nativeMessages.at(-1), {
    type: "robodog.bridge", event: "ack", id: "move-1", action: "cmd_vel",
    ok: true, message: "simulation",
  });

  const state = { state: "MOVING", safety: { estop: false } };
  socket.onmessage({ data: JSON.stringify({
    type: "state", seq: 42, t: 12.5, data: state, events: [{ level: "info" }],
  }) });
  assert.deepEqual(bridge.nativeMessages.at(-1), {
    type: "robodog.bridge", event: "state", seq: 42, t: 12.5,
    state, events: [{ level: "info" }],
  });

  socket.onclose();
  assert.equal(bridge.nativeMessages.at(-1).event, "connection");
  assert.equal(bridge.nativeMessages.at(-1).connected, false);
})().catch(error => {
  console.error(error);
  process.exitCode = 1;
});
