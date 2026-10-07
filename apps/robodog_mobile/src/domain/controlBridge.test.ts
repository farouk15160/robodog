import assert from 'node:assert/strict';
import test from 'node:test';

import {
  buildEstopEnvelope,
  buildGaitEnvelope,
  buildGreetingEnvelope,
  buildVelocityEnvelope,
  parseBridgeFeedback,
  serializeControlEnvelope,
} from './controlBridge.ts';

test('velocity envelope clamps motion to the supplied robot limits', () => {
  const envelope = buildVelocityEnvelope(
    'motion-17',
    { vx: 1.8, vy: -1.4, wz: 4 },
    { linear: 1, angular: 2 },
  );

  assert.deepEqual(envelope, {
    type: 'robodog.command',
    id: 'motion-17',
    payload: { action: 'cmd_vel', vx: 1, vy: -1, wz: 2 },
  });
  assert.ok(Object.isFrozen(envelope));
  assert.ok(Object.isFrozen(envelope.payload));
});

test('velocity envelope preserves an explicit zero stop command', () => {
  assert.deepEqual(
    buildVelocityEnvelope('stop-1', { vx: 0, vy: -0, wz: 0 }, { linear: 1, angular: 2 }),
    {
      type: 'robodog.command',
      id: 'stop-1',
      payload: { action: 'cmd_vel', vx: 0, vy: 0, wz: 0 },
    },
  );
});

test('velocity envelope rejects non-finite motion and unsafe limits', () => {
  assert.throws(
    () => buildVelocityEnvelope('motion-1', { vx: Number.NaN, vy: 0, wz: 0 }, { linear: 1, angular: 2 }),
    /finite/,
  );
  assert.throws(
    () => buildVelocityEnvelope('motion-1', { vx: 0, vy: 0, wz: 0 }, { linear: -1, angular: 2 }),
    /non-negative/,
  );
  assert.throws(
    () => buildVelocityEnvelope('motion-1', { vx: 0, vy: 0, wz: 0 }, { linear: 1, angular: Infinity }),
    /finite/,
  );
});

test('command ids are short transport-safe correlation tokens', () => {
  assert.throws(
    () => buildVelocityEnvelope('bad id', { vx: 0, vy: 0, wz: 0 }, { linear: 1, angular: 2 }),
    /id/,
  );
  assert.throws(
    () => buildVelocityEnvelope('x'.repeat(65), { vx: 0, vy: 0, wz: 0 }, { linear: 1, angular: 2 }),
    /id/,
  );
});

test('action builders expose only supported gait, greeting and E-stop payloads', () => {
  const trot = buildGaitEnvelope('gait-1', 'trot');
  const stand = buildGaitEnvelope('gait-2', 'stand');
  const greeting = buildGreetingEnvelope('hello-1');
  const estop = buildEstopEnvelope('stop-2', 'mobile operator');
  assert.deepEqual(trot, {
    type: 'robodog.command', id: 'gait-1',
    payload: { action: 'gait', gait: 'trot', enable: true },
  });
  assert.deepEqual(stand, {
    type: 'robodog.command', id: 'gait-2',
    payload: { action: 'gait', gait: 'stand', enable: true },
  });
  assert.deepEqual(greeting, {
    type: 'robodog.command', id: 'hello-1', payload: { action: 'greeting' },
  });
  assert.deepEqual(estop, {
    type: 'robodog.command', id: 'stop-2',
    payload: { action: 'estop', reason: 'mobile operator' },
  });
  [trot, stand, greeting, estop].forEach((envelope) => {
    assert.ok(Object.isFrozen(envelope));
    assert.ok(Object.isFrozen(envelope.payload));
  });
});

test('action builders reject unsupported values and unsafe E-stop reasons', () => {
  assert.throws(() => buildGaitEnvelope('gait-3', 'pace' as 'trot'), /gait/);
  assert.throws(() => buildEstopEnvelope('stop-3', ''), /reason/);
  assert.throws(() => buildEstopEnvelope('stop-3', 'x'.repeat(161)), /reason/);
  assert.throws(() => buildEstopEnvelope('stop-3', 'line\nbreak'), /reason/);
  assert.throws(() => buildEstopEnvelope('stop-3', 'robot stop 🛑'), /reason/);
});

test('serialization returns JSON data for postMessage without executable script', () => {
  const envelope = buildEstopEnvelope('stop-json', "operator's </script><script>alert(1)</script>");
  const message = serializeControlEnvelope(envelope);

  assert.deepEqual(JSON.parse(message), envelope);
  assert.ok(!message.includes('</script>'));
  assert.ok(!message.includes('<script>'));
});

test('feedback parser accepts and freezes ready and connection events', () => {
  const ready = parseBridgeFeedback(JSON.stringify({
    type: 'robodog.bridge', event: 'ready', version: 1,
    limits: { max_linear_velocity: 1 }, info: { backend: 'mujoco' },
  }));
  const connection = parseBridgeFeedback({
    type: 'robodog.bridge', event: 'connection', connected: true, message: 'connected',
  });

  assert.deepEqual(ready, {
    type: 'robodog.bridge', event: 'ready', version: 1,
    limits: { max_linear_velocity: 1 }, info: { backend: 'mujoco' },
  });
  assert.ok(Object.isFrozen(ready));
  assert.ok(ready.event === 'ready' && Object.isFrozen(ready.limits));
  assert.deepEqual(connection, {
    type: 'robodog.bridge', event: 'connection', connected: true, message: 'connected',
  });
});

test('feedback parser validates acknowledgement and bridge errors', () => {
  assert.deepEqual(parseBridgeFeedback({
    type: 'robodog.bridge', event: 'ack', id: 'motion-4', action: 'cmd_vel',
    ok: true, message: 'accepted', data: { accepted: [0.5, 0, 0] }, ignored: 'drop me',
  }), {
    type: 'robodog.bridge', event: 'ack', id: 'motion-4', action: 'cmd_vel',
    ok: true, message: 'accepted', data: { accepted: [0.5, 0, 0] },
  });
  assert.deepEqual(parseBridgeFeedback({
    type: 'robodog.bridge', event: 'error', id: 'motion-4', message: 'socket closed',
  }), {
    type: 'robodog.bridge', event: 'error', id: 'motion-4', message: 'socket closed',
  });
});

test('feedback parser validates sequenced state without retaining caller objects', () => {
  const source = {
    type: 'robodog.bridge', event: 'state', seq: 42, t: 8.5,
    state: { safety: { estop: false } }, events: [{ kind: 'contact' }],
  };
  const parsed = parseBridgeFeedback(source);
  source.state.safety.estop = true;

  assert.deepEqual(parsed, {
    type: 'robodog.bridge', event: 'state', seq: 42, t: 8.5,
    state: { safety: { estop: false } }, events: [{ kind: 'contact' }],
  });
  assert.ok(parsed.event === 'state' && Object.isFrozen(parsed.state.safety));
  assert.ok(parsed.event === 'state' && Object.isFrozen(parsed.events));
});

test('feedback parser rejects malformed, non-finite and unknown messages', () => {
  const invalidMessages: readonly unknown[] = [
    'not JSON',
    '[]',
    new Date(),
    { type: 'other', event: 'connection', connected: true, message: 'connected' },
    { type: 'robodog.bridge', event: 'connection', connected: 'yes', message: 'connected' },
    { type: 'robodog.bridge', event: 'ack', ok: true, message: 4 },
    { type: 'robodog.bridge', event: 'state', seq: -1, state: {}, events: [] },
    { type: 'robodog.bridge', event: 'state', seq: 1, t: Number.NaN, state: {}, events: [] },
    '{"type":"robodog.bridge","event":"ack","ok":true,"message":"ok","data":{"__proto__":{}}}',
    { type: 'robodog.bridge', event: 'unknown' },
  ];
  invalidMessages.forEach((message) => assert.throws(() => parseBridgeFeedback(message)));
});
