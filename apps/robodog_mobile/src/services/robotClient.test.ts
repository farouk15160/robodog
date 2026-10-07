import assert from 'node:assert/strict';
import test from 'node:test';

import { connectDiscoveredRobot, connectManualRobot, fetchRobotState } from './robotClient.ts';

const descriptor = {
  schema: 'robodog.discovery.v1',
  device: { id: '7d2c17bf-4f25-413a-a65f-b9ac7093f2aa', name: 'Test Dog', model: 'robodog' },
  api: { protocol: 1 }, telemetry: { state: '/api/state' }, camera: { mjpeg: '/stream/color.mjpg' },
  ui: { remote_control: '/remote' },
};

const response = (payload: unknown, status = 200) => new Response(JSON.stringify(payload), {
  status,
  headers: { 'content-type': 'application/json' },
});

test('connects to the exact descriptor path advertised over mDNS', async () => {
  const calls: string[] = [];
  const fetcher: typeof fetch = async (input) => {
    calls.push(String(input));
    return response(descriptor);
  };
  const robot = await connectDiscoveredRobot({
    name: 'Test Dog', type: '_robodog._tcp.', host: '192.168.1.22', port: 8080,
    txt: { id: descriptor.device.id, api: '1', path: '/.well-known/robodog' },
  }, fetcher);
  assert.equal(robot.id, descriptor.device.id);
  assert.deepEqual(calls, ['http://192.168.1.22:8080/.well-known/robodog']);
});

test('manual connection reads the well-known descriptor', async () => {
  let requested = '';
  const robot = await connectManualRobot('robodog.local:8080', async (input) => {
    requested = String(input);
    return response(descriptor);
  });
  assert.equal(requested, 'http://robodog.local:8080/.well-known/robodog');
  assert.equal(robot.origin, 'http://robodog.local:8080');
});

test('fetches and validates telemetry from the selected robot only', async () => {
  const robot = await connectManualRobot('10.0.0.4:8080', async () => response(descriptor));
  let requested = '';
  const state = await fetchRobotState(robot, async (input) => {
    requested = String(input);
    return response({ state: 'ACTIVE', simulation: { backend: 'mujoco' }, safety: { estop: false } });
  });
  assert.equal(requested, 'http://10.0.0.4:8080/api/state');
  assert.equal(state.backend, 'mujoco');
});

test('fails closed on invalid records and HTTP errors', async () => {
  await assert.rejects(() => connectDiscoveredRobot({ type: '_ipp._tcp.' }, fetch));
  await assert.rejects(() => connectManualRobot('robot.local:8080', async () => response({}, 503)), /HTTP 503/);
});
