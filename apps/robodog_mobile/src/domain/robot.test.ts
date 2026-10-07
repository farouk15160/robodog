import assert from 'node:assert/strict';
import test from 'node:test';

import {
  buildEndpointUrl,
  buildRemoteControlUrl,
  isAllowedRobotNavigation,
  normalizeManualOrigin,
  parseRobotDescriptor,
  parseRobotState,
} from './robot.ts';

test('normalizes safe manual LAN origins', () => {
  assert.equal(normalizeManualOrigin('192.168.1.42:8080'), 'http://192.168.1.42:8080');
  assert.equal(normalizeManualOrigin('http://robodog.local:8080/'), 'http://robodog.local:8080');
  assert.equal(normalizeManualOrigin('127.0.0.1:8080'), 'http://127.0.0.1:8080');
  assert.equal(normalizeManualOrigin('169.254.10.2:8080'), 'http://169.254.10.2:8080');
  assert.equal(normalizeManualOrigin('http://[fe80::1234]:8080'), 'http://[fe80::1234]:8080');
});

test('rejects credentials, paths, fragments, unsupported schemes and invalid ports', () => {
  for (const value of [
    'https://user:pass@robot.local',
    'http://robot.local/admin',
    'file:///etc/passwd',
    'javascript:alert(1)',
    'robot.local:99999',
    'robot.local/#fragment',
    'https://example.com',
    'http://8.8.8.8:8080',
  ]) {
    assert.throws(() => normalizeManualOrigin(value));
  }
});

test('parses discovery descriptor and uses advertised remote route', () => {
  const robot = parseRobotDescriptor(
    {
      protocol_version: 1,
      robot_id: 'dog-01',
      name: 'Workshop RoboDog',
      model: 'robodog-rs06',
      ui: { remote_control: '/pilot' },
      endpoints: { state: '/api/state', camera_mjpeg: '/stream/color.mjpg' },
    },
    'http://192.168.4.12:8080',
  );

  assert.equal(robot.id, 'dog-01');
  assert.equal(buildRemoteControlUrl(robot), 'http://192.168.4.12:8080/pilot');
  assert.equal(buildEndpointUrl(robot, 'state'), 'http://192.168.4.12:8080/api/state');
});

test('parses the nested well-known discovery descriptor served by RoboDog', () => {
  const robot = parseRobotDescriptor(
    {
      schema: 'robodog.discovery.v1',
      device: { id: '7d2c17bf-4f25-413a-a65f-b9ac7093f2aa', name: 'Garage Dog', model: 'robodog' },
      api: { protocol: 1 },
      telemetry: { state: '/api/state' },
      camera: { mjpeg: '/stream/color.mjpg', max_rate_hz: 10 },
      ui: { remote_control: '/remote' },
    },
    'http://10.0.0.8:8080',
  );
  assert.equal(robot.id, '7d2c17bf-4f25-413a-a65f-b9ac7093f2aa');
  assert.equal(robot.name, 'Garage Dog');
  assert.equal(buildEndpointUrl(robot, 'cameraMjpeg'), 'http://10.0.0.8:8080/stream/color.mjpg');
});

test('falls back to /remote and blocks endpoint origin escape', () => {
  const robot = parseRobotDescriptor(
    { protocol_version: 1, robot_id: 'dog-02', name: 'RoboDog' },
    'http://robot.local:8080',
  );
  assert.equal(buildRemoteControlUrl(robot), 'http://robot.local:8080/remote');
  assert.throws(() =>
    parseRobotDescriptor(
      { protocol_version: 1, robot_id: 'dog-02', endpoints: { state: 'https://evil.example/state' } },
      'http://robot.local:8080',
    ),
  );
});

test('WebView navigation is pinned to the selected robot origin', () => {
  const origin = 'http://192.168.4.12:8080';
  assert.equal(isAllowedRobotNavigation(origin, `${origin}/remote`), true);
  assert.equal(isAllowedRobotNavigation(origin, `${origin}/assets/app.js`), true);
  assert.equal(isAllowedRobotNavigation(origin, 'about:blank'), true);
  assert.equal(isAllowedRobotNavigation(origin, 'http://192.168.4.12:8081/remote'), false);
  assert.equal(isAllowedRobotNavigation(origin, 'https://evil.example/remote'), false);
  assert.equal(isAllowedRobotNavigation(origin, 'javascript:alert(1)'), false);
});

test('validates and normalizes read-only robot state', () => {
  const state = parseRobotState({
    mode: 'TROT',
    backend: 'mujoco',
    command: { vx: 0.8, vy: 0.1, wz: -0.2 },
    safety: { estop: false, drive_owner: 'web' },
    power: { voltage_v: 24.4, current_a: 6.1 },
    telemetry: { peak_torque_nm: 11.2, rms_torque_nm: 4.1, max_temperature_c: 47.0 },
  });
  assert.equal(state.mode, 'TROT');
  assert.equal(state.velocity.vx, 0.8);
  assert.equal(state.safety.estop, false);
  assert.equal(state.power?.voltageV, 24.4);
  assert.equal(state.telemetry?.peakTorqueNm, 11.2);
});

test('rejects malformed robot state instead of trusting network data', () => {
  assert.throws(() => parseRobotState({ mode: 123 }));
  assert.throws(() => parseRobotState({ mode: 'TROT', command: { vx: Number.NaN } }));
});

test('parses the ROS web bridge state shape', () => {
  const state = parseRobotState({
    state: 'ACTIVE',
    base: { vel: [0.92, -0.04, 0] },
    controller: { mode: 'IMPEDANCE', gait: 'trot' },
    simulation: { backend: 'mujoco' },
    power: { voltage: 24, watts: 120 },
    safety: { estop: false, max_temp: 49, hottest: 'FL_kfe_joint' },
    joints: [
      { stats: { joint_torque_rms_nm: 4.2, joint_torque_peak_nm: 11.1 } },
      { stats: { joint_torque_rms_nm: 5.0, joint_torque_peak_nm: 13.8 } },
    ],
  });
  assert.equal(state.mode, 'IMPEDANCE');
  assert.equal(state.backend, 'mujoco');
  assert.equal(state.gait, 'trot');
  assert.equal(state.telemetry?.hottestJoint, 'FL_kfe_joint');
  assert.equal(state.velocity.vx, 0.92);
  assert.equal(state.velocity.wz, 0);
  assert.equal(state.power?.currentA, 5);
  assert.equal(state.telemetry?.peakTorqueNm, 13.8);
  assert.equal(state.telemetry?.rmsTorqueNm, 5);
});

test('reads yaw rate from ROS base omega', () => {
  const state = parseRobotState({ state: 'ACTIVE', base: { omega: [0, 0, -0.7] }, safety: { estop: false } });
  assert.equal(state.velocity.wz, -0.7);
});
