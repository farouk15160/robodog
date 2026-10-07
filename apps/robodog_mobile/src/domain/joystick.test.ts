import assert from 'node:assert/strict';
import test from 'node:test';

import { clampJoystick, commandForJoystick, ZERO_COMMAND } from './joystick.ts';

test('clamps joystick to a circular unit range', () => {
  assert.deepEqual(clampJoystick({ x: 2, y: 0 }), { x: 1, y: 0 });
  const diagonal = clampJoystick({ x: 1, y: 1 });
  assert.ok(Math.abs(Math.hypot(diagonal.x, diagonal.y) - 1) < 1e-9);
});

test('maps joysticks only while deadman is held', () => {
  assert.deepEqual(commandForJoystick({ x: 0.4, y: -0.5 }, { x: -0.25, y: 0 }, false), ZERO_COMMAND);
  assert.deepEqual(commandForJoystick({ x: 0.4, y: -0.5 }, { x: -0.25, y: 0 }, true), {
    vx: 0.5,
    vy: 0.4,
    wz: 0.5,
  });
});

test('release returns a new immutable zero command', () => {
  const first = commandForJoystick({ x: 1, y: 1 }, { x: 1, y: 1 }, false);
  const second = commandForJoystick({ x: 1, y: 1 }, { x: 1, y: 1 }, false);
  assert.deepEqual(first, ZERO_COMMAND);
  assert.notEqual(first, second);
});
