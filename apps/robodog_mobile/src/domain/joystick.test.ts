import assert from 'node:assert/strict';
import test from 'node:test';

import {
  clampJoystick,
  commandForJoystick,
  commandForHandheldInput,
  createHandheldInput,
  isHandheldInputActive,
  joystickGeometry,
  joystickTranslation,
  releaseHandheldInput,
  setHandheldStickActive,
  updateHandheldStick,
  ZERO_COMMAND,
} from './joystick.ts';

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

test('responsive joystick geometry keeps the knob inside every supported track', () => {
  assert.deepEqual(joystickGeometry(96), {
    diameter: 96,
    knobDiameter: 36,
    travel: 30,
  });
  assert.deepEqual(joystickGeometry(142), {
    diameter: 142,
    knobDiameter: 52,
    travel: 45,
  });
  assert.deepEqual(joystickTranslation({ x: 1, y: 0 }, 96), { x: 30, y: 0 });
  assert.deepEqual(joystickTranslation({ x: 0, y: -1 }, 96), { x: 0, y: -30 });
});

test('tracks two independent sticks without mutating the previous input state', () => {
  const empty = createHandheldInput();
  const moving = updateHandheldStick(empty, 'move', { x: 0.2, y: -0.8 });
  const moveHeld = setHandheldStickActive(moving, 'move', true);
  const bothHeld = setHandheldStickActive(moveHeld, 'turn', true);

  assert.deepEqual(empty, {
    move: { x: 0, y: 0 }, turn: { x: 0, y: 0 }, activeSticks: [],
  });
  assert.deepEqual(bothHeld.activeSticks, ['move', 'turn']);
  assert.equal(isHandheldInputActive(bothHeld), true);
  assert.notEqual(moving, empty);
  assert.equal(Object.isFrozen(bothHeld), true);
  assert.equal(Object.isFrozen(bothHeld.activeSticks), true);
});

test('external release clears active touches and stale vectors', () => {
  const moving = setHandheldStickActive(
    updateHandheldStick(createHandheldInput(), 'move', { x: 0.4, y: -0.7 }),
    'move',
    true,
  );
  const released = releaseHandheldInput(moving);

  assert.deepEqual(released, {
    move: { x: 0, y: 0 }, turn: { x: 0, y: 0 }, activeSticks: [],
  });
  assert.equal(isHandheldInputActive(released), false);
  assert.notEqual(released, moving);
});

test('control availability gates an otherwise active handheld command', () => {
  const active = setHandheldStickActive(
    updateHandheldStick(createHandheldInput(), 'move', { x: 0, y: -1 }),
    'move',
    true,
  );

  assert.deepEqual(commandForHandheldInput(active, false, { linear: 0.5, angular: 1.2 }), ZERO_COMMAND);
  assert.deepEqual(commandForHandheldInput(active, true, { linear: 0.5, angular: 1.2 }), {
    vx: 0.5, vy: 0, wz: 0,
  });
});
