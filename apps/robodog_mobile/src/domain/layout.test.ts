import assert from 'node:assert/strict';
import test from 'node:test';

import { resolveControllerLayout } from './layout.ts';

test('keeps both joysticks side-by-side and thumb reachable on a portrait phone', () => {
  const layout = resolveControllerLayout(390, 844);

  assert.equal(layout.presentation, 'portrait-controller');
  assert.equal(layout.joystickColumns, 2);
  assert.equal(layout.controlPlacement, 'bottom');
  assert.equal(layout.splitColumns, false);
  assert.equal(layout.compact, true);
  assert.ok(layout.joystickDiameter <= 142);
  assert.ok(layout.joystickDiameter >= 112);
  assert.ok(layout.cameraHeight <= 156);
});

test('fits two usable joysticks across a narrow portrait phone', () => {
  const layout = resolveControllerLayout(320, 568);
  const occupiedWidth = layout.pagePadding * 2
    + layout.joystickDiameter * layout.joystickColumns
    + layout.joystickGap;

  assert.ok(occupiedWidth <= 320);
  assert.equal(layout.joystickColumns, 2);
  assert.equal(layout.controlPlacement, 'bottom');
  assert.ok(layout.joystickDiameter <= 112);
  assert.ok(layout.cameraHeight <= 112);
});

test('shrinks portrait controls to keep the controller deck on a short phone', () => {
  const layout = resolveControllerLayout(360, 640);

  assert.equal(layout.presentation, 'portrait-controller');
  assert.ok(layout.joystickDiameter <= 128);
  assert.ok(layout.joystickDiameter >= 104);
});

test('uses a split cockpit on a landscape phone', () => {
  const layout = resolveControllerLayout(844, 390);

  assert.equal(layout.presentation, 'landscape-split');
  assert.equal(layout.orientation, 'landscape');
  assert.equal(layout.splitColumns, true);
  assert.equal(layout.controlPlacement, 'side');
  assert.equal(layout.joystickColumns, 2);
  assert.ok(layout.joystickDiameter <= 112);
  assert.ok(layout.cameraHeight <= 200);
});

test('gives tablets a roomier dashboard independent of orientation', () => {
  const portrait = resolveControllerLayout(768, 1024);
  const landscape = resolveControllerLayout(1024, 768);

  for (const layout of [portrait, landscape]) {
    assert.equal(layout.presentation, 'tablet-dashboard');
    assert.equal(layout.compact, false);
    assert.equal(layout.joystickColumns, 2);
    assert.equal(layout.joystickDiameter, 142);
    assert.equal(layout.pagePadding, 24);
  }
  assert.equal(portrait.splitColumns, false);
  assert.equal(landscape.splitColumns, true);
});

test('returns frozen independent layout values and sanitises invalid dimensions', () => {
  const first = resolveControllerLayout(Number.NaN, -1);
  const second = resolveControllerLayout(Number.NaN, -1);

  assert.equal(Object.isFrozen(first), true);
  assert.notEqual(first, second);
  assert.equal(first.presentation, 'portrait-controller');
  assert.ok(first.joystickDiameter > 0);
});
