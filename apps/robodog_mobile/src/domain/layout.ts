export type ControllerPresentation =
  | 'portrait-controller'
  | 'landscape-split'
  | 'tablet-dashboard';

export type ControllerOrientation = 'portrait' | 'landscape';
export type ControlPlacement = 'bottom' | 'side';

export interface ControllerLayout {
  readonly presentation: ControllerPresentation;
  readonly orientation: ControllerOrientation;
  readonly compact: boolean;
  readonly splitColumns: boolean;
  readonly controlPlacement: ControlPlacement;
  readonly joystickColumns: 2;
  readonly joystickDiameter: number;
  readonly joystickGap: number;
  readonly cameraHeight: number;
  readonly pagePadding: number;
}

const DEFAULT_WIDTH = 390;
const DEFAULT_HEIGHT = 844;
const TABLET_SHORTEST_SIDE = 600;
const MIN_VIEWPORT_EDGE = 240;

const usableDimension = (value: number, fallback: number): number =>
  Number.isFinite(value) && value >= MIN_VIEWPORT_EDGE ? value : fallback;

const clamp = (value: number, minimum: number, maximum: number): number =>
  Math.min(maximum, Math.max(minimum, value));

/**
 * Converts viewport dimensions into stable controller layout measurements.
 * The returned value is independent and frozen so render code cannot mutate
 * layout state shared by a later orientation update.
 */
export const resolveControllerLayout = (rawWidth: number, rawHeight: number): ControllerLayout => {
  const width = usableDimension(rawWidth, DEFAULT_WIDTH);
  const height = usableDimension(rawHeight, DEFAULT_HEIGHT);
  const orientation: ControllerOrientation = width > height ? 'landscape' : 'portrait';
  const tablet = Math.min(width, height) >= TABLET_SHORTEST_SIDE;
  const compact = !tablet;
  const splitColumns = orientation === 'landscape';
  const pagePadding = tablet ? 24 : 12;
  const joystickGap = tablet ? 16 : 8;
  const availableJoystickWidth = width - pagePadding * 2 - joystickGap;
  const portraitHeightLimit = clamp(Math.floor((height - 64) / 4.5), 104, 142);
  const landscapeHeightLimit = clamp(Math.floor((height - 100) / 2.6), 104, 142);
  const joystickDiameter = tablet
    ? 142
    : orientation === 'portrait'
      ? Math.min(clamp(Math.floor(availableJoystickWidth / 2), 104, 142), portraitHeightLimit)
      : Math.min(clamp(Math.floor(availableJoystickWidth / 2), 104, 142), landscapeHeightLimit);
  const cameraHeight = tablet
    ? clamp(Math.round(width * 0.38), 200, 320)
    : orientation === 'portrait'
      ? clamp(Math.round(height * 0.18), 112, 156)
      : clamp(Math.round(height * 0.46), 112, 200);
  const presentation: ControllerPresentation = tablet
    ? 'tablet-dashboard'
    : orientation === 'portrait'
      ? 'portrait-controller'
      : 'landscape-split';

  return Object.freeze({
    presentation,
    orientation,
    compact,
    splitColumns,
    controlPlacement: splitColumns ? 'side' : 'bottom',
    joystickColumns: 2,
    joystickDiameter,
    joystickGap,
    cameraHeight,
    pagePadding,
  });
};
