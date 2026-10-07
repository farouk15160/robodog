export type ControllerPresentation =
  | 'portrait-controller'
  | 'landscape-split'
  | 'tablet-dashboard';

export type ControllerOrientation = 'portrait' | 'landscape';
export type ControlPlacement = 'bottom' | 'side';

export interface LandscapeGamepadMeasurements {
  /** Width reserved for each thumb-control pane. */
  readonly sidePaneWidth: number;
  /** Width reserved for camera, telemetry and the compact action rows. */
  readonly centerPaneWidth: number;
  /** Horizontal gap between a side pane and the centre pane. */
  readonly gap: number;
  readonly telemetryHeight: number;
  readonly actionButtonHeight: number;
  readonly actionRowsHeight: number;
  /** Top identity bar and bottom breathing room outside the gamepad body. */
  readonly verticalChromeHeight: number;
}

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
  /** Present only for phone-sized landscape viewports. */
  readonly landscapeGamepad: LandscapeGamepadMeasurements | null;
}

const DEFAULT_WIDTH = 390;
const DEFAULT_HEIGHT = 844;
const TABLET_SHORTEST_SIDE = 600;
const MIN_VIEWPORT_EDGE = 240;
const MIN_GAMEPAD_WIDTH = 340;
const LANDSCAPE_JOYSTICK_MIN = 96;
const GAMEPAD_CENTER_MIN_WIDTH = 140;
const GAMEPAD_SIDE_MAX_WIDTH = 168;
const GAMEPAD_GAP = 6;
const GAMEPAD_TELEMETRY_HEIGHT = 38;
const GAMEPAD_ACTION_BUTTON_HEIGHT = 34;
const GAMEPAD_ACTION_ROWS_HEIGHT = 0;
const GAMEPAD_VERTICAL_CHROME_HEIGHT = 42;
const GAMEPAD_SECTION_GAPS_HEIGHT = 8;

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
  const landscapeHeightLimit = clamp(
    Math.floor((height - 88) / 2.25),
    LANDSCAPE_JOYSTICK_MIN,
    142,
  );
  const joystickDiameter = tablet
    ? 142
    : orientation === 'portrait'
      ? Math.min(clamp(Math.floor(availableJoystickWidth / 2), 104, 142), portraitHeightLimit)
      : Math.min(
        clamp(Math.floor(availableJoystickWidth / 2), LANDSCAPE_JOYSTICK_MIN, 142),
        landscapeHeightLimit,
      );
  const landscapeGamepadEnabled = orientation === 'landscape'
    && !tablet
    && width >= MIN_GAMEPAD_WIDTH;
  const availableGamepadHeight = height - GAMEPAD_VERTICAL_CHROME_HEIGHT;
  const gamepadCameraHeight = clamp(
    availableGamepadHeight
      - GAMEPAD_TELEMETRY_HEIGHT
      - GAMEPAD_SECTION_GAPS_HEIGHT,
    96,
    200,
  );
  const cameraHeight = tablet
    ? clamp(Math.round(width * 0.38), 200, 320)
    : orientation === 'portrait'
      ? clamp(Math.round(height * 0.18), 112, 156)
      : landscapeGamepadEnabled
        ? gamepadCameraHeight
        : clamp(Math.round(height * 0.46), 112, 200);
  const presentation: ControllerPresentation = tablet
    ? 'tablet-dashboard'
    : orientation === 'portrait'
      ? 'portrait-controller'
      : 'landscape-split';
  const landscapeGamepad = landscapeGamepadEnabled
    ? (() => {
        const availableWidth = width - pagePadding * 2;
        const minimumSidePaneWidth = joystickDiameter;
        const preferredMinimumSidePaneWidth = joystickDiameter + 8;
        const maximumSidePaneWidth = Math.floor(
          (availableWidth - GAMEPAD_CENTER_MIN_WIDTH - GAMEPAD_GAP * 2) / 2,
        );
        const preferredSidePaneWidth = clamp(
          Math.floor(availableWidth * 0.22),
          preferredMinimumSidePaneWidth,
          GAMEPAD_SIDE_MAX_WIDTH,
        );
        const sidePaneWidth = maximumSidePaneWidth < preferredMinimumSidePaneWidth
          ? Math.max(minimumSidePaneWidth, maximumSidePaneWidth)
          : Math.min(preferredSidePaneWidth, maximumSidePaneWidth);
        const centerPaneWidth = availableWidth - sidePaneWidth * 2 - GAMEPAD_GAP * 2;
        return Object.freeze({
          sidePaneWidth,
          centerPaneWidth,
          gap: GAMEPAD_GAP,
          telemetryHeight: GAMEPAD_TELEMETRY_HEIGHT,
          actionButtonHeight: GAMEPAD_ACTION_BUTTON_HEIGHT,
          actionRowsHeight: GAMEPAD_ACTION_ROWS_HEIGHT,
          verticalChromeHeight: GAMEPAD_VERTICAL_CHROME_HEIGHT,
        });
      })()
    : null;

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
    landscapeGamepad,
  });
};
