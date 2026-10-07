export interface JoystickVector {
  readonly x: number;
  readonly y: number;
}

export interface MotionCommand {
  readonly vx: number;
  readonly vy: number;
  readonly wz: number;
}

export interface JoystickGeometry {
  readonly diameter: number;
  readonly knobDiameter: number;
  readonly travel: number;
}

export type HandheldStick = 'move' | 'turn';

export interface HandheldInput {
  readonly move: JoystickVector;
  readonly turn: JoystickVector;
  readonly activeSticks: readonly HandheldStick[];
}

export const ZERO_COMMAND: MotionCommand = Object.freeze({ vx: 0, vy: 0, wz: 0 });

const zeroVector = (): JoystickVector => Object.freeze({ x: 0, y: 0 });

export const createHandheldInput = (): HandheldInput => Object.freeze({
  move: zeroVector(),
  turn: zeroVector(),
  activeSticks: Object.freeze([]),
});

export const updateHandheldStick = (
  state: HandheldInput,
  stick: HandheldStick,
  vector: JoystickVector,
): HandheldInput => Object.freeze({
  ...state,
  [stick]: clampJoystick(vector),
  activeSticks: state.activeSticks,
});

export const setHandheldStickActive = (
  state: HandheldInput,
  stick: HandheldStick,
  active: boolean,
): HandheldInput => {
  const next = active
    ? [...new Set([...state.activeSticks, stick])]
    : state.activeSticks.filter((candidate) => candidate !== stick);
  return Object.freeze({ ...state, activeSticks: Object.freeze(next) });
};

export const releaseHandheldInput = (_state: HandheldInput): HandheldInput =>
  createHandheldInput();

export const isHandheldInputActive = (state: HandheldInput): boolean =>
  state.activeSticks.length > 0;

export const clampJoystick = ({ x, y }: JoystickVector): JoystickVector => {
  if (![x, y].every(Number.isFinite)) return Object.freeze({ x: 0, y: 0 });
  const magnitude = Math.hypot(x, y);
  if (magnitude <= 1) return Object.freeze({ x, y });
  return Object.freeze({ x: x / magnitude, y: y / magnitude });
};

export const joystickGeometry = (requestedDiameter: number): JoystickGeometry => {
  const finiteDiameter = Number.isFinite(requestedDiameter) ? requestedDiameter : 142;
  const diameter = Math.round(Math.min(160, Math.max(96, finiteDiameter)));
  const knobDiameter = Math.round(Math.min(52, Math.max(36, diameter * 0.366)));
  return Object.freeze({
    diameter,
    knobDiameter,
    travel: (diameter - knobDiameter) / 2,
  });
};

export const joystickTranslation = (
  value: JoystickVector,
  requestedDiameter: number,
): JoystickVector => {
  const vector = clampJoystick(value);
  const { travel } = joystickGeometry(requestedDiameter);
  return Object.freeze({ x: vector.x * travel, y: vector.y * travel });
};

export const commandForJoystick = (
  translation: JoystickVector,
  rotation: JoystickVector,
  deadmanHeld: boolean,
  limits: Readonly<{ linear: number; angular: number }> = Object.freeze({ linear: 1, angular: 2 }),
): MotionCommand => {
  if (!deadmanHeld) return Object.freeze({ ...ZERO_COMMAND });
  const move = clampJoystick(translation);
  const turn = clampJoystick(rotation);
  const scaled = (value: number, scale: number): number => value === 0 ? 0 : value * scale;
  return Object.freeze({
    vx: scaled(-move.y, limits.linear),
    vy: scaled(move.x, limits.linear),
    wz: scaled(-turn.x, limits.angular),
  });
};

export const commandForHandheldInput = (
  state: HandheldInput,
  controlAvailable: boolean,
  limits: Readonly<{ linear: number; angular: number }> = Object.freeze({ linear: 1, angular: 2 }),
): MotionCommand =>
  commandForJoystick(state.move, state.turn, controlAvailable && isHandheldInputActive(state), limits);
