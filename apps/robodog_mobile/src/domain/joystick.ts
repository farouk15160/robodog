export interface JoystickVector {
  readonly x: number;
  readonly y: number;
}

export interface MotionCommand {
  readonly vx: number;
  readonly vy: number;
  readonly wz: number;
}

export const ZERO_COMMAND: MotionCommand = Object.freeze({ vx: 0, vy: 0, wz: 0 });

export const clampJoystick = ({ x, y }: JoystickVector): JoystickVector => {
  if (![x, y].every(Number.isFinite)) return Object.freeze({ x: 0, y: 0 });
  const magnitude = Math.hypot(x, y);
  if (magnitude <= 1) return Object.freeze({ x, y });
  return Object.freeze({ x: x / magnitude, y: y / magnitude });
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
  return Object.freeze({
    vx: -move.y * limits.linear,
    vy: move.x * limits.linear,
    wz: -turn.x * limits.angular,
  });
};
