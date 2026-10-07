import type { MotionCommand } from './joystick';

export interface ControlLimits {
  readonly linear: number;
  readonly angular: number;
}

export interface VelocityControlEnvelope {
  readonly type: 'robodog.command';
  readonly id: string;
  readonly payload: Readonly<MotionCommand & { readonly action: 'cmd_vel' }>;
}

export type MobileGait = 'trot' | 'stand';

export interface GaitControlEnvelope {
  readonly type: 'robodog.command';
  readonly id: string;
  readonly payload: Readonly<{
    readonly action: 'gait';
    readonly gait: MobileGait;
    readonly enable: true;
  }>;
}

export interface GreetingControlEnvelope {
  readonly type: 'robodog.command';
  readonly id: string;
  readonly payload: Readonly<{ readonly action: 'greeting' }>;
}

export interface EstopControlEnvelope {
  readonly type: 'robodog.command';
  readonly id: string;
  readonly payload: Readonly<{ readonly action: 'estop'; readonly reason: string }>;
}

export type ControlEnvelope =
  | VelocityControlEnvelope
  | GaitControlEnvelope
  | GreetingControlEnvelope
  | EstopControlEnvelope;

export type JsonValue =
  | null
  | boolean
  | number
  | string
  | readonly JsonValue[]
  | JsonObject;

export interface JsonObject {
  readonly [key: string]: JsonValue;
}

export type BridgeFeedback =
  | Readonly<{
      type: 'robodog.bridge'; event: 'ready'; version: 1;
      limits: JsonObject; info: JsonObject;
    }>
  | Readonly<{
      type: 'robodog.bridge'; event: 'connection'; connected: boolean; message: string;
    }>
  | Readonly<{
      type: 'robodog.bridge'; event: 'ack'; id?: string; action?: string;
      ok: boolean; message: string; data?: JsonObject;
    }>
  | Readonly<{
      type: 'robodog.bridge'; event: 'state'; seq: number; t?: number;
      state: JsonObject; events: readonly JsonValue[];
    }>
  | Readonly<{
      type: 'robodog.bridge'; event: 'error'; id?: string; message: string;
    }>;

const validateId = (id: string): string => {
  if (!/^[A-Za-z0-9][A-Za-z0-9._:-]{0,63}$/.test(id)) {
    throw new Error('command id must be a 1-64 character transport-safe token');
  }
  return id;
};

const validateLimit = (value: number, label: string): number => {
  if (!Number.isFinite(value)) throw new Error(`${label} limit must be finite`);
  if (value < 0) throw new Error(`${label} limit must be non-negative`);
  return value;
};

const clamp = (value: number, limit: number, label: string): number => {
  if (!Number.isFinite(value)) throw new Error(`${label} must be finite`);
  if (value === 0) return 0;
  return Math.max(-limit, Math.min(limit, value));
};

export const buildVelocityEnvelope = (
  id: string,
  command: MotionCommand,
  limits: ControlLimits,
): VelocityControlEnvelope => Object.freeze({
  type: 'robodog.command',
  id: validateId(id),
  payload: Object.freeze({
    action: 'cmd_vel',
    vx: clamp(command.vx, validateLimit(limits.linear, 'linear'), 'vx'),
    vy: clamp(command.vy, validateLimit(limits.linear, 'linear'), 'vy'),
    wz: clamp(command.wz, validateLimit(limits.angular, 'angular'), 'wz'),
  }),
});

export const buildGaitEnvelope = (id: string, gait: MobileGait): GaitControlEnvelope => {
  if (gait !== 'trot' && gait !== 'stand') throw new Error('gait must be trot or stand');
  return Object.freeze({
    type: 'robodog.command',
    id: validateId(id),
    payload: Object.freeze({ action: 'gait', gait, enable: true }),
  });
};

export const buildGreetingEnvelope = (id: string): GreetingControlEnvelope => Object.freeze({
  type: 'robodog.command',
  id: validateId(id),
  payload: Object.freeze({ action: 'greeting' }),
});

const validateReason = (reason: string): string => {
  if (!/^[\x20-\x7e]{1,160}$/.test(reason)) {
    throw new Error('E-stop reason must contain 1-160 printable ASCII characters');
  }
  return reason;
};

export const buildEstopEnvelope = (
  id: string,
  reason = 'mobile app operator',
): EstopControlEnvelope => Object.freeze({
  type: 'robodog.command',
  id: validateId(id),
  payload: Object.freeze({ action: 'estop', reason: validateReason(reason) }),
});

export const serializeControlEnvelope = (envelope: ControlEnvelope): string =>
  JSON.stringify(envelope)
    .replace(/</g, '\\u003c')
    .replace(/>/g, '\\u003e')
    .replace(/&/g, '\\u0026')
    .replace(/\u2028/g, '\\u2028')
    .replace(/\u2029/g, '\\u2029');

const asRecord = (value: unknown, label: string): Record<string, unknown> => {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new Error(`${label} must be an object`);
  }
  const prototype = Object.getPrototypeOf(value);
  if (prototype !== Object.prototype && prototype !== null) {
    throw new Error(`${label} must be a plain object`);
  }
  return value as Record<string, unknown>;
};

const asBridgeString = (value: unknown, label: string, maxLength = 1024): string => {
  if (typeof value !== 'string' || value.length > maxLength || /[\u0000\u007f]/.test(value)) {
    throw new Error(`${label} must be a valid string`);
  }
  return value;
};

const cloneJson = (value: unknown, depth = 0): JsonValue => {
  if (depth > 16) throw new Error('bridge data is too deeply nested');
  if (value === null || typeof value === 'boolean') return value;
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new Error('bridge numbers must be finite');
    return value;
  }
  if (typeof value === 'string') {
    if (value.length > 65536) throw new Error('bridge string is too long');
    return value;
  }
  if (Array.isArray(value)) {
    if (value.length > 2048) throw new Error('bridge array is too large');
    return Object.freeze(value.map((item) => cloneJson(item, depth + 1)));
  }
  const source = asRecord(value, 'bridge data');
  const entries = Object.entries(source);
  if (entries.length > 512) throw new Error('bridge object has too many fields');
  if (entries.some(([key]) => ['__proto__', 'prototype', 'constructor'].includes(key))) {
    throw new Error('bridge data contains an unsafe field');
  }
  return Object.freeze(Object.fromEntries(
    entries.map(([key, item]) => [key, cloneJson(item, depth + 1)]),
  ));
};

const cloneObject = (value: unknown, label: string): JsonObject => {
  const cloned = cloneJson(asRecord(value, label));
  return cloned as JsonObject;
};

const optionalId = (value: unknown): string | undefined =>
  value === undefined ? undefined : validateId(asBridgeString(value, 'feedback id', 64));

const parseFeedbackSource = (payload: unknown): Record<string, unknown> => {
  if (typeof payload !== 'string') return asRecord(payload, 'bridge feedback');
  if (payload.length > 524288) throw new Error('bridge feedback is too large');
  try {
    return asRecord(JSON.parse(payload), 'bridge feedback');
  } catch (error) {
    if (error instanceof SyntaxError) throw new Error('bridge feedback must be valid JSON');
    throw error;
  }
};

export const parseBridgeFeedback = (payload: unknown): BridgeFeedback => {
  const source = parseFeedbackSource(payload);
  if (source.type !== 'robodog.bridge') throw new Error('unexpected bridge message type');
  const event = asBridgeString(source.event, 'bridge event', 32);
  if (event === 'ready') {
    if (source.version !== 1) throw new Error('unsupported bridge version');
    return Object.freeze({
      type: 'robodog.bridge', event, version: 1,
      limits: cloneObject(source.limits, 'bridge limits'),
      info: cloneObject(source.info, 'bridge info'),
    });
  }
  if (event === 'connection') {
    if (typeof source.connected !== 'boolean') throw new Error('connected must be boolean');
    return Object.freeze({
      type: 'robodog.bridge', event, connected: source.connected,
      message: asBridgeString(source.message, 'connection message'),
    });
  }
  if (event === 'ack') return parseAck(source);
  if (event === 'state') return parseState(source);
  if (event === 'error') {
    const id = optionalId(source.id);
    return Object.freeze({
      type: 'robodog.bridge', event,
      ...(id === undefined ? {} : { id }),
      message: asBridgeString(source.message, 'bridge error'),
    });
  }
  throw new Error('unsupported bridge event');
};

const parseAck = (source: Record<string, unknown>): BridgeFeedback => {
  if (typeof source.ok !== 'boolean') throw new Error('ack ok must be boolean');
  const id = optionalId(source.id);
  const action = source.action === undefined
    ? undefined
    : asBridgeString(source.action, 'ack action', 64);
  return Object.freeze({
    type: 'robodog.bridge', event: 'ack',
    ...(id === undefined ? {} : { id }),
    ...(action === undefined ? {} : { action }),
    ok: source.ok,
    message: asBridgeString(source.message, 'ack message'),
    ...(source.data === undefined ? {} : { data: cloneObject(source.data, 'ack data') }),
  });
};

const parseState = (source: Record<string, unknown>): BridgeFeedback => {
  if (!Number.isSafeInteger(source.seq) || Number(source.seq) < 0) {
    throw new Error('state seq must be a non-negative safe integer');
  }
  if (source.t !== undefined && (typeof source.t !== 'number' || !Number.isFinite(source.t))) {
    throw new Error('state timestamp must be finite');
  }
  if (!Array.isArray(source.events)) throw new Error('state events must be an array');
  const events = cloneJson(source.events) as readonly JsonValue[];
  return Object.freeze({
    type: 'robodog.bridge', event: 'state', seq: Number(source.seq),
    ...(source.t === undefined ? {} : { t: source.t as number }),
    state: cloneObject(source.state, 'robot state'), events,
  });
};
