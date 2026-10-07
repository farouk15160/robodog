export type RobotEndpoint = 'state' | 'cameraMjpeg';

export interface RobotDescriptor {
  readonly id: string;
  readonly name: string;
  readonly model: string;
  readonly apiVersion: number;
  readonly origin: string;
  readonly remoteControlPath: string;
  readonly endpoints: Readonly<Partial<Record<RobotEndpoint, string>>>;
}

export interface RobotState {
  readonly mode: string;
  readonly gait?: string;
  readonly backend: string;
  readonly velocity: Readonly<{ vx: number; vy: number; wz: number }>;
  readonly safety: Readonly<{ estop: boolean; driveOwner: string | null }>;
  readonly power?: Readonly<{ voltageV: number; currentA: number }>;
  readonly telemetry?: Readonly<{
    peakTorqueNm: number;
    rmsTorqueNm: number;
    maxTemperatureC: number;
    hottestJoint?: string;
  }>;
}

type JsonRecord = Record<string, unknown>;

const asRecord = (value: unknown, label: string): JsonRecord => {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new Error(`${label} must be an object`);
  }
  return value as JsonRecord;
};

const asString = (value: unknown, label: string, fallback?: string): string => {
  if (value === undefined && fallback !== undefined) return fallback;
  if (typeof value !== 'string' || value.trim().length === 0 || value.length > 160) {
    throw new Error(`${label} must be a non-empty string`);
  }
  return value.trim();
};

const asFiniteNumber = (value: unknown, label: string, fallback = 0): number => {
  const candidate = value === undefined ? fallback : value;
  if (typeof candidate !== 'number' || !Number.isFinite(candidate)) {
    throw new Error(`${label} must be a finite number`);
  }
  return candidate;
};

const normalizePath = (value: unknown, fallback: string): string => {
  const path = value === undefined ? fallback : asString(value, 'endpoint');
  if (!path.startsWith('/') || path.startsWith('//') || path.includes('\\')) {
    throw new Error('Robot endpoint must be an origin-relative path');
  }
  const parsed = new URL(path, 'http://robot.invalid');
  if (parsed.origin !== 'http://robot.invalid') throw new Error('Robot endpoint escaped its origin');
  return `${parsed.pathname}${parsed.search}`;
};

export const isPrivateRobotHost = (input: string): boolean => {
  const host = input.replace(/^\[|\]$/g, '').toLowerCase();
  if (host === 'localhost' || /^[a-z0-9.-]+\.local$/.test(host)) return true;
  if (host === '::1' || /^(?:fe[89ab][0-9a-f]|f[cd][0-9a-f]{2}):/.test(host)) return true;
  const octets = host.split('.').map(Number);
  if (octets.length !== 4 || octets.some((part) => !Number.isInteger(part) || part < 0 || part > 255)) return false;
  return octets[0] === 10 || octets[0] === 127 ||
    (octets[0] === 169 && octets[1] === 254) ||
    (octets[0] === 192 && octets[1] === 168) ||
    (octets[0] === 172 && octets[1] >= 16 && octets[1] <= 31);
};

export const normalizeManualOrigin = (input: string): string => {
  const raw = input.trim();
  if (!raw || raw.length > 255) throw new Error('Enter a robot host or URL');
  const withScheme = /^https?:\/\//i.test(raw) ? raw : `http://${raw}`;
  let parsed: URL;
  try {
    parsed = new URL(withScheme);
  } catch {
    throw new Error('Enter a valid robot host and port');
  }
  if (!['http:', 'https:'].includes(parsed.protocol)) throw new Error('Only HTTP or HTTPS robots are supported');
  if (parsed.username || parsed.password) throw new Error('Credentials are not allowed in robot URLs');
  if (!isPrivateRobotHost(parsed.hostname)) throw new Error('Robot must use a private or local network address');
  if (!parsed.hostname || parsed.pathname !== '/' || parsed.search || parsed.hash) {
    throw new Error('Enter only the robot host and port');
  }
  if (parsed.port) {
    const port = Number(parsed.port);
    if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error('Port must be between 1 and 65535');
  }
  return parsed.origin;
};

export const parseRobotDescriptor = (payload: unknown, originInput: string): RobotDescriptor => {
  const source = asRecord(payload, 'robot descriptor');
  const origin = normalizeManualOrigin(originInput);
  const device = source.device === undefined ? {} : asRecord(source.device, 'device');
  const api = source.api === undefined ? {} : asRecord(source.api, 'api');
  const ui = source.ui === undefined ? {} : asRecord(source.ui, 'ui');
  const telemetrySource = source.telemetry === undefined ? {} : asRecord(source.telemetry, 'telemetry');
  const cameraSource = source.camera === undefined ? {} : asRecord(source.camera, 'camera');
  const rawEndpoints = source.endpoints === undefined ? {} : asRecord(source.endpoints, 'endpoints');
  const apiVersion = asFiniteNumber(source.protocol_version ?? source.api_version ?? api.protocol, 'protocol_version', 1);
  if (!Number.isInteger(apiVersion) || apiVersion < 1) throw new Error('Unsupported API version');

  return Object.freeze({
    id: asString(source.robot_id ?? source.id ?? device.id, 'robot_id'),
    name: asString(source.name ?? device.name, 'name', 'RoboDog'),
    model: asString(source.model ?? device.model, 'model', 'RoboDog'),
    apiVersion,
    origin,
    remoteControlPath: normalizePath(ui.remote_control, '/remote'),
    endpoints: Object.freeze({
      state: normalizePath(rawEndpoints.state ?? telemetrySource.state, '/api/state'),
      cameraMjpeg: normalizePath(rawEndpoints.camera_mjpeg ?? cameraSource.mjpeg, '/stream/color.mjpg'),
    }),
  });
};

export const buildEndpointUrl = (robot: RobotDescriptor, endpoint: RobotEndpoint): string => {
  const path = robot.endpoints[endpoint];
  if (!path) throw new Error(`Robot did not advertise ${endpoint}`);
  return new URL(path, robot.origin).toString();
};

export const buildRemoteControlUrl = (robot: RobotDescriptor): string =>
  new URL(robot.remoteControlPath, robot.origin).toString();

export const isAllowedRobotNavigation = (robotOriginInput: string, target: string): boolean => {
  if (target === 'about:blank') return true;
  try {
    return new URL(target).origin === normalizeManualOrigin(robotOriginInput);
  } catch {
    return false;
  }
};

export const parseRobotState = (payload: unknown): RobotState => {
  const source = asRecord(payload, 'robot state');
  const command = source.command === undefined ? {} : asRecord(source.command, 'command');
  const safety = source.safety === undefined ? {} : asRecord(source.safety, 'safety');
  const controller = source.controller === undefined ? {} : asRecord(source.controller, 'controller');
  const simulation = source.simulation === undefined ? {} : asRecord(source.simulation, 'simulation');
  const base = source.base === undefined ? {} : asRecord(source.base, 'base');
  const baseVelocity = Array.isArray(base.vel) ? base.vel : [];
  const baseOmega = Array.isArray(base.omega) ? base.omega : [];
  const power = source.power === undefined ? undefined : asRecord(source.power, 'power');
  const telemetry = source.telemetry === undefined ? undefined : asRecord(source.telemetry, 'telemetry');
  const joints = Array.isArray(source.joints) ? source.joints.map((joint) => asRecord(joint, 'joint')) : [];
  const jointMaximum = (key: string): number => joints.reduce((maximum, joint) => {
    const stats = joint.stats === undefined ? joint : asRecord(joint.stats, 'joint.stats');
    const value = stats[key];
    return typeof value === 'number' && Number.isFinite(value) ? Math.max(maximum, Math.abs(value)) : maximum;
  }, 0);
  const estop = safety.estop ?? source.estop ?? false;
  if (typeof estop !== 'boolean') throw new Error('estop must be boolean');
  const driveOwner = safety.drive_owner;
  if (driveOwner !== undefined && driveOwner !== null && typeof driveOwner !== 'string') {
    throw new Error('drive_owner must be a string or null');
  }

  return Object.freeze({
    mode: asString(source.mode ?? controller.mode ?? source.state, 'mode'),
    ...(typeof controller.gait === 'string' && controller.gait.trim()
      ? { gait: controller.gait.trim() }
      : {}),
    backend: asString(source.backend ?? simulation.backend, 'backend', 'unknown'),
    velocity: Object.freeze({
      vx: asFiniteNumber(command.vx ?? source.vx ?? baseVelocity[0], 'vx'),
      vy: asFiniteNumber(command.vy ?? source.vy ?? baseVelocity[1], 'vy'),
      wz: asFiniteNumber(command.wz ?? source.wz ?? baseOmega[2], 'wz'),
    }),
    safety: Object.freeze({ estop, driveOwner: driveOwner === undefined ? null : (driveOwner as string | null) }),
    ...(power
      ? { power: Object.freeze({
          voltageV: asFiniteNumber(power.voltage_v ?? power.voltage, 'voltage'),
          currentA: asFiniteNumber(
            power.current_a ?? (typeof power.watts === 'number' && typeof (power.voltage_v ?? power.voltage) === 'number' && Number(power.voltage_v ?? power.voltage) !== 0
              ? power.watts / Number(power.voltage_v ?? power.voltage) : 0),
            'current',
          ),
        }) }
      : {}),
    ...((telemetry || joints.length || safety.max_temp !== undefined)
      ? {
          telemetry: Object.freeze({
            peakTorqueNm: asFiniteNumber(telemetry?.peak_torque_nm ?? jointMaximum('joint_torque_peak_nm'), 'peak_torque_nm'),
            rmsTorqueNm: asFiniteNumber(telemetry?.rms_torque_nm ?? jointMaximum('joint_torque_rms_nm'), 'rms_torque_nm'),
            maxTemperatureC: asFiniteNumber(telemetry?.max_temperature_c ?? safety.max_temp, 'max_temperature_c'),
            ...(typeof safety.hottest === 'string' && safety.hottest.trim()
              ? { hottestJoint: safety.hottest.trim() }
              : {}),
          }),
        }
      : {}),
  });
};
