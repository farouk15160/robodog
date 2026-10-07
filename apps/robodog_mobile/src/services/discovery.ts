import type { RobotDescriptor } from '../domain/robot.ts';
import { isPrivateRobotHost } from '../domain/robot.ts';

export interface MdnsServiceRecord {
  readonly name: string;
  readonly type: string;
  readonly host: string;
  readonly port: number;
  readonly txt?: Readonly<Record<string, string>>;
}

export interface RobotDiscoveryAdapter {
  start(onService: (service: MdnsServiceRecord) => void): Promise<() => void>;
}

export const parseMdnsService = (value: unknown): MdnsServiceRecord | null => {
  if (typeof value !== 'object' || value === null) return null;
  const source = value as Record<string, unknown>;
  if (typeof source.name !== 'string' || !source.name.trim()) return null;
  if (source.type !== '_robodog._tcp.' && source.type !== '_robodog._tcp') return null;
  if (typeof source.host !== 'string') return null;
  const host = source.host.replace(/\.$/, '').toLowerCase();
  if (!isPrivateRobotHost(host) || typeof source.port !== 'number' || !Number.isInteger(source.port) || source.port < 1 || source.port > 65535) return null;
  if (typeof source.txt !== 'object' || source.txt === null || Array.isArray(source.txt)) return null;
  const rawTxt = source.txt as Record<string, unknown>;
  if (Object.values(rawTxt).some((value) => typeof value !== 'string')) return null;
  const txt = rawTxt as Record<string, string>;
  if (!txt.id || (txt.api !== undefined && !/^\d+$/.test(txt.api))) return null;
  if (txt.path !== undefined && (!/^\/(?!\/)/.test(txt.path) || txt.path.includes('\\'))) return null;
  return Object.freeze({ name: source.name.trim(), type: '_robodog._tcp.', host, port: source.port, txt: Object.freeze({ ...txt }) });
};

export const descriptorUrlForService = (service: MdnsServiceRecord): string =>
  `http://${service.host.includes(':') ? `[${service.host}]` : service.host}:${service.port}${service.txt?.path?.startsWith('/') ? service.txt.path : '/.well-known/robodog'}`;

export const mergeDiscoveredRobots = (
  current: readonly RobotDescriptor[],
  incoming: readonly RobotDescriptor[],
): readonly RobotDescriptor[] => {
  const byId = new Map(current.map((robot) => [robot.id, robot]));
  for (const robot of incoming) byId.set(robot.id, robot);
  const incomingIds = new Set(incoming.map((robot) => robot.id));
  return Object.freeze([
    ...incoming,
    ...Array.from(byId.values()).filter((robot) => !incomingIds.has(robot.id)),
  ]);
};
