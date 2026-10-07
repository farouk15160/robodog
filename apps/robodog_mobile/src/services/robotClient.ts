import { normalizeManualOrigin, parseRobotDescriptor, parseRobotState } from '../domain/robot.ts';
import type { RobotDescriptor, RobotState } from '../domain/robot.ts';
import { descriptorUrlForService, parseMdnsService } from './discovery.ts';
import type { MdnsServiceRecord } from './discovery.ts';

type Fetcher = typeof fetch;

const fetchJson = async (url: string, fetcher: Fetcher): Promise<unknown> => {
  const response = await fetcher(url, { headers: { Accept: 'application/json' } });
  if (!response.ok) throw new Error(`Robot returned HTTP ${response.status}`);
  return response.json();
};

export const connectDiscoveredRobot = async (
  rawService: unknown,
  fetcher: Fetcher = fetch,
): Promise<RobotDescriptor> => {
  const service = parseMdnsService(rawService);
  if (!service) throw new Error('Invalid RoboDog discovery record');
  const url = descriptorUrlForService(service);
  return parseRobotDescriptor(await fetchJson(url, fetcher), new URL(url).origin);
};

export const connectManualRobot = async (input: string, fetcher: Fetcher = fetch): Promise<RobotDescriptor> => {
  const origin = normalizeManualOrigin(input);
  const payload = await fetchJson(`${origin}/.well-known/robodog`, fetcher);
  return parseRobotDescriptor(payload, origin);
};

export const fetchRobotState = async (robot: RobotDescriptor, fetcher: Fetcher = fetch): Promise<RobotState> => {
  const path = robot.endpoints.state ?? '/api/state';
  return parseRobotState(await fetchJson(new URL(path, robot.origin).toString(), fetcher));
};

export type { MdnsServiceRecord };
