import assert from 'node:assert/strict';
import test from 'node:test';

import { descriptorUrlForService, mergeDiscoveredRobots, parseMdnsService } from './discovery.ts';

test('accepts only RoboDog mDNS records with valid port and identity', () => {
  const service = parseMdnsService({
    name: 'RoboDog Workshop',
    type: '_robodog._tcp.',
    host: 'robodog.local.',
    port: 8080,
    txt: { id: 'dog-01', api: '1' },
  });
  assert.equal(service?.host, 'robodog.local');
  assert.equal(descriptorUrlForService(service!), 'http://robodog.local:8080/.well-known/robodog');
  assert.equal(parseMdnsService({ name: 'Printer', type: '_ipp._tcp.', host: 'printer.local', port: 80 }), null);
});

test('uses advertised descriptor path and accepts private IPv4 hosts', () => {
  const service = parseMdnsService({
    name: 'RoboDog Lab', type: '_robodog._tcp', host: '192.168.20.14', port: 8080,
    txt: { id: 'dog-02', api: '1', path: '/.well-known/robodog' },
  });
  assert.ok(service);
  assert.equal(descriptorUrlForService(service), 'http://192.168.20.14:8080/.well-known/robodog');
});

test('rejects malformed or origin-escaping TXT records', () => {
  const base = { name: 'RoboDog', type: '_robodog._tcp.', host: 'robot.local', port: 8080 };
  assert.equal(parseMdnsService({ ...base, txt: { id: 42, api: '1' } }), null);
  assert.equal(parseMdnsService({ ...base, txt: { id: 'dog', api: 1 } }), null);
  assert.equal(parseMdnsService({ ...base, txt: { id: 'dog', path: 7 } }), null);
  assert.equal(parseMdnsService({ ...base, txt: { id: 'dog', path: '//evil.example/info' } }), null);
  assert.equal(parseMdnsService({ ...base, txt: [] }), null);
});

test('deduplicates robots immutably by stable id', () => {
  const first = { id: 'a', name: 'Old', model: 'RoboDog', apiVersion: 1, origin: 'http://a.local', endpoints: {} };
  const updated = { ...first, name: 'Updated' };
  const second = { ...first, id: 'b', origin: 'http://b.local' };
  const original = [first];
  const merged = mergeDiscoveredRobots(original, [updated, second]);
  assert.deepEqual(merged.map((robot) => robot.name), ['Updated', 'Old']);
  assert.equal(original[0].name, 'Old');
});
