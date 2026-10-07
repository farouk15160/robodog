import { requireOptionalNativeModule } from 'expo-modules-core';

import type { MdnsServiceRecord, RobotDiscoveryAdapter } from './discovery';

interface Subscription { remove(): void }
interface NativeDiscoveryModule {
  startDiscovery(serviceType: string): void;
  stopDiscovery(): void;
  addListener(event: 'onService', listener: (record: unknown) => void): Subscription;
}

const nativeModule = requireOptionalNativeModule('RobotDiscovery') as NativeDiscoveryModule | null;

export const nativeDiscoveryAvailable = nativeModule !== null;

export const nativeRobotDiscovery: RobotDiscoveryAdapter = {
  async start(onService) {
    if (!nativeModule) return () => undefined;
    const subscription = nativeModule.addListener('onService', (record) => {
      onService(record as MdnsServiceRecord);
    });
    nativeModule.startDiscovery('_robodog._tcp');
    return () => {
      subscription.remove();
      nativeModule.stopDiscovery();
    };
  },
};
