const { withAndroidManifest } = require('@expo/config-plugins');

/**
 * Current robots serve the operator UI on trusted-LAN HTTP. Keep this native
 * exception explicit and local so it can be removed when robot TLS ships.
 */
module.exports = function withRobodogNetwork(config) {
  return withAndroidManifest(config, (nextConfig) => {
    const application = nextConfig.modResults.manifest.application?.[0];
    if (!application) throw new Error('Android manifest has no application element');
    application.$ = {
      ...application.$,
      'android:usesCleartextTraffic': 'true',
    };
    return nextConfig;
  });
};
