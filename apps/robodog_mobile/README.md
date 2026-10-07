# RoboDog Control mobile app

Expo/React Native operator app for discovering a RoboDog on the local network,
viewing telemetry and camera output, and opening the robot's working same-origin
web controller.

## Run a development build

The app contains a local Swift/Kotlin DNS-SD module, so Expo Go cannot perform
automatic discovery. Build the native development client once, then use Metro
for normal TypeScript changes:

```bash
npm ci
npm run android       # Android Studio/device required
# npm run ios         # macOS and Xcode required
npm start
```

The robot and phone must be on the same trusted LAN. The robot advertises
`_robodog._tcp.local` with `id`, `api`, and `path` TXT records. Manual connection
accepts a hostname or IP such as `robodog.local:8080`.

## Safety and control boundary

- The native cockpit is read-only until the robot provides a paired,
  authenticated native command endpoint. Its native joysticks and action buttons
  are deliberately locked.
- **Open working web remote** loads the discovered robot's `/remote` page inside
  a navigation-pinned WebView. This retains the server's existing same-origin
  WebSocket controls, deadman timeout, E-stop, stand, walk and greeting behavior.
- The WebView refuses top-level navigation to any other origin. Network payloads,
  advertised routes, hosts, ports and telemetry values are validated before use.
- Current robots use trusted-LAN HTTP, so the Android config plugin explicitly
  enables cleartext traffic. Remove that exception when robot TLS and certificate
  pinning ship.

The current camera surface consumes the robot's MJPEG stream. `control.tsx`
contains the media surface boundary; the planned 60 fps implementation replaces
that surface with native WebRTC/H.264 hardware decode without changing discovery,
telemetry, navigation, or cockpit layout.

## Verification

```bash
npm test
npm run test:coverage
npm run typecheck
npx expo-doctor
```

Domain, protocol and HTTP transport code must remain above 80% line and branch
coverage. Native mDNS implementations still require device builds on Android and
iOS for platform integration testing.
