# RoboDog Control mobile app

Expo/React Native operator app for discovering a RoboDog on the local network,
viewing telemetry and camera output, and driving it from a responsive handheld
cockpit. Portrait keeps the camera and telemetry above two side-by-side
joysticks fixed in the lower control area. Landscape switches to a split view.

## Run a development build

The app contains a local Swift/Kotlin DNS-SD module, so Expo Go cannot perform
automatic discovery. Expo Go can still run the controller by using manual
hostname/IP entry. Build the native development client once for DNS-SD, then
use Metro for normal TypeScript changes:

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

- The native joysticks with touch dead-man behavior, Stand, Walk, Greeting and
  emergency stop use an opt-in hidden WebView at
  `/remote?native_bridge=1`. The robot-served page owns the WebSocket, so its
  Origin and Host still exactly match the selected robot.
- Bridge messages use a small, strict schema. The hidden WebView is pinned to
  the selected origin, and the bridge is created only when both the URL opt-in
  and React Native message channel are present. The app does not open a native
  command socket or weaken the server hostname allowlist.
- Wait for **READY**, then touch and drag either joystick. Each joystick acts as
  a dead-man control: releasing one recentres that axis, while releasing both,
  backgrounding or leaving the screen sends zero. WebSocket disconnect and
  drive-lease expiry also stop motion.
- The server's one-operator drive lease and the controller's independent
  velocity timeout remain active. **Web Remote** opens the complete visible
  `/remote` interface in a navigation-pinned WebView.
- Network payloads, advertised routes, hosts, ports and telemetry values are
  validated before use.
- Current robots use trusted-LAN HTTP, so the Android config plugin explicitly
  enables cleartext traffic. Remove that exception when robot TLS and certificate
  pinning ship.

The current camera surface consumes the robot's MJPEG stream. The simulated
camera is configured at 15 Hz and the web encoder is capped at 10 Hz and
640 pixels wide, so this is not a 60 fps path. `control.tsx` contains the media surface
boundary; the planned 60 fps implementation replaces that surface with native
WebRTC/H.264 hardware decode without changing discovery, telemetry, navigation,
or cockpit layout.

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
