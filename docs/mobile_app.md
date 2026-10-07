# RoboDog mobile app

`apps/robodog_mobile` is an Expo/React Native TypeScript client for phones and
tablets. It discovers a robot on the local network, validates its public
descriptor and telemetry, and provides a responsive handheld controller. In
portrait, camera and telemetry stay above a bottom control area containing two
side-by-side joysticks. Landscape uses a split camera/control cockpit.

## Robot setup

Install Avahi's service publisher on the robot computer, ensure its daemon is
running, build the ROS workspace, and launch normally:

```bash
sudo apt-get install avahi-daemon avahi-utils python3-pil
sudo systemctl enable --now avahi-daemon
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robodog_bringup robot.launch.py \
  backend:=mujoco world:=house device_name:="Workshop RoboDog"
```

The default `web_discovery:=true` advertises `_robodog._tcp.local`. The TXT
records contain only the API version, stable UUID, model and discovery path.
`GET /.well-known/robodog` returns the validated service descriptor, including
the telemetry, MJPEG and `/remote` paths. The UUID is created once at
`${ROS_HOME:-~/.ros}/robodog/device.json` with mode `0600`. It identifies a
robot across address changes; it is not a credential.

Useful launch overrides are:

| Argument | Default | Purpose |
|---|---|---|
| `web_discovery` | `true` | Enable DNS-SD advertisement |
| `device_name` | `RoboDog` | Friendly name shown by clients |
| `device_identity_file` | empty | Absolute UUID state file; empty uses `ROS_HOME` |
| `web_host` | `0.0.0.0` | Bind address; `127.0.0.1` also suppresses DNS-SD |
| `web_port` | `8080` | HTTP, WebSocket and MJPEG port |

If Avahi is missing or multicast is blocked, discovery fails without stopping
the web server. Enter `http://<robot-ip>:8080` manually in the app.

## Build and run the app

```bash
cd apps/robodog_mobile
npm ci
npm test
npm run typecheck

# A development build is required for native Bonjour/Android NSD discovery.
npx expo run:android
# or, on macOS with Xcode:
npx expo run:ios
```

Expo Go supports the controller, camera and manual address entry. Use a
development build for automatic DNS-SD discovery because Expo Go does not
contain the project-local Swift/Kotlin discovery module.
iOS declares local-network use and `_robodog._tcp` Bonjour access. Android uses
Network Service Discovery and requests the corresponding network permissions.

## Operator workflow

1. Keep the phone/tablet and robot on the same isolated LAN.
2. Select the discovered UUID/name, or enter the robot address manually.
3. Check mode, gait, measured speed, backend, the configured supply-voltage
   assumption, hottest joint, and rolling torque RMS/peak.
4. Wait for the cockpit's control status to show **READY**. Touch and drag the
   **MOVE** joystick for forward/lateral velocity or the **TURN** joystick for
   yaw. Each stick acts as a dead-man control; both can be held together, remain
   side by side in portrait, and spring back to centre when released.
5. Use **Stand**, **Walk**, **Greeting** and **EMERGENCY STOP** directly from
   the cockpit. The E-stop is latched by the robot.
6. Open **Web Remote** for the full browser controller and map export. Its
   navigation remains restricted to the selected robot origin.

The app routes cockpit commands through a hidden, navigation-pinned WebView at
`/remote?native_bridge=1`. This is an explicit bridge mode of the robot-served
page, not a standalone native WebSocket: the page opens `/ws`, so its Origin
and Host exactly match the selected robot and the server's hostname allowlist
is unchanged. The bridge is present only when the URL opt-in and React Native
message channel both exist. It accepts a strict, finite command schema and
returns validated readiness, connection, acknowledgement and state messages.
An ordinary browser visiting `/remote` does not activate it.

The existing one-operator 0.30 s drive lease and the controller's independent
0.35 s body-velocity timeout remain authoritative. The app refreshes a held
command at 10 Hz while either stick is touched and sends zero when both sticks
are released, when the app is backgrounded, on screen exit and on bridge
teardown. It releases locally on link loss; the server publishes zero on
disconnect or lease expiry. Releasing one joystick recentres its axis.

## Security boundary

Discovery is public LAN metadata and never grants motion authority. The current
HTTP/MJPEG/WebSocket server has no login or TLS. Use it only on a trusted,
isolated network; do not expose port 8080 to the internet.

The current same-origin bridge does not replace the planned production security
layer. Before this interface is used outside a trusted LAN, add:

- TLS with pinned robot identity;
- a short, time-limited pairing window and code displayed locally;
- revocable per-device tokens stored in Keychain/Keystore;
- strict schemas, finite-number checks, message-size and rate limits;
- an explicit authenticated control lease covering every motion-changing
  action; and
- zero/release on touch release, stale telemetry, background, socket error,
  disconnect and unmount.

The existing browser `/ws` Origin, Host and private-network checks must remain
in place. The current bridge deliberately preserves them and does not add a
native socket exception.

## Camera path and 60 fps plan

The current preview is MJPEG. The simulated camera is configured at 15 Hz and the
web encoder is capped at 10 Hz and 640 pixels wide. It cannot deliver 60 fps.
Video is kept separate from command telemetry so a slow frame cannot delay a
dead-man refresh.

The future media adapter should use Jetson hardware H.264 encoding and WebRTC
with a native React Native decoder. Before enabling a `720p60` profile, record:

- verified sensor output mode and actual encoded/decoded frame rate;
- dropped frames, bitrate and packet loss;
- glass-to-glass latency and jitter;
- Jetson CPU/GPU/encoder load and temperature; and
- command-channel latency while the video link is saturated.

Keep a lower-bandwidth profile such as `480p30` and retain MJPEG as a diagnostic
fallback.
