# RoboDog mobile app

`apps/robodog_mobile` is an Expo/React Native TypeScript client for phones and
tablets. It discovers a robot on the local network, validates its public
descriptor and telemetry, shows a responsive cockpit, and provides a working
same-origin Web Remote for motion control.

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

Expo Go can exercise screens that do not require the native discovery module,
using manual address entry. Use a development build for automatic discovery.
iOS declares local-network use and `_robodog._tcp` Bonjour access. Android uses
Network Service Discovery and requests the corresponding network permissions.

## Operator workflow

1. Keep the phone/tablet and robot on the same isolated LAN.
2. Select the discovered UUID/name, or enter the robot address manually.
3. Check mode, gait, measured speed, backend, the configured supply-voltage
   assumption, hottest joint, and rolling torque RMS/peak.
4. Open **Web Remote** for working motion controls, camera, Greeting and map
   export. Navigation is restricted to the selected robot origin.
5. Hold the dead-man input while moving. Releasing, backgrounding, losing
   focus, disconnecting or expiring either command timeout sends or produces
   zero velocity.
6. Inside **Web Remote**, use the persistent red emergency stop for a latched
   stop.

The native cockpit's direct motion buttons are intentionally locked in this
version. React Native clients do not have a trustworthy browser Origin, and
accepting missing or `null` origins on `/ws` would weaken the working web
boundary. Web Remote runs the robot-served page at `/remote`, so its WebSocket
has the exact same origin and retains the one-operator 0.30 s drive lease and
the controller's independent 0.35 s body-velocity timeout.

## Security boundary

Discovery is public LAN metadata and never grants motion authority. The current
HTTP/MJPEG/WebSocket server has no login or TLS. Use it only on a trusted,
isolated network; do not expose port 8080 to the internet.

Direct native motion control requires a separate protocol before it is enabled:

- TLS with pinned robot identity;
- a short, time-limited pairing window and code displayed locally;
- revocable per-device tokens stored in Keychain/Keystore;
- strict schemas, finite-number checks, message-size and rate limits;
- an explicit control lease covering velocity, gait, pose, Greeting, enable,
  joint jog and emergency-stop clearing; and
- zero/release on touch release, stale telemetry, background, socket error,
  disconnect and unmount.

The existing browser `/ws` Origin and private-host checks must remain in place
when that endpoint is added.

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
