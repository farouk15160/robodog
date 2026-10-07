# Remote control

The web GUI has separate **Dashboard** and **Remote Control** pages. Remote
Control is designed for a phone, tablet or laptop and keeps the camera and
essential robot feedback beside the motion controls. It reports controller
state and gait, measured base speed and yaw rate, position, the most-loaded and
hottest joints, backend/world and mapping status. The full joint and rolling
RMS diagnostics remain on the Dashboard.

## Start and connect

Launch normally. Open <http://localhost:8080/remote> on the robot, or find the
robot's LAN address with `hostname -I` and open
`http://<robot-ip>:8080/remote` from a phone, tablet or laptop on the same
network. The **Remote Control** tab and browser back/forward navigation use the
same route:

```bash
ros2 launch robodog_bringup robot.launch.py backend:=mujoco world:=house
```

The web server also advertises `_robodog._tcp.local` by default when
`avahi-publish-service` is installed and the Avahi daemon is running. The
service instance has the friendly name; its TXT record contains a persistent
device UUID, model, API version and the path `/.well-known/robodog`. It does not
contain credentials. Mobile clients verify that descriptor after resolving the
service. Multicast-blocked networks can still use the IP address manually.
Set `web_discovery:=false` to disable advertisement, or set
`device_name:="Workshop RoboDog"` to distinguish several robots.
The React Native client uses this service and retains manual address entry; see
the [mobile app runbook](mobile_app.md).

The app's portrait controller keeps the camera and telemetry above two
side-by-side joysticks in a bottom control area; landscape uses a split cockpit.
Its active controls use a hidden WebView at `/remote?native_bridge=1`. The
robot-served page, rather than React Native, opens `/ws`, preserving the exact
Origin/Host checks and hostname allowlist. Bridge input uses a strict command
schema and retains the drive lease and dead-man timeouts described below. The
app sends zero on dead-man release and background/screen exit; disconnect or
lease expiry also produces zero. This does not create a standalone native
command socket. Expo Go works with manual address entry, while automatic DNS-SD
discovery requires the native development build.

The server listens on all network interfaces by default. To restrict it to the
robot itself, bind it to loopback. Loopback binding also suppresses DNS-SD
advertisement:

```bash
ros2 launch robodog_bringup robot.launch.py \
  backend:=mujoco world:=house web_host:=127.0.0.1
```

The server requires the command WebSocket to have the GUI's same origin, but it
also restricts command origins to `localhost`, literal private/link-local IP
addresses, the robot computer's advertised `hostname.local`, or names listed
in `allowed_command_hosts` in `web.yaml`. Other DNS hostnames remain blocked
against rebinding. The server does not provide login, authorization or
TLS. The default LAN binding is for an isolated, trusted network. Do not expose
port 8080 to the internet. Put an authenticated TLS reverse proxy and network
access control in front of it if it must cross an untrusted network, and add the
proxy's DNS name to `allowed_command_hosts`.

## Drive controls

Press **Start travel (trot)**, then hold a control to move. The remote selects
the tuned trot controller because the recorded 1 m/s walk operating point is
unstable; see [locomotion validation](locomotion_validation.md).

| Input | Command |
|---|---|
| Left touch joystick | Forward/back and lateral velocity |
| Right touch joystick | Yaw rate |
| `W` / `S` or up/down arrows | Forward/back |
| `A` / `D` | Lateral left/right |
| `Q` / `E` or left/right arrows | Turn left/right |

In the React Native cockpit, **MOVE** provides forward/lateral velocity and
**TURN** provides yaw. Once the cockpit shows **READY**, touching either stick
enables motion. Releasing one stick recentres that axis; releasing both sticks
commands zero velocity.

The speed slider starts at **0.5 m/s**. It remains bounded by the server's
backend limit: simulation allows up to 2.0 m/s, while hardware remains limited
to 0.5 m/s. Releasing touch, mouse or keys sends zero velocity immediately.
Changing page, hiding the tab or losing window focus also sends zero.

While a control is held, the browser refreshes the velocity command at 10 Hz.
The web server grants one connected operator a 0.30 s drive lease and rejects
another operator's nonzero command while that lease is active. A disconnect or
expired lease publishes zero. Independently, the 400 Hz controller zeros a
streamed body velocity after 0.35 s without a fresh `/cmd_vel` message. These
timeouts stop translation and yaw; they leave the selected gait controller
active. Use **Stand** or **STOP MOVEMENT** to select standing as well as zeroing
velocity. The red emergency-stop buttons remain the control for an actual
latched stop.

## Greeting

**Greeting** performs a short front-left-leg wave and returns to the stand
pose. One press is enough: in MuJoCo the controller first enables the joints,
moves to stand, and waits for finite feedback, four foot contacts, a level and
stationary body at stand height, and joints within 0.15 rad of the stand pose.
It then shifts the body rear-right before lifting the paw, keeps the base inside
the remaining three-foot support triangle, plants the paw, and recentres. A
queued greeting expires after 10 seconds rather than starting from a fallen or
unsettled body. The emergency stop rejects the request. A new pose, joint,
gait, nonzero velocity, disable or emergency-stop command cancels a queued
greeting; velocity or gait input also cancels an active greeting and returns to
stand.

The current MuJoCo validation covered one complete 8.70 s greeting with 435
state samples: minimum base height 0.294 m, maximum absolute roll 0.067 rad
(3.84 degrees), maximum absolute pitch 0.021 rad (1.21 degrees), and at least
three contacts throughout the lifted phase. The kinematic test enforces at
least 40 mm support-triangle margin while paw clearance is at least 60 mm and
limits peak commanded joint speed to 1.25 rad/s. This remains a
**simulation-only** behavior until it has been validated on the assembled
robot; hardware rejects it. The exact launch, measurement command and raw values
are preserved in [`greeting_validation.json`](greeting_validation.json); rerun
the measurement with `python3 tools/validate_greeting.py` from the repository
root after sourcing `ros2_ws/install/setup.bash`.

## Save a room scan

When RTAB-Map is active, **Save room scan** creates one immutable export of the
map accumulated so far. The optional name accepts 1–64 letters, digits, dots,
underscores or dashes and must start with a letter or digit. The browser cannot
choose a server filesystem path. By default exports go below
`${ROS_HOME:-~/.ros}/robodog/exports`; an operator can set a fixed server-side
root with the `mapping_export_directory:=/absolute/path` launch argument.

The equivalent CLI supports an explicit absolute destination:

```bash
ros2 run robodog_perception save_map \
  --output-dir /absolute/path/to/room-scans --name workshop
```

Each completed session directory contains:

- `cloud_map.pcd`: binary PCD v0.7 from RTAB-Map's assembled
  `/robodog/mapping/cloud_map`, including the fields supplied by that map;
- `octomap.ot`: the full colored occupancy tree for occupied, free and unknown
  3D space;
- `rtabmap.db.back`: a consistent database backup containing the SLAM graph and
  observations; and
- `manifest.json`: format/version, source metadata, point count, frame, and
  SHA-256 plus byte size for every artifact.

The manifest is written last. A failed export removes partial artifacts, and an
existing session name is never overwritten. Move through the room and wait for
mapping updates before saving; the service rejects an empty or unavailable
assembled cloud. It also rejects a map whose assembled cloud has not updated
within the preceding five seconds. The persistent live database and a saved room-scan bundle
serve different purposes: restart from the database to continue mapping, and
use PCD/OctoMap files in inspection and downstream mapping tools. See the
[mapping runbook](mapping.md) for frames, topics and validation boundaries.
