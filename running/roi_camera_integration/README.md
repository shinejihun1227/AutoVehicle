# ROI Front Camera Integration

This directory contains a Front-camera-only raw UDP integration for the
`hyunho0429/ROI` `dev/stanley` controller.

It is deliberately separate from the ROS camera bridge. MORAI continues to
send Camera UDP to Ubuntu, and ROI keeps sending Ego Ctrl Cmd UDP to MORAI.
The camera output is used internally by the ROI control loop.

## Configuration

```text
MORAI Host IP:          192.168.0.151
Ubuntu bind IP:         0.0.0.0
Front destination port: 1101
Expected image:         1280 x 720
```

The Host Port `1100` is the MORAI source port. The Ubuntu receiver binds the
Destination Port `1101`.

## Copy to Ubuntu ROI

From the Windows workspace, copy these four files into the ROI package:

```text
~/ROI/src/path_planning/src/path_planning/front_camera_udp.py
~/ROI/src/path_planning/src/path_planning/front_camera_perception.py
~/ROI/src/path_planning/src/path_planning/camera_behavior.py
~/ROI/src/path_planning/src/front_camera_debug.py
```

The first three files are importable by the ROI Python package. The debug
script can be run directly from the `src` directory after setting `PYTHONPATH`
or copied as a package script.

In the ROI launch file, add this argument:

```xml
<arg name="camera_port" default="1101"/>
```

Pass it through the node's `args` attribute:

```xml
--camera-port $(arg camera_port)
```

In `pure_pursuit_udp_runtime.py`, add this parser argument inside
`argument_parser()`:

```python
parser.add_argument("--camera-port", type=int, default=1101)
```

## Test camera before touching the controller

```bash
cd ~/ROI/src/path_planning/src
source /opt/ros/noetic/setup.bash
PYTHONPATH="$PWD:$PYTHONPATH" python3 front_camera_debug.py \
  --bind-ip 0.0.0.0 \
  --port 1101 \
  --save-dir /tmp/roi_front_camera
```

Expected output includes:

```text
Front camera listening on 0.0.0.0:1101
datagrams=... frames=... invalid=... dropped=... pending=... invalid_reasons=...
```

The diagnostic counters distinguish the common failure modes:

- `datagrams=0`: Ubuntu's socket is not receiving packets.
- `invalid_reasons=invalid_header=...`: the packet header is rejected.
- `invalid_reasons=payload_size_exceeds_packet=...`: the packet size field
  does not match the received UDP datagram.
- `invalid_reasons=invalid_tail=...`: the packet tail is not `AI` or `EI`.
- `pending>0, frames=0`: fragments are arriving but no complete JPEG has
  been assembled yet.
- `non_jpeg` increasing: a frame was assembled but did not start/end with
  JPEG markers.
- `timeouts` increasing: an incomplete frame expired before its fragments
  arrived.

If the resolution is not `1280x720`, check the MORAI sensor setting. If the
frame count stays at zero, check the UDP IP/port and run:

```bash
sudo tcpdump -ni any \
  'udp and src host 192.168.0.151 and dst port 1101'
```

If `datagrams` increases but `frames=0`, inspect the actual packet layout:

```bash
cd ~/AutoVehicle/running/roi_camera_integration
python3 front_camera_packet_probe.py \
  --bind-ip 0.0.0.0 \
  --port 1101 \
  --count 3
```

The probe prints the datagram length, little/big-endian metadata candidates,
JPEG marker positions, and `AI`/`EI` marker positions. Stop the old debug
receiver with `Ctrl+C` before starting the probe because both programs cannot
bind the same UDP port at the same time.

## ROI control-loop integration

The ROI runtime already uses `selectors.DefaultSelector`. Add the imports:

```python
from path_planning.front_camera_udp import FrontCameraUdpReceiver
from path_planning.front_camera_perception import FrontCameraPerception
from path_planning.camera_behavior import FrontCameraBehavior
```

After creating the main selector, create and register the camera socket:

```python
camera_receiver = FrontCameraUdpReceiver(
    bind_ip=arguments.bind_ip,
    port=arguments.camera_port,
)
camera_perception = FrontCameraPerception(
    resize_width=640,
    process_rate_hz=15.0,
)
camera_behavior = FrontCameraBehavior()
latest_camera_observation = None
selector.register(camera_receiver.socket, selectors.EVENT_READ, "camera")
```

In the selector packet branch, add a `camera` branch before the existing
collision branch:

```python
elif key.data == "camera":
    frame = camera_receiver.feed_datagram(packet, sender, received)
    if frame is not None:
        latest_camera_observation = camera_perception.process_jpeg(
            frame.jpeg, frame.received_monotonic
        )
        if latest_camera_observation is not None:
            camera_behavior.update(latest_camera_observation)
```

Immediately after the ROI controller computes `accel`, `brake` and
`normalized_steering`, apply the camera behavior gate:

```python
accel, brake, normalized_steering = camera_behavior.apply(
    accel,
    brake,
    normalized_steering,
    now_monotonic=now,
)
command = pedal_command(accel, brake, normalized_steering)
```

For the existing ROI control loop, the exact placement is inside the
`else` branch after:

```python
accel, brake = speed_controller.compute(
    target_speed_mps, state.speed_mps, now
)
```

and before the existing `command = pedal_command(...)` line. The camera gate
must not replace the existing GPS/IMU localization or Pure Pursuit result; it
only overrides the final accel/brake/steering command.

Close the camera socket in the runtime `finally` block:

```python
camera_receiver.close()
```

The default behavior is intentionally conservative:

- red traffic light plus stop line: request full brake;
- green traffic light: release the camera stop request;
- camera stale or unavailable: keep the GPS/IMU path controller running;
- lane steering correction: disabled by default (`lane_steer_gain=0.0`).

Do not enable lane steering correction until the sign and scale have been
calibrated on the actual fixed Front-camera view.

## Automated tests before MORAI integration

The camera integration has unit tests that do not require a running MORAI
instance. They cover UDP frame reassembly, invalid/expired packet handling,
the conservative camera behavior gate, and dependency-free multi-camera lane
fusion. OpenCV image tests run automatically when `python3-opencv` is
installed; otherwise they are reported as skipped.

From the `running` directory:

```bash
cd running
python3 -m unittest discover -s roi_camera_integration/tests -p "test_*.py" -v
```

The tests intentionally do not send Ego Ctrl Cmd packets and do not modify the
Pure Pursuit or Stanley runtime. The next integration step should only be
started after these tests pass on the Ubuntu target.
