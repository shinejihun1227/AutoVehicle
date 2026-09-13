# Highway integration verification

Upstream baseline: AutoVehicle `3330360` and ROI `13b740689c280d04fd456b00b854efbd793c38c2`.

Executed locally on Windows, with ROS message/transport stubs where required:

| Suite | Passed tests |
|---|---:|
| Camera (including new timestamp/coordinate/stopline contract) | 73 |
| LiDAR (including road-aligned merge gap) | 50 |
| Frenet/highway/managed safety integration | 52 |
| Stopline | 54 |
| Turn/signal route fusion | 200 |
| Existing blackout stack | 41 |
| Curvature (including dynamic path callbacks) | 22 |
| Total | 492 |

The existing highway speed-filter test fixture was adjusted to represent the first
lane change, because the integrated node now defaults to at most one change.
The new count-limit test independently covers that restriction. After the initial
fixture failure, the entire avoidance suite was rerun successfully.

Python source was parsed against Python 3.8 syntax. A separate static launch walk
resolved all local package paths, include arguments and executable filenames:
27 nodes with YOLO/control off, 27 with YOLO/control enabled, and 22 without YOLO.
No duplicate node name or second nominal Pure Pursuit was found. The shell helper
passed `bash -n`.

Not executed locally: Docker build, catkin build, real ROS launch, new model load /
inference, live GPU execution, network exchange with the new Ubuntu host, MORAI
driving or actual vehicle validation. Docker/ROS/PyTorch are unavailable in this
Windows tool environment. The Docker build explicitly runs the message contract,
regressions, real launch resolution and model inference; it fails if these checks
do not pass. Simulator scenarios still require operator testing.

Existing blackout tests validate that package; they do not imply that the new
highway launch enables its fallback controller.
