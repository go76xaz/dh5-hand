# dh5-hand

Everything needed to operate a DH-Robotics **DH5 six-axis hand**, either from a
Windows terminal or through ROS 2. One Python library (`dh5`) does the work;
the command prompt, the GUI and the ROS 2 node are thin layers on top of it.

```
dh5-hand/
├── src/dh5/                  Python library + command prompt + GUI
│   ├── registers.py            register addresses and axis limits
│   ├── conversions.py          percent <-> raw values
│   ├── modbus.py               serial Modbus RTU transport
│   ├── hand.py                 DH5Hand: axes, percent moves, sensors, faults
│   ├── gestures.py             poses as a data table (open_hand, pinch, wink, ...)
│   ├── cli.py                  the DH5> command registry
│   ├── __main__.py             `dh5` / `python -m dh5` entry point
│   └── gui.py                  Tkinter slider GUI (`dh5-gui`)
├── ros2/
│   ├── dh5_interfaces/         .msg / .srv definitions
│   └── dh5_controller/         the node, launch file and small service clients
├── tests/                    pytest suite against a simulated hand (no hardware)
└── docs/                     vendor user manual and API reference
```

## Windows terminal

```powershell
python -m venv .venv
.venv\Scripts\pip install -e ".[dev]"

dh5 --port COM3            # initialize the hand, then a DH5> prompt
dh5 --help                 # all options
dh5-gui                    # slider GUI
```

Type `help` at the `DH5>` prompt for the commands, including one per gesture.

As a library:

```python
from dh5 import DH5Hand, configure_logging, gestures

configure_logging()
with DH5Hand(port="COM3") as hand:
    hand.initialize()
    hand.move({2: 100, 5: 100})                     # both axes start in one frame
    gestures.perform(hand, "two_finger_pinch", width=12)
```

Find the COM port in Device Manager under *Ports (COM & LPT)*.

## ROS 2

ROS 2 needs Linux (or WSL2 / Docker), not native Windows. On WSL2, forward the
USB serial adapter first with `usbipd`. Workspace layout: link or copy `ros2/`
into `<workspace>/src/`.

```bash
pip install ./dh5-hand                  # the node imports the dh5 library
cd <workspace>
colcon build --packages-select dh5_interfaces dh5_controller
source install/setup.bash

ros2 launch dh5_controller controller.launch.py port:=/dev/ttyUSB0
```

Launch arguments (all passed through as node parameters):

| Argument | Default | Meaning |
| --- | --- | --- |
| `port`, `baud_rate`, `modbus_id` | `/dev/ttyUSB0`, `115200`, `1` | serial connection |
| `state_rate_hz` | `10.0` | rate of `dh5/AxisInfos` and `dh5/joint_states` |
| `sensor_rate_hz` | `10.0` | rate of `dh5/fingertips` |
| `stale_timeout` | `1.0` | seconds without any change before a fingertip is flagged stale |
| `move_poll_interval` | `0.05` | seconds between status polls while a move runs |
| `contact_threshold` | `0.5` | default change of (mx, my, fz) that counts as contact |
| `move_timeout` | `30.0` | default timeout of a `dh5/move_axes` goal, seconds |
| `publish_period` | - | deprecated, use `state_rate_hz` |

Both publishers share one serial bus. The node logs the time one read cycle
really takes after startup; if it warns that a rate cannot be reached, lower it.

| Interface | Type | Purpose |
| --- | --- | --- |
| `dh5/AxisInfos` (topic) | `AxisInfos` | raw and percent position, velocity, current, status (moving / reached / stalled), faults of every axis |
| `dh5/joint_states` (topic) | `sensor_msgs/JointState` | position of every axis in **percent of stroke** (not radians) |
| `dh5/fingertips` (topic) | `FingertipArray` | mx, my, fz of every fingertip plus a `stale` flag for frozen data; only on 3-axis (Hualichuang) sensors |
| `dh5/move_axes` (action) | `MoveAxes` | non-blocking move with optional per-axis speed/force, progress feedback, cancel, and optional stop on fingertip contact |
| `dh5/stop` | `std_srvs/Trigger` | stop every axis where it is; also ends a running move, gesture or action |
| `dh5/calibrate_sensors` | `std_srvs/Trigger` | zero the fingertip sensors - hand at rest, nothing touching the fingertips |
| `dh5/initialize` | `Initialize` | initialize the hand (mode 1 close, 2 open, 3 find stroke) |
| `dh5/set_axis_position`, `set_axis_speed`, `set_axis_force`, `move_axis_percent` | `SetAxisValue` | one axis |
| `dh5/move_axes_percent`, `set_axes_speed` | `MoveAxesPercent`, `SetAxesValues` | several axes at once, others left untouched |
| `dh5/raw/set_position`, `set_speed`, `set_force` | `SetValues` | low-level: raw register units, all 6 axes required, no completion wait - prefer the services above |
| `dh5/clear_cur_fault`, `clear_history_faults`, `restart_system`, `get_faults` | `std_srvs/Trigger` | fault handling |
| `dh5/<gesture>` | `Trigger` or `TwoFingerPinch` | one service per entry in `dh5/gestures.py` |

```bash
ros2 service call /dh5/initialize dh5_interfaces/srv/Initialize "{mode: 2}"
ros2 service call /dh5/open_hand std_srvs/srv/Trigger
ros2 service call /dh5/two_finger_pinch dh5_interfaces/srv/TwoFingerPinch "{width: 10, axis_mode: 'axis2'}"
ros2 topic echo /dh5/AxisInfos

ros2 service call /dh5/calibrate_sensors std_srvs/srv/Trigger
ros2 topic echo /dh5/fingertips
# close index and middle at 30 % speed, each stopping as soon as it touches something
ros2 action send_goal --feedback /dh5/move_axes dh5_interfaces/action/MoveAxes \
  "{axes: [2, 3], percents: [0.0, 0.0], speeds: [30, 30], stop_on_contact: true}"
ros2 service call /dh5/stop std_srvs/srv/Trigger
```

Commands (services and action goals) run one at a time; one that arrives while
another runs is refused as busy. `dh5/stop` is the exception and works at any
time. The DH5 has no stop register, so stopping commands each axis's measured
position as its new target, which means a small overshoot of one bus round trip.

`stop_on_contact` compares each finger's reading against the one taken when
the goal started, so sensor drift does not matter. Axes 1 and 6 both use the
thumb sensor. A finger whose data freezes cannot report contact; the result
message says so.

Adding an entry to `GESTURES` gives both the `DH5>` prompt and the ROS 2 node a
new command; no other file changes.

## Tests

```powershell
.venv\Scripts\python -m pytest
```

The suite runs the real transport against a simulated DH5 and tests the ROS 2
node's logic with stubbed `rclpy`, so neither hardware nor ROS is required.
