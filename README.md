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

Launch arguments: `port`, `baud_rate`, `modbus_id`, `publish_period`.

| Interface | Type | Purpose |
| --- | --- | --- |
| `dh5/AxisInfos` (topic) | `AxisInfos` | position, velocity, current, faults of every axis |
| `dh5/initialize` | `Initialize` | initialize the hand (mode 1 close, 2 open, 3 find stroke) |
| `dh5/set_position`, `set_speed`, `set_force` | `SetValues` | all axes at once |
| `dh5/set_axis_position`, `set_axis_speed`, `set_axis_force`, `move_axis_percent` | `SetAxisValue` | one axis |
| `dh5/move_axes_percent` | `MoveAxesPercent` | several axes to different percentages, simultaneously |
| `dh5/clear_cur_fault`, `clear_history_faults`, `restart_system`, `get_faults` | `std_srvs/Trigger` | fault handling |
| `dh5/<gesture>` | `Trigger` or `TwoFingerPinch` | one service per entry in `dh5/gestures.py` |

```bash
ros2 service call /dh5/initialize dh5_interfaces/srv/Initialize "{mode: 2}"
ros2 service call /dh5/open_hand std_srvs/srv/Trigger
ros2 service call /dh5/two_finger_pinch dh5_interfaces/srv/TwoFingerPinch "{width: 10, axis_mode: 'axis2'}"
ros2 topic echo /dh5/AxisInfos
```

Adding an entry to `GESTURES` gives both the `DH5>` prompt and the ROS 2 node a
new command; no other file changes.

## Tests

```powershell
.venv\Scripts\python -m pytest
```

The suite runs the real transport against a simulated DH5 and tests the ROS 2
node's logic with stubbed `rclpy`, so neither hardware nor ROS is required.
