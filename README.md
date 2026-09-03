# ROS 2 Command Center (ros2-cmdctr)

A terminal-based command center for inspecting and controlling ROS 2 systems.

Built with [Textual](https://textual.textualize.io/) and `rclpy`.

## Features

- Live topic, node, service, and action discovery
- Topic bandwidth, rate, publisher/subscriber counts
- TF tree health and diagnostics overview
- Remote parameter inspection and editing
- Service calls and one-shot topic publishing
- Managed `ros2 bag record` and `ros2 launch` process controls
- Live `/rosout` logs and `/diagnostics` status streams

## Requirements

- ROS 2 Humble (or newer)
- Python 3.9+
- `textual >= 8.2, < 9`
- `PyYAML >= 6, < 7`
- `rclpy`, `ros2cli`, and standard ROS 2 message packages

## Setup

### 1. Clone into a ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/gurshh-rain/ROS2-command-center.git node_viewer
cd node_viewer
```

### 2. Install Python dependencies

Make sure your ROS 2 Python environment is active, then:

```bash
pip install -e .
```

This installs `textual`, `PyYAML`, `ros2cli`, and the package itself in editable mode.

### 3. Build with colcon (recommended)

From the workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select node_viewer
source install/setup.bash
```

> If you already ran `pip install -e .` and only want the `ros2 cmd_center` verb, you can skip `colcon build`. The `ros2 run` command still requires the colcon build step.

## Running

Launch the TUI with the new `ros2` verb:

```bash
ros2 cmd_center
```

Or run it directly with Python:

```bash
python app.py
```

Or use the installed console script:

```bash
ros2 run node_viewer cmd_center
```

## Controls

- `1-9` — switch views
- `r` — refresh
- `p` — pause
- `e` — toggle echo
- `x` — hide selected item
- `q` — quit

## License

Open source under the MIT License. See [LICENSE](LICENSE).
