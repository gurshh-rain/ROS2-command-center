# ros2-cmdctr

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
- `rclpy` and standard ROS 2 message packages

## Running

Activate your ROS 2 environment and run:

```bash
python app.py
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
# ROS2-command-center
