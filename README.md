# LQR Nav2 Controller Plugin

A custom Nav2 controller plugin implementing an LQR (Linear Quadratic Regulator) controller for ROS2 Humble.

## Getting Started

1. Open the project in VS Code

2. When prompted "Reopen in Container", click yes. Or manually: `Ctrl+Shift+P` > `Dev Containers: Reopen in Container`

3. Wait for the container to build (first time ~5-10 min, subsequent opens use cache)

4. Once inside the container, build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```
