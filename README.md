# LQR Nav2 Controller Plugin

A custom Nav2 controller plugin implementing an LQR (Linear Quadratic Regulator) controller for ROS2 Humble.

## Project Structure

```
LQR-Controller/
├── .devcontainer/
│   ├── devcontainer.json          # VS Code dev container config
│   └── Dockerfile                 # ROS2 Humble + Nav2 image
├── src/
│   └── lqr_controller/            # ROS2 package
│       ├── include/
│       │   └── lqr_controller/
│       │       └── lqr_controller.hpp      # Controller class header
│       ├── src/
│       │   └── lqr_controller.cpp          # Controller implementation
│       ├── CMakeLists.txt                  # Build config
│       ├── package.xml                     # ROS2 package manifest
│       ├── lqr_controller_plugin.xml       # Nav2 pluginlib descriptor
│       └── LICENSE
├── .gitignore
├── ros_entrypoint.sh
├── proposal.md
└── README.md
```

## Getting Started

1. Open the project in VS Code

2. When prompted "Reopen in Container", click yes. Or manually: `Ctrl+Shift+P` > `Dev Containers: Reopen in Container`

3. Wait for the container to build (first time ~5-10 min, subsequent opens use cache)

4. Once inside the container, build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

# References:
[1] https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html#requirements
[2] https://github.com/ros-navigation/navigation2_tutorials/tree/126902457c5c646b136569886d6325f070c1073d/nav2_pure_pursuit_controller