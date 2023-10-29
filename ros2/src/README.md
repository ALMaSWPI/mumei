HURON ROS2 Module
-----------------

This module contains ROS2 packages that support using ROS2 for simulation or hardware control. It includes the HURON description files (URDF/xacro), Gazebo 11 support, and example code. 
The ROS packages can be found in `src/`.

We are using [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).

List of packages:
<ul>
  <li>huron_description: includes meshes, a macro xacro file to generate a HURON robot, a top-level xacro file to be visualized in RViz.</li>
  <li>huron_gazebo: Gazebo 11-specific configuration, including sensor/control plugins and a Gazebo launch file.</li>
  <li><p>examples: Example of using HURON API with ROS2 <strong>(TBA)</strong></p></li>
</ul>

***

### Usage:

Note: the steps below assume you have ROS2 Humble installed.

#### Building the ROS Workspace:
1. Prerequisites:
- You have built and installed the main `huron` project (instructions [here](../README.md)).
- Make sure you are in `huron/ros2/`.
2. Build the workspace:

```colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON```

This command generates 3 new folders in `huron/ros2/`: `build/`, `install/`, and `log/`.
Command explanation:
- `--symlink-install`: The output files will be symlinks to source files, which means you don't have to re-source the workspace after modifying an **already built** file.
- `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`: Generates a `compile_commands.json` for auto-completion with [clangd](https://clangd.llvm.org/).
- Tip: you can set an alias to call this tedious command faster:

```alias cb='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'```

Then, you can simply do `cb` instead of the full command.

#### Launching Gazebo 11 simulation
1. Source the workspace setup file

```. install/setup.bash```

2. Launch Gazebo:

```ros2 launch huron_gazebo gazebo.launch.py```
