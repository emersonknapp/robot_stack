# Robot Stack
Bringup entrypoint and configurations for home robot project.

Exec_depends on packages and provides launchfiles, rviz configs, and other configurations, but does not directly provide any code.

## Run

Simulation workflow is launched via

```
ros2 launch robot_stack sim_neato_willow.launch.py
```

It runs cartographer SLAM. Once you have a map

```
ros2 run nav2_map_server map_saver
```

TODO document hardware workflow
