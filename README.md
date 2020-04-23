# robot_stack

ROS2 packages specific to [Home Robo](https://github.com/emersonknapp/robo)


## Packages

### launch_candy

Common utilities for writing shorter Python launchfiles

### parking

"Parking Spots" - named poses saved on a navigation map, used to navigate to a preset pose by name.

Provides services for renaming, moving, deleting, adding, and navigating to the parking spots.

### parking_rviz_plugins

RViz extension to place and modify parking spots interactively in the GUI.

### robot_indicators

Lighting and other on-robot indicators of internal state. (e.g. xpad player LEDs for different states)

### robot_runtime

No functionality. Entrypoint launchfiles and config files for the nodes that run on the headless robot device.

### robot_stack

No functionality. Entrypoint launchfiles and config files for remote visualization, and simulation workflows.
