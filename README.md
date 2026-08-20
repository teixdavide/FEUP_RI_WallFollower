# Authors

- Davide Texeira 202109860
- Pedro Oliveira 202108669
- Tomás Palma 202108880

# Directory structure

- The URDF robot definition file is in the `resources` folder.
- The world definition is in the `my_world.wbt` file inside of the `worlds` folder.
- The robot code modules in C++ are inside the `src` folder.
- The custom controller `position_track` is inside the `controllers/` folder. However, in running we had to put it inside a folder called `/usr/local/webots/resources/projects/controllers/position_track`

# Requirements

- Webots
- Ros2 Jazzy

# Compile

```
colcon build
source install/local_setup.bash
ros2 launch ri_assign1 robot_launch.py
```
