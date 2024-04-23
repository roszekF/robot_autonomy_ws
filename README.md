# robot_autonomy_ws

Setup

```shell
git submodule update --init --recursive
colcon build
rosdep install --from-paths src --rosdistro humble -y
source install/setup.bash
source sim_setup.bash
```

in one terminal run

```shell
ros2 launch project_demo_bringup project_demo.launch.py
```

If you want to drive robot manually, run this in another terminal

```shell
source install/setup.bash
source sim_setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```
