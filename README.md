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
source install/setup.bash
source sim_setup.bash
ros2 launch my_turtlebot turtlebot_simulation.launch.py
```

this in another one

```shell
source install/setup.bash
source sim_setup.bash
ros2 run final_project map_publisher
```

and this in a third one

```shell
source install/setup.bash
source sim_setup.bash
ros2 run final_project bt_publisher
```
