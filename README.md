# robot_autonomy_ws

Setup

```shell
source install/setup.bash
rosdep install --from-paths src --rosdistro humble -y
```

in one terminal run

```shell
ros2 launch my_turtlebot turtlebot_simulation.launch.py
```

and this in another one

```shell
ros2 run final_project map_publisher
```
