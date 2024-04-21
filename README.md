# robot_autonomy_ws

1. Install turtlebot3 (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
*Note: Use instruction for ros2 humble*

2. Clone this repository
```shell
cd <this_repository>
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch project_demo_bringup project_demo.launch.py
```

3. To move the robot, run the following commands in a new terminal 
```shell
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

