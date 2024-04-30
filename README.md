# robot_autonomy_ws

## Setup

```shell
git submodule update --init --recursive
colcon build
rosdep install --from-paths src --rosdistro humble -y
source install/setup.bash
source sim_setup.bash
```

## Running the whole project

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

## (alternative) Running the behavior tree

```shell
source install/setup.bash
source sim_setup.bash
ros2 launch my_turtlebot turtlebot_simulation.launch.py params_file:=src/final_project/resource/nav2_params.yaml
```

After the simulation is running, set the 2D pose in the middle of the Rviz. Click with the Goal Pose any where on the map to start the BT.
