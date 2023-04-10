Source code goes in "src"
Any code we want to run should be a script in "scripts"
Any output should be saved to "results"
Package requirements are in the requirements.txt

---

## ROS 2 WS
ROS Humble and Gazebo Classic 11.10

Package installs:
1. `sudo apt install ros-humble-joint-state-publisher`
2. `sudo apt install ros-humble-joint-state-publisher-gui`
3. `sudo apt install python3-roslaunch`
4. `sudo apt install ros-humble-xacro`

Build Workspace inside `ros2_ws`
1. `colcon build`
2. `. install/setup.bash`

Run these commands to ensure the Gazebo world loads
1. `. /usr/share/gazebo/setup.sh`
2. `export GAZEBO_PLUGIN_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/lib:${GAZEBO_PLUGIN_PATH}`
3. `export GAZEBO_MODEL_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/models:${GAZEBO_MODEL_PATH}`
4. `export GAZEBO_RESOURCE_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/world:${GAZEBO_RESOURCE_PATH}`


To fire up the gazebo sim type the following from inside the ROS workspace
1. `roscose`
2. `ros2 launch robot_description display.launch.py`


To fire up gazebo nav2 test
1. cd ~/rbe594_capstone_prime/ros2_ws
2. . install/setup.bash
3. ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

### To run the code in src/warehouse_robot and launch the robot with nav integration
Run these commands in their own terminals to get the whole system up and running. We can probably put all this into a single launch file.
Run these from the ros2_ws directory after build & install. You should also have GAZEBO_MODEL_PATH include the models directory so that
the gazebo map loads correctly

ros2 launch warehouse_robot launch_sim.launch.py world:=./src/warehouse_robot/worlds/warehouse.world use_sim_time:=true
ros2 launch nav2_bringup localization_launch.py map:=./src/warehouse_robot/maps/empty_warehouse2.yaml use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
rviz2 -d ./src/warehouse_robot/config/main.rviz

Install required packages using `install_packages.bash`