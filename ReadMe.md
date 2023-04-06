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
1. `colcon build --symlink-install`
2. `. install/setup.bash`

Run these commands to ensure the Gazebo world loads
1. `. /usr/share/gazebo/setup.sh`
2. `export GAZEBO_PLUGIN_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/lib:${GAZEBO_PLUGIN_PATH}`
3. `export GAZEBO_MODEL_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/models:${GAZEBO_MODEL_PATH}`
4. `export GAZEBO_RESOURCE_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/world:${GAZEBO_RESOURCE_PATH}`


To fire up the gazebo sim type the following from inside the ROS workspace
1. `ros2 launch robot_description display.launch.py`


To fire up gazebo nav2 test
1. `cd ~/rbe594_capstone_prime/ros2_ws`
2. `. install/setup.bash`
3. `ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`
