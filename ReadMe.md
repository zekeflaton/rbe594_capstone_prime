Source code goes in "src"
Any code we want to run should be a script in "scripts"
Any output should be saved to "results"
Package requirements are in the requirements.txt

---

## ROS 2 WS
ROS Humble and Gazebo Classic 11.10

Build Workspace inside `ros2_ws`
1. `colcon build`
2. `. install/setup.bash`

Run these commands to ensure the Gazebo world loads
1. `. /usr/share/gazebo/setup.sh`
2. `export GAZEBO_PLUGIN_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/lib:${GAZEBO_PLUGIN_PATH}`
3. `export GAZEBO_MODEL_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/models:${GAZEBO_MODEL_PATH}`
4. `export GAZEBO_RESOURCE_PATH=~/<path to repo>/rbe594_capstone_prime/ros2_ws/install/robot_description/share/robot_description/world:${GAZEBO_RESOURCE_PATH}`
