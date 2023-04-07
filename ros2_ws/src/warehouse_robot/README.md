## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

## Launch Commands
Run these commands in their own terminals to get the whole system up and running. We can probably put all this into a single launch file.
Run these from the ros2_ws directory after build & install. You should also have GAZEBO_RESOURCE_PATH include the models directory so that
the gazebo map loads correctly

ros2 launch warehouse_robot launch_sim.launch.py world:=./src/warehouse_robot/worlds/warehouse.world use_sim_time:=true
ros2 launch nav2_bringup localization_launch.py map:=./src/warehouse_robot/maps/empty_warehouse2.yaml use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
rviz2 -d ./src/warehouse/config/main.rviz