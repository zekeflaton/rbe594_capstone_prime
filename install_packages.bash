# install a crap-ton of packages, I just dumped these from my working
# environment using apt list | grep ros-humble

apt-get update && apt-get -y install ros-humble-ackermann-msgs \
ros-humble-action-msgs \
ros-humble-action-tutorials-cpp \
ros-humble-action-tutorials-interfaces \
ros-humble-action-tutorials-py \
ros-humble-actionlib-msgs \
ros-humble-admittance-controller \
ros-humble-ament-cmake-auto \
ros-humble-ament-cmake-copyright \
ros-humble-ament-cmake-core \
ros-humble-ament-cmake-cppcheck \
ros-humble-ament-cmake-cpplint \
ros-humble-ament-cmake-export-definitions \
ros-humble-ament-cmake-export-dependencies \
ros-humble-ament-cmake-export-include-directories \
ros-humble-ament-cmake-export-interfaces \
ros-humble-ament-cmake-export-libraries \
ros-humble-ament-cmake-export-link-flags \
ros-humble-ament-cmake-export-targets \
ros-humble-ament-cmake-flake8 \
ros-humble-ament-cmake-gen-version-h \
ros-humble-ament-cmake-gmock \
ros-humble-ament-cmake-gtest \
ros-humble-ament-cmake-include-directories \
ros-humble-ament-cmake-libraries \
ros-humble-ament-cmake-lint-cmake \
ros-humble-ament-cmake-pep257 \
ros-humble-ament-cmake-pytest \
ros-humble-ament-cmake-python \
ros-humble-ament-cmake-ros \
ros-humble-ament-cmake-target-dependencies \
ros-humble-ament-cmake-test \
ros-humble-ament-cmake-uncrustify \
ros-humble-ament-cmake-version \
ros-humble-ament-cmake-xmllint \
ros-humble-ament-cmake \
ros-humble-ament-copyright \
ros-humble-ament-cppcheck \
ros-humble-ament-cpplint \
ros-humble-ament-flake8 \
ros-humble-ament-index-cpp \
ros-humble-ament-index-python \
ros-humble-ament-lint-auto \
ros-humble-ament-lint-cmake \
ros-humble-ament-lint-common \
ros-humble-ament-lint \
ros-humble-ament-package \
ros-humble-ament-pep257 \
ros-humble-ament-uncrustify \
ros-humble-ament-xmllint \
ros-humble-angles \
ros-humble-backward-ros \
ros-humble-behaviortree-cpp-v3 \
ros-humble-bond \
ros-humble-bondcpp \
ros-humble-builtin-interfaces \
ros-humble-camera-calibration-parsers \
ros-humble-camera-info-manager \
ros-humble-cartographer-ros-msgs \
ros-humble-cartographer-ros \
ros-humble-cartographer \
ros-humble-class-loader \
ros-humble-common-interfaces \
ros-humble-composition-interfaces \
ros-humble-composition \
ros-humble-console-bridge-vendor \
ros-humble-control-msgs \
ros-humble-control-toolbox \
ros-humble-controller-interface \
ros-humble-controller-manager-msgs \
ros-humble-controller-manager \
ros-humble-costmap-queue \
ros-humble-cv-bridge \
ros-humble-demo-nodes-cpp-native \
ros-humble-demo-nodes-cpp \
ros-humble-demo-nodes-py \
ros-humble-depthimage-to-laserscan \
ros-humble-desktop \
ros-humble-diagnostic-msgs \
ros-humble-diagnostic-updater \
ros-humble-diff-drive-controller \
ros-humble-domain-coordinator \
ros-humble-dummy-map-server \
ros-humble-dummy-robot-bringup \
ros-humble-dummy-sensors \
ros-humble-dwb-core \
ros-humble-dwb-critics \
ros-humble-dwb-msgs \
ros-humble-dwb-plugins \
ros-humble-dynamixel-sdk \
ros-humble-effort-controllers \
ros-humble-eigen3-cmake-module \
ros-humble-example-interfaces \
ros-humble-examples-rclcpp-minimal-action-client \
ros-humble-examples-rclcpp-minimal-action-server \
ros-humble-examples-rclcpp-minimal-client \
ros-humble-examples-rclcpp-minimal-composition \
ros-humble-examples-rclcpp-minimal-publisher \
ros-humble-examples-rclcpp-minimal-service \
ros-humble-examples-rclcpp-minimal-subscriber \
ros-humble-examples-rclcpp-minimal-timer \
ros-humble-examples-rclcpp-multithreaded-executor \
ros-humble-examples-rclpy-executors \
ros-humble-examples-rclpy-minimal-action-client \
ros-humble-examples-rclpy-minimal-action-server \
ros-humble-examples-rclpy-minimal-client \
ros-humble-examples-rclpy-minimal-publisher \
ros-humble-examples-rclpy-minimal-service \
ros-humble-examples-rclpy-minimal-subscriber \
ros-humble-fastcdr \
ros-humble-fastrtps-cmake-module \
ros-humble-fastrtps \
ros-humble-filters \
ros-humble-foonathan-memory-vendor \
ros-humble-force-torque-sensor-broadcaster \
ros-humble-forward-command-controller \
ros-humble-gazebo-dev \
ros-humble-gazebo-msgs \
ros-humble-gazebo-plugins \
ros-humble-gazebo-ros-pkgs \
ros-humble-gazebo-ros2-control \
ros-humble-gazebo-ros \
ros-humble-generate-parameter-library-py \
ros-humble-generate-parameter-library \
ros-humble-geometry-msgs \
ros-humble-geometry2 \
ros-humble-gmock-vendor \
ros-humble-gtest-vendor \
ros-humble-hardware-interface \
ros-humble-hls-lfcd-lds-driver \
ros-humble-ignition-cmake2-vendor \
ros-humble-ignition-math6-vendor \
ros-humble-image-geometry \
ros-humble-image-tools \
ros-humble-image-transport \
ros-humble-imu-sensor-broadcaster \
ros-humble-interactive-markers \
ros-humble-intra-process-demo \
ros-humble-joint-limits \
ros-humble-joint-state-broadcaster \
ros-humble-joint-state-publisher \
ros-humble-joint-trajectory-controller \
ros-humble-joy \
ros-humble-kdl-parser \
ros-humble-keyboard-handler \
ros-humble-kinematics-interface \
ros-humble-laser-geometry \
ros-humble-launch-ros \
ros-humble-launch-testing-ament-cmake \
ros-humble-launch-testing-ros \
ros-humble-launch-testing \
ros-humble-launch-xml \
ros-humble-launch-yaml \
ros-humble-launch \
ros-humble-libcurl-vendor \
ros-humble-libstatistics-collector \
ros-humble-libyaml-vendor \
ros-humble-lifecycle-msgs \
ros-humble-lifecycle \
ros-humble-logging-demo \
ros-humble-map-msgs \
ros-humble-message-filters \
ros-humble-nav-2d-msgs \
ros-humble-nav-2d-utils \
ros-humble-nav-msgs \
ros-humble-nav2-amcl \
ros-humble-nav2-behavior-tree \
ros-humble-nav2-behaviors \
ros-humble-nav2-bringup \
ros-humble-nav2-bt-navigator \
ros-humble-nav2-collision-monitor \
ros-humble-nav2-common \
ros-humble-nav2-constrained-smoother \
ros-humble-nav2-controller \
ros-humble-nav2-core \
ros-humble-nav2-costmap-2d \
ros-humble-nav2-dwb-controller \
ros-humble-nav2-lifecycle-manager \
ros-humble-nav2-map-server \
ros-humble-nav2-msgs \
ros-humble-nav2-navfn-planner \
ros-humble-nav2-planner \
ros-humble-nav2-regulated-pure-pursuit-controller \
ros-humble-nav2-rotation-shim-controller \
ros-humble-nav2-rviz-plugins \
ros-humble-nav2-simple-commander \
ros-humble-nav2-smac-planner \
ros-humble-nav2-smoother \
ros-humble-nav2-theta-star-planner \
ros-humble-nav2-util \
ros-humble-nav2-velocity-smoother \
ros-humble-nav2-voxel-grid \
ros-humble-nav2-waypoint-follower \
ros-humble-navigation2 \
ros-humble-ompl \
ros-humble-orocos-kdl-vendor \
ros-humble-osrf-pycommon \
ros-humble-parameter-traits \
ros-humble-pcl-conversions \
ros-humble-pcl-msgs \
ros-humble-pendulum-control \
ros-humble-pendulum-msgs \
ros-humble-pluginlib \
ros-humble-position-controllers \
ros-humble-pybind11-vendor \
ros-humble-python-cmake-module \
ros-humble-python-qt-binding \
ros-humble-qt-dotgraph \
ros-humble-qt-gui-cpp \
ros-humble-qt-gui-py-common \
ros-humble-qt-gui \
ros-humble-quality-of-service-demo-cpp \
ros-humble-quality-of-service-demo-py \
ros-humble-rcl-action \
ros-humble-rcl-interfaces \
ros-humble-rcl-lifecycle \
ros-humble-rcl-logging-interface \
ros-humble-rcl-logging-spdlog \
ros-humble-rcl-yaml-param-parser \
ros-humble-rcl \
ros-humble-rclcpp-action \
ros-humble-rclcpp-components \
ros-humble-rclcpp-lifecycle \
ros-humble-rclcpp \
ros-humble-rclpy \
ros-humble-rcpputils \
ros-humble-rcutils \
ros-humble-realtime-tools \
ros-humble-resource-retriever \
ros-humble-rmw-dds-common \
ros-humble-rmw-fastrtps-cpp \
ros-humble-rmw-fastrtps-shared-cpp \
ros-humble-rmw-implementation-cmake \
ros-humble-rmw-implementation \
ros-humble-rmw \
ros-humble-robot-state-publisher \
ros-humble-ros-base \
ros-humble-ros-core \
ros-humble-ros-environment \
ros-humble-ros-workspace \
ros-humble-ros2-control-test-assets \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-ros2action \
ros-humble-ros2bag \
ros-humble-ros2cli-common-extensions \
ros-humble-ros2cli \
ros-humble-ros2component \
ros-humble-ros2controlcli \
ros-humble-ros2doctor \
ros-humble-ros2interface \
ros-humble-ros2launch \
ros-humble-ros2lifecycle \
ros-humble-ros2multicast \
ros-humble-ros2node \
ros-humble-ros2param \
ros-humble-ros2pkg \
ros-humble-ros2run \
ros-humble-ros2service \
ros-humble-ros2topic \
ros-humble-rosbag2-compression-zstd \
ros-humble-rosbag2-compression \
ros-humble-rosbag2-cpp \
ros-humble-rosbag2-interfaces \
ros-humble-rosbag2-py \
ros-humble-rosbag2-storage-default-plugins \
ros-humble-rosbag2-storage \
ros-humble-rosbag2-transport \
ros-humble-rosbag2 \
ros-humble-rosgraph-msgs \
ros-humble-rosidl-adapter \
ros-humble-rosidl-cli \
ros-humble-rosidl-cmake \
ros-humble-rosidl-default-generators \
ros-humble-rosidl-default-runtime \
ros-humble-rosidl-generator-c \
ros-humble-rosidl-generator-cpp \
ros-humble-rosidl-generator-py \
ros-humble-rosidl-parser \
ros-humble-rosidl-runtime-c \
ros-humble-rosidl-runtime-cpp \
ros-humble-rosidl-runtime-py \
ros-humble-rosidl-typesupport-c \
ros-humble-rosidl-typesupport-cpp \
ros-humble-rosidl-typesupport-fastrtps-c \
ros-humble-rosidl-typesupport-fastrtps-cpp \
ros-humble-rosidl-typesupport-interface \
ros-humble-rosidl-typesupport-introspection-c \
ros-humble-rosidl-typesupport-introspection-cpp \
ros-humble-rpyutils \
ros-humble-rqt-action \
ros-humble-rqt-bag-plugins \
ros-humble-rqt-bag \
ros-humble-rqt-common-plugins \
ros-humble-rqt-console \
ros-humble-rqt-graph \
ros-humble-rqt-gui-cpp \
ros-humble-rqt-gui-py \
ros-humble-rqt-gui \
ros-humble-rqt-image-view \
ros-humble-rqt-msg \
ros-humble-rqt-plot \
ros-humble-rqt-publisher \
ros-humble-rqt-py-common \
ros-humble-rqt-py-console \
ros-humble-rqt-reconfigure \
ros-humble-rqt-service-caller \
ros-humble-rqt-shell \
ros-humble-rqt-srv \
ros-humble-rqt-topic \
ros-humble-rsl \
ros-humble-rttest \
ros-humble-rviz-assimp-vendor \
ros-humble-rviz-common \
ros-humble-rviz-default-plugins \
ros-humble-rviz-ogre-vendor \
ros-humble-rviz-rendering \
ros-humble-rviz2 \
ros-humble-sdl2-vendor \
ros-humble-sensor-msgs-py \
ros-humble-sensor-msgs \
ros-humble-shape-msgs \
ros-humble-shared-queues-vendor \
ros-humble-slam-toolbox \
ros-humble-smclib \
ros-humble-spdlog-vendor \
ros-humble-sqlite3-vendor \
ros-humble-sros2-cmake \
ros-humble-sros2 \
ros-humble-statistics-msgs \
ros-humble-std-msgs \
ros-humble-std-srvs \
ros-humble-stereo-msgs \
ros-humble-tango-icons-vendor \
ros-humble-tcb-span \
ros-humble-teleop-twist-joy \
ros-humble-teleop-twist-keyboard \
ros-humble-tf2-bullet \
ros-humble-tf2-eigen-kdl \
ros-humble-tf2-eigen \
ros-humble-tf2-geometry-msgs \
ros-humble-tf2-kdl \
ros-humble-tf2-msgs \
ros-humble-tf2-py \
ros-humble-tf2-ros-py \
ros-humble-tf2-ros \
ros-humble-tf2-sensor-msgs \
ros-humble-tf2-tools \
ros-humble-tf2 \
ros-humble-tinyxml-vendor \
ros-humble-tinyxml2-vendor \
ros-humble-tl-expected \
ros-humble-tlsf-cpp \
ros-humble-tlsf \
ros-humble-topic-monitor \
ros-humble-tracetools \
ros-humble-trajectory-msgs \
ros-humble-transmission-interface \
ros-humble-tricycle-controller \
ros-humble-turtlebot3-bringup \
ros-humble-turtlebot3-cartographer \
ros-humble-turtlebot3-description \
ros-humble-turtlebot3-example \
ros-humble-turtlebot3-fake-node-dbgsym \
ros-humble-turtlebot3-fake-node \
ros-humble-turtlebot3-gazebo-dbgsym \
ros-humble-turtlebot3-gazebo \
ros-humble-turtlebot3-msgs-dbgsym \
ros-humble-turtlebot3-msgs \
ros-humble-turtlebot3-navigation2 \
ros-humble-turtlebot3-node-dbgsym \
ros-humble-turtlebot3-node \
ros-humble-turtlebot3-simulations \
ros-humble-turtlebot3-teleop \
ros-humble-turtlebot3 \
ros-humble-turtlesim \
ros-humble-twist-mux \
ros-humble-uncrustify-vendor \
ros-humble-unique-identifier-msgs \
ros-humble-urdf-parser-plugin \
ros-humble-urdf \
ros-humble-urdfdom-headers \
ros-humble-urdfdom \
ros-humble-velocity-controllers \
ros-humble-visualization-msgs \
ros-humble-xacro \
ros-humble-yaml-cpp-vendor \
ros-humble-zstd-vendor \
ros-humble-apriltag-ros \
python3-pip

# for good measure, install python dependencies
python3 -m pip install -r requirements.txt