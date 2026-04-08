FROM ros:humble-ros-base 

RUN apt-get update && apt-get install -y \
  # Install libfranka build depencencies
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libfmt-dev \
  libpoco-dev \
  ros-humble-pinocchio \
  # Install ros packages
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-pip \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# Install robotiq gripper
ARG ROS2_ROBOTIQ_GRIPPER_COMMIT_HASH="2ff85455d4b9f973c4b0bab1ce95fb09367f0d26"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  mkdir tmp_build && \
  cd tmp_build && \
  git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
  cd ros2_robotiq_gripper && git checkout ${ROS2_ROBOTIQ_GRIPPER_COMMIT_HASH} && cd .. && \
  sed -i 's/kGripperMaxSpeed = 0.150;/kGripperMaxSpeed = 1.0;/g' ros2_robotiq_gripper/robotiq_driver/src/hardware_interface.cpp && \
  vcs import . --input ros2_robotiq_gripper/ros2_robotiq_gripper.humble.repos && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  colcon build --install-base /opt/ros/humble/franka --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  cd .. && \
  rm -rf tmp_build && \
  rm -rf /var/lib/apt/lists/*"

# Install rqt reconfigure from source (for array parameters)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  mkdir -p /tmp/rqt_reconfigure && cd /tmp && \
  git clone https://github.com/ros-visualization/rqt_reconfigure.git --branch humble && \
  cd rqt_reconfigure && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  colcon build --install-base /opt/ros/humble/franka --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  cd .. && \
  rm -rf /tmp/rqt_reconfigure && \
  rm -rf /var/lib/apt/lists/*"

# Install libfranka, franka_ros2 and franka_description
# Note: The versions need to be set according to your robot's system version
# https://frankarobotics.github.io/docs/compatibility.html
ARG LIBFRANKA_VERSION=0.18.2
ARG FRANKA_ROS2_VERSION=v2.1.0
ARG FRANKA_DESCRIPTION_VERSION=1.3.0

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && \
mkdir -p /tmp/franka_ros2 && cd /tmp/franka_ros2 && \
  git clone --recursive https://github.com/frankarobotics/libfranka --branch ${LIBFRANKA_VERSION} && \
  git clone --recursive https://github.com/frankarobotics/franka_ros2.git --branch ${FRANKA_ROS2_VERSION} && \
  git clone --recursive https://github.com/frankarobotics/franka_description.git --branch ${FRANKA_DESCRIPTION_VERSION} && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths . --ignore-src -r -y && \
  rm -rf /var/lib/apt/lists/* && \
  colcon build --install-base /opt/ros/humble/franka --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=off  && \
  cd .. && \ 
  rm -rf /tmp/franka_ros2 && \
  echo "export PS1='\[\e[1;31m\][gello]\[\e[0m\] \[\e[1;32m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> ~/.bashrc && \
  echo "export LS_COLORS='di=1;97;42:ow=1;97;42:tw=1;97;42'" >> ~/.bashrc && \
  echo "" >> ~/.bashrc && \
  echo "export ROS_DOMAIN_ID=94" >> ~/.bashrc && \
  echo "source /opt/ros/humble/franka/setup.bash" >> ~/.bashrc && \
  echo "source /workspace/ros2/install/setup.bash" >> ~/.bashrc && \
  echo "" >> ~/.bashrc && \
  echo "alias eb='nano ~/.bashrc'" >> ~/.bashrc && \
  echo "alias sb='source ~/.bashrc'" >> ~/.bashrc && \
  echo "alias cm='cd /workspace/ros2 && colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON'" >> ~/.bashrc && \
  echo "" >> ~/.bashrc && \
  echo "alias robot='ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py'" >> ~/.bashrc && \
  echo "alias gello='ros2 launch franka_gello_state_publisher main.launch.py'" >> ~/.bashrc && \
  echo "alias koras='ros2 launch franka_gripper_manager koras_gripper_client.launch.py'" >> ~/.bashrc && \
  echo "source /opt/ros/humble/franka/setup.bash" >> ~/.profile'
