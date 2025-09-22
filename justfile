set working-directory := '../..'

alias b := build
alias re:=rebuild

livox_ignore := "./src/livox_ros_driver2/COLCON_IGNORE"
livox_build := "./src/livox_ros_driver2/build.sh"

default:
  just --list

install:
  #!/usr/bin/env bash
  set -euxo pipefail
  git clone https://github.com/Livox-SDK/Livox-SDK2.git
  cd ./Livox-SDK2/
  mkdir build
  cd build
  cmake .. && make -j
  sudo make install
  cd ..
  touch COLCON_IGNORE
  cd ../src
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ros_driver2
  rm livox_ros_driver2/build.sh
  cp trafctec_ros/build_livox_ros_driver2.sh livox_ros_driver2/build.sh
  cd livox_ros_driver2
  ./build.sh humble
  touch COLCON_IGNORE
  cd ..
  git clone https://github.com/LihanChen2004/livox_laser_simulation_ros2.git
  cd ..
  wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
  sudo apt update
  sudo apt install -q -y --no-install-recommends \
      ros-humble-ros-gz \
      ros-humble-gazebo-ros \
      ros-humble-gazebo-ros-pkgs
  rosdep init
  rosdep update
  rosdep install --from-paths src -i

rebuild:
  rm -rf log build install
  if [ -f {{livox_ignore}} ]; then \
    rm {{livox_ignore}}; \
  fi
  {{livox_build}} humble
  touch {{livox_ignore}}

build:
  colcon build --symlink-install

display:
  ros2 launch trafctec_description display.launch.py


simulate:
  ros2 launch trafctec_simulation gazebo.launch.py

only-map:
  ros2 launch trafctec_navigation rtabmap_map_sim_lidar.launch.py

map args='use_sim_time:=true':
  @echo "Use 'just navigate use_sim_time=true' when using the real robot."
  ros2 launch trafctec_navigation bringup.launch.py localization:=false {{args}}

map-real: (map "use_sim_time:=false")

save-map:
  ros2 run nav2_map_server map_saver_cli

locate:
  ros2 launch trafctec_navigation rtabmap_loc_sim_lidar.launch.py

navigate args='use_sim_time:=true':
  @echo "Use 'just navigate use_sim_time=true' when using the real robot."
  ros2 launch trafctec_navigation bringup.launch.py {{args}}

navigate-real: (navigate "use_sim_time:=false output_topic:=\\cmd_vel")

mid:
  ros2 launch trafctec_bringup MID360.launch.py use_sim_time:=false

teleop:
  ros2 run trafctec_bringup teleop_twist_keyboard.sh

view-frames:
  cd ./log && ros2 run tf2_tools view_frames

sync:
    @bash ./src/trafctec_ros/sync_justfile.sh
