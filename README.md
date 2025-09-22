# Trafc Tec ROS 2

Navigation with in ROS 2. Tested with ROS Jazzy on Ubuntu 24.04 Jammy Jellyfish. This is a meta-package containing the following ROS 2 packages:

- `tracftec_bringup`: Top-level wrapper for other packages
- `tracftec_description`: 3D models and URDFs for the tracftec Bot, including simple launchs for visualization with rviz;
- `tracftec_simulation`: Simulation with Gazebo Classic;
- `tracftec_navigation`: Navigation stack including localization with [Carthographer](https://google-cartographer-ros.readthedocs.io/en/latest/) and path planinig and path tracking with [Nav2 stack](https://docs.nav2.org/);
- `tracftec_simulation`: Top-level package that integrates with all others with launchs in simulation and with the real robot. And can be also used with [Exwayz](https://exwayz.notion.site/Exwayz-Documentation-3bca99777b384ccc8d1734c9f3b646a3) for localization;

For more information about each package, take a look in the corresponding `README.md` file inside the package.

## Prerequisites

You'll need ROS 2 Humble installed on Ubuntu 22.04.

- Follow the official installation guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

For convenience, this guide also provides commands using [Just](https://github.com/casey/just), a handy command runner. To use the shorter commands, install it with:

```bash
sudo apt update && sudo apt install just
```

**Note**: Using `just`is optional. All commands are also provided in their raw shell format.

## Installation

First, create your workspace and clone the repository.

```bash
# Create and navigate to your workspace
mkdir -p ~/tracftec_ws/src
cd ~/tracftec_ws/src

# Clone this repository
git clone <your-repo-url> tracftec_ros
```

Now, follow either the Just or Manual instructions to install dependencies.

### **Option 1**: Installation with `just` (Recommended)

This is the easiest method. The command automates all dependency installations.
Install dependencies:

```bash
# Navigate to the repo directory
cd ~/tracftec_ws/src/tracftec_ros

# Run the automated installer
just install

```

### **\*Option 2**: Manual Installation

Run these commands from your workspace root (`~/tracftec_ws`).

```bash
# Install Livox SDK2
sudo apt install -y cmake
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
cd .. && touch COLCON_IGNORE
cd .. # Back to ~/tracftec_ws

# Install livox_ros_driver2 and simulation plugin
cd src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
rm livox_ros_driver2/build.sh
cp tracftec_ros/build_livox_ros_driver2.sh livox_ros_driver2/build.sh
cd livox_ros_driver2 && ./build.sh humble && touch COLCON_IGNORE && cd ..
git clone https://github.com/LihanChen2004/livox_laser_simulation_ros2.git
cd .. # Back to ~/tracftec_ws

# Install Gazebo
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-gz ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs

# Install all other ROS dependencies
sudo rosdep init 2>/dev/null || rosdep update
rosdep update
rosdep install --from-paths src -y --ignore-src
```

## Building & Sourcing

After installation, build and source the workspace.

- 1. **Build the code:**

  - with `just`

  ```bash
  # From the ~/tracftec_ws directory
  just b
  ```

  - manually

  ```bash
  # From the ~/tracftec_ws directory
  colcon build --symlink-install
  ```

- 2. Source the workspace:
     This step is the same for both methods.

```bash
. install/setup.bash
```

- 3. Setup auto source

Finally add this to your `~/.bashrc` or `~/.zshrc`

```bash
if [ -f /opt/ros/humble/setup.zsh ]; then
  . /opt/ros/humble/setup.zsh
  if [ -f /usr/share/gazebo/setup.sh ]; then
    . /usr/share/gazebo/setup.sh
  fi
  if [ -f /usr/share/gazebo-11/setup.sh ]; then
    . /usr/share/gazebo-11/setup.sh
  fi
```

## Usage

Here are the commands for common tasks. Each table shows the convenient just command and its corresponding manual ros2 command.

### Simulation

Open a new terminal for each command.
| Task | With `just` | Manual Command |
| :------------------------- | :-------------- | :------------------------------------------------------------------ |
| 1. Launch Gazebo | `just simulate` | `ros2 launch tracftec_simulation gazebo.launch.py` |
| 2. Launch Navigation (Sim) | `just navigate` | `ros2 launch tracftec_navigation bringup.launch.py use_sim_time:=true` |
| 3. Manual Control | `just teleop` | `ros2 run tracftec_bringup teleop_twist_keyboard.sh` |

### Mapping

Use these commands to create and save a map.

| Task                     | With `just`     | Manual Command                                                                              |
| :----------------------- | :-------------- | :------------------------------------------------------------------------------------------ |
| **Mapping (Simulation)** | `just map`      | `ros2 launch tracftec_navigation bringup.launch.py localization:=false use_sim_time:=true`  |
| **Mapping (Real Robot)** | `just map-real` | `ros2 launch tracftec_navigation bringup.launch.py localization:=false use_sim_time:=false` |
| **Save the Map**         | `just save-map` | `ros2 run nav2_map_server map_saver_cli`                                                    |

### Real Robot Navigation

Use these commands to operate the physical robot.

| Task                  | With `just`          | Manual Command                                                                                 |
| :-------------------- | :------------------- | :--------------------------------------------------------------------------------------------- |
| 1. Start LiDAR Driver | `just mid`           | `ros2 launch tracftec_bringup MID360.launch.py use_sim_time:=false`                            |
| 2. Launch Navigation  | `just navigate-real` | `ros2 launch tracftec_navigation bringup.launch.py use_sim_time:=false output_topic:=/cmd_vel` |

### Utilities

| Task            | With `just`        | Manual Command                                               |
| :-------------- | :----------------- | :----------------------------------------------------------- |
| Clean & Rebuild | `just rebuild`     | `rm -rf build install log && colcon build --symlink-install` |
| View TF Tree    | `just view-frames` | `ros2 run tf2_tools view_frames && evince frames.pdf`        |
