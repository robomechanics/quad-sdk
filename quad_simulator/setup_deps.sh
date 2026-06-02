GZ_SIM_RESOURCE_PATH_UPDATE="export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:\
$HOME/ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts/models:\
$HOME/ros2_ws/install/quad_sim_scripts/share/quad_sim_scripts/worlds:\
$HOME/ros2_ws/install/spirit_description/share/spirit_description/models:\
$HOME/ros2_ws/install/a1_description/share/a1_description/models:\
$HOME/ros2_ws/install/go1_description/share/go1_description/models:\
$HOME/ros2_ws/install/go2_description/share/go2_description/models:\
$HOME/ros2_ws/install/go2w_description/share/go2w_description/models:\
$HOME/ros2_ws/install/spot_description/share/spot_description/models:\
$HOME/ros2_ws/install/vision60_description/share/vision60_description/models:\
$HOME/ros2_ws/install/sensor_description/share/sensor_description/models:\
$HOME/ros2_ws/install/objects_description/share/objects_description/models:\
$HOME/ros2_ws/install/underbrush_description/share/underbrush_description/models:\
$HOME/ros2_ws/install/b2_description/share/b2_description/models"

GZ_SIM_SYSTEM_PLUGIN_PATH_UPDATE="export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/jazzy/lib:$HOME/ros2_ws/install/gazebo_plugins/lib"

QUAD_LOGGER_SRC ="export QUAD_LOGGER_SRC=$HOME/ros2_ws/src/quad-sdk/quad_logger/bags"

LD_LIBRARY_PATH_UPDATE="export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH"

if ! grep -Fxq "$GZ_SIM_RESOURCE_PATH_UPDATE" ~/.bashrc; then
   echo "Adding GZ_SIM_RESOURCE_PATH update to bashrc"
   echo "$GZ_SIM_RESOURCE_PATH_UPDATE" >> ~/.bashrc
else
   echo "bashrc already contains GZ_SIM_RESOURCE_PATH update"
fi

if ! grep -Fxq "$GZ_SIM_SYSTEM_PLUGIN_PATH_UPDATE" ~/.bashrc; then
   echo "Adding GZ_SIM_SYSTEM_PLUGIN_PATH update to bashrc"
   echo "$GZ_SIM_SYSTEM_PLUGIN_PATH_UPDATE" >> ~/.bashrc
else
   echo "bashrc already contains GZ_SIM_SYSTEM_PLUGIN_PATH update"
fi

if ! grep -Fxq "$LD_LIBRARY_PATH_UPDATE" ~/.bashrc; then
   echo "Adding LD_LIBRARY_PATH update to bashrc"
   echo "$LD_LIBRARY_PATH_UPDATE" >> ~/.bashrc
else
   echo "bashrc already contains LD_LIBRARY_PATH update"
fi

sudo apt-get update
sudo apt-get install -y lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic
sudo apt install -y mesa-utils\
      ros-jazzy-gz-tools-vendor\
      ros-jazzy-gz-sim-vendor

sudo apt install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-effort-controllers \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-imu-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-mujoco-vendor \
    ros-jazzy-mujoco-ros2-control
