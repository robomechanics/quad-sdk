PREFIX="$HOME/ros2_ws/install/gazebo_scripts/share/gazebo_scripts"
IGN_GAZEBO_RESOURCE_PATH_UPDATE="export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}::${PREFIX}/spirit_description:${PREFIX}/other/sensor_description:${PREFIX}/other/objects_description:${PREFIX}/gazebo_scripts/worlds:${PREFIX}/gazebo_scripts/models"

if grep -Fxq "$IGN_GAZEBO_RESOURCE_PATH_UPDATE" ~/.bashrc > /dev/null
then
   echo "bashrc contains gazebo model path update"
else
   echo "Adding gazebo model path update to bashrc"
   echo "$IGN_GAZEBO_RESOURCE_PATH_UPDATE" >> ~/.bashrc
fi

sudo apt-get update
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
sudo apt install -y mesa-utils\
      gz-tools

sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-controller-manager \
    ros-humble-effort-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-imu-tools \
    ros-humble-ros-gz \

#Notes:
#ros-humble-gazebo-ros-pkgs Integration with Gazebo Classic
#ros-humble-ros-gz Integration with Ignition Gazebo

# sudo apt install -y ros-melodic-controller-manager
# sudo apt install -y ros-melodic-joint-state-controller
# sudo apt install -y ros-melodic-gazebo-ros-pkgs
# sudo apt install -y ros-melodic-ros-control
# sudo apt install -y ros-melodic-gazebo-ros-control
# sudo apt install -y ros-melodic-effort-controllers
# sudo apt install -y ros-melodic-robot-state-publisher
# sudo apt install -y ros-melodic-imu-tools
# sudo apt install -y ros-melodic-message-to-tf