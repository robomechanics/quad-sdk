GAZEBO_MODEL_PATH_UPDATE="export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/spirit_description:$(pwd)/other/sensor_description:$(pwd)/other/objects_description:$(pwd)/gazebo_scripts/worlds"

if grep "$GAZEBO_MODEL_PATH_UPDATE" ~/.bashrc > /dev/null
then
   echo "bashrc contains gazebo model path update"
else
   echo "Adding gazebo model path update to bashrc"
   echo "$GAZEBO_MODEL_PATH_UPDATE" >> ~/.bashrc
fi

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y gazebo9 -y



sudo apt install -y ros-noetic-controller-manager -y
sudo apt install -y ros-noetic-joint-state-controller -y
sudo apt install -y ros-noetic-gazebo-ros-pkgs -y
sudo apt install -y ros-noetic-ros-control -y
sudo apt install -y ros-noetic-gazebo-ros-control -y
sudo apt install -y ros-noetic-effort-controllers -y
sudo apt install -y ros-noetic-robot-state-publisher -y
sudo apt install -y ros-noetic-imu-tools -y
sudo apt install -y ros-noetic-message-to-tf -y