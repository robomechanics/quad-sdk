sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
cat /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo9 -y

GAZEBO_MODEL_PATH_UPDATE='export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/spirit-software/spirit_simulator/spirit_description:~/catkin_ws/src/spirit-software/spirit_simulator/other/sensor_description:~/catkin_ws/src/spirit-software/spirit_simulator/other/objects_description'
if grep "$GAZEBO_MODEL_PATH_UPDATE" ~/.bashrc > /dev/null
then
   echo "bashrc contains gazebo model path update"
else
   echo "Adding gazebo model path update to bashrc"
   echo "$GAZEBO_MODEL_PATH_UPDATE" >> ~/.bashrc
fi

sudo apt install ros-melodic-controller-manager -y
sudo apt install ros-melodic-joint-state-controller -y
sudo apt install ros-melodic-gazebo-ros-pkgs -y
sudo apt install ros-melodic-ros-control -y
sudo apt install ros-melodic-gazebo-ros-control -y
sudo apt install ros-melodic-effort-controllers -y
sudo apt install ros-melodic-robot-state-publisher -y
sudo apt install ros-melodic-imu-tools -y
sudo apt install ros-melodic-message-to-tf -y