/*
MIT License (modified)

Copyright (c) 2018 The Trustees of the University of Pennsylvania
Authors:
Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin() {
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin() {
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Initialize ROS if necessary
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "ContactPublisher",ros::init_options::NoSigintHandler);
  }

  // Initialize node
  this->rosNode.reset(new ros::NodeHandle("ContactPublisher"));

  // Initialize Publishers
  this->toe0_publisher = this->rosNode->advertise<geometry_msgs::Vector3>("/gazebo/toe0_forces", 1, true);
  this->toe1_publisher = this->rosNode->advertise<geometry_msgs::Vector3>("/gazebo/toe1_forces", 1, true);
  this->toe2_publisher = this->rosNode->advertise<geometry_msgs::Vector3>("/gazebo/toe2_forces", 1, true);
  this->toe3_publisher = this->rosNode->advertise<geometry_msgs::Vector3>("/gazebo/toe3_forces", 1, true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate() {
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
    std::string str = contacts.contact(i).collision1();
    std::string toe0_string = "toe0_collision";
    std::string toe1_string = "toe1_collision";
    std::string toe2_string = "toe2_collision";
    std::string toe3_string = "toe3_collision";
    std::size_t found_toe0 = str.find(toe0_string);
    std::size_t found_toe1 = str.find(toe1_string);
    std::size_t found_toe2 = str.find(toe2_string);
    std::size_t found_toe3 = str.find(toe3_string);

    if (found_toe0 != std::string::npos) {
      geometry_msgs::Vector3 toe0_msg; toe0_msg.x = 0.; toe0_msg.y = 0.; toe0_msg.z = 0.;
      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
        toe0_msg.x -= contacts.contact(i).wrench(j).body_1_wrench().force().x();
        toe0_msg.y -= contacts.contact(i).wrench(j).body_1_wrench().force().y();
        toe0_msg.z -= contacts.contact(i).wrench(j).body_1_wrench().force().z();
      }
      this->toe0_publisher.publish(toe0_msg);
    }

    if (found_toe1 != std::string::npos) {
      geometry_msgs::Vector3 toe1_msg; toe1_msg.x = 0.; toe1_msg.y = 0.; toe1_msg.z = 0.;
      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
        toe1_msg.x -= contacts.contact(i).wrench(j).body_1_wrench().force().x();
        toe1_msg.y -= contacts.contact(i).wrench(j).body_1_wrench().force().y();
        toe1_msg.z -= contacts.contact(i).wrench(j).body_1_wrench().force().z();
      }
      this->toe1_publisher.publish(toe1_msg);
    }

    if (found_toe2 != std::string::npos) {
      geometry_msgs::Vector3 toe2_msg; toe2_msg.x = 0.; toe2_msg.y = 0.; toe2_msg.z = 0.;
      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
        toe2_msg.x -= contacts.contact(i).wrench(j).body_1_wrench().force().x();
        toe2_msg.y -= contacts.contact(i).wrench(j).body_1_wrench().force().y();
        toe2_msg.z -= contacts.contact(i).wrench(j).body_1_wrench().force().z();
      }
      this->toe2_publisher.publish(toe2_msg);
    }

    if (found_toe3 != std::string::npos) {
      geometry_msgs::Vector3 toe3_msg; toe3_msg.x = 0.; toe3_msg.y = 0.; toe3_msg.z = 0.;
      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j) {
        toe3_msg.x -= contacts.contact(i).wrench(j).body_1_wrench().force().x();
        toe3_msg.y -= contacts.contact(i).wrench(j).body_1_wrench().force().y();
        toe3_msg.z -= contacts.contact(i).wrench(j).body_1_wrench().force().z();
      }
      this->toe3_publisher.publish(toe3_msg);
    }
  }
}
