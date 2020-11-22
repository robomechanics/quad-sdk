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

#include "contact_plugin.h"

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
  this->contact_publisher = this->rosNode->advertise<spirit_msgs::ContactMode>("/gazebo/toe_forces", 1, true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate() {
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  // Initialize outgoing messages with zeros (send zero contact force if no contact detected)
  spirit_msgs::ContactMode contact_msg;
  contact_msg.leg_contacts.resize(4);

  std::string toe_collision_names[4] = {"toe0_collision","toe1_collision", "toe2_collision","toe3_collision"};

  // Assume no contact
  for (unsigned int i = 0; i < 4; ++i)
  {
    contact_msg.leg_contacts.at(i).contact_prob = 0.0;
    contact_msg.leg_contacts.at(i).contact_state = false;
    contact_msg.leg_contacts.at(i).contact_forces.x = 0;
    contact_msg.leg_contacts.at(i).contact_forces.y = 0;
    contact_msg.leg_contacts.at(i).contact_forces.z = 0;
  }

  // Populate messages with contact forces
  for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
    std::string str = contacts.contact(i).collision1();
    std::cout << "Collision name: " << str << std::endl;
    for (unsigned int j = 0; j < 4; ++j)
    {
      std::string toe_string = toe_collision_names[j];

      
      std::size_t found_toe = str.find(toe_string);
      if (found_toe != std::string::npos)
      {
        contact_msg.leg_contacts.at(j).contact_prob = 1.0;
        contact_msg.leg_contacts.at(j).contact_state = true;
        for (unsigned int k = 0; k < contacts.contact(i).position_size(); ++k) {
          contact_msg.leg_contacts.at(j).contact_forces.x -= contacts.contact(i).wrench(k).body_1_wrench().force().x();
          contact_msg.leg_contacts.at(j).contact_forces.y -= contacts.contact(i).wrench(k).body_1_wrench().force().y();
          contact_msg.leg_contacts.at(j).contact_forces.z -= contacts.contact(i).wrench(k).body_1_wrench().force().z();
        }
      }
    }
  }

  this->contact_publisher.publish(contact_msg);
}
