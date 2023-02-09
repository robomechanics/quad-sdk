/*
 * Copyright (C) 2023 Vincent FOUCAULT
 * Author: Vincent FOUCAULT
 * email:  elpimous12@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/
#include #include "robot_driver/hardware_interfaces/imu_interface.h"

ImuInterface::ImuInterface(){}

ImuInterface::~ImuInterface(){}

void ImuInterface::initializeImuInterface(const std::string& imu_link_name)
{
    imu_orientation_.resize(4);
    imu_ang_vel_.resize(3);
    imu_lin_acc_.resize(3);

    imu_data_.name = "imu";
    imu_data_.frame_id = imu_link_name;
    imu_data_.orientation = &imu_orientation_[0];
    imu_data_.angular_velocity = &imu_ang_vel_[0];
    imu_data_.linear_acceleration = &imu_lin_acc_[0];
    imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imu_data_));
}