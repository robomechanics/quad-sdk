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

#include <hardware_interface/imu_sensor_interface.h>

class ImuInterface

{
public:
    ImuInterface();
    virtual ~ImuInterface();

    // imu initialization
    void initializeImuInterface(const std::string& imu_link_name);

protected:

    hardware_interface::ImuSensorInterface imu_sensor_interface_;

    hardware_interface::ImuSensorHandle::Data imu_data_;
    std::vector<double> imu_orientation_;
    std::vector<double>	imu_ang_vel_;
    std::vector<double>	imu_lin_acc_;
};