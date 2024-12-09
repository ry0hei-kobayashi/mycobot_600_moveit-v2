// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mycobot_control/mycobot_hardware.hpp"
#include <string>
#include <vector>
#include <iostream>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>

#include <vector>
#include <cmath>  // M_PI
#include <iostream>

#include <regex>
#include <rclcpp/rclcpp.hpp>

std::vector<double> rad2deg(const std::vector<double>& radians) {
    std::vector<double> degrees;
    degrees.reserve(radians.size()); 

    for (auto r : radians) {
        degrees.push_back(r * 180.0 / M_PI);
    }

    return degrees;
}

std::vector<double> deg2rad(const std::vector<double>& degrees) {
    std::vector<double> radians;
    radians.reserve(degrees.size()); 

    for (auto d : degrees) {
        radians.push_back(d * M_PI / 180.0);
    }

    return radians;
}

std::string createSetAnglesCommand(const std::vector<double>& angles, int speed) {
    std::ostringstream command;
    command << "set_angles(";
    for (size_t i = 0; i < angles.size(); ++i) {
        command << std::fixed << std::setprecision(3) << angles[i];
        if (i < angles.size() - 1) {
            command << ",";
        }
    }
    command << "," << std::fixed << std::setprecision(3) << speed << ")\n";
    return command.str();
}

std::string createSetSpeedCommand(int percentage) {
    std::ostringstream command;
    command << "set_speed(" << percentage << ")\n";
    return command.str();
}

namespace mycobot_control 
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  //socket connection
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(addrNum.c_str());
  connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));


  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  std::string angles = "get_angles()\n";
  const char* data = angles.c_str();
  char recvData[1024]; // 受信データ用バッファ
  
  send(sockfd, data, angles.size(), 0);
  memset(recvData, 0, sizeof(recvData)); // 受信バッファの初期化
  int recvLen = recv(sockfd, recvData, sizeof(recvData) - 1 , 0);
  if (recvLen > 0) {
      recvData[recvLen] = '\0'; // 文字列として終端を追加
      std::cout << recvData << std::endl;
  }

  //std2vector
  std::regex re(R"(\[([^\]]+)\])");
  std::smatch match;
  std::vector<double> deg_angles;
  
  std::string recvStr(recvData);
  if (std::regex_search(recvStr, match, re)) {
      // match[1]: "1.252415,-48.058783,1.271586,-1.669922,-97.207031,-10.195312"
      std::string numbers = match[1].str();
  
      // カンマで分割
      std::regex recomma(",");
      std::sregex_token_iterator it(numbers.begin(), numbers.end(), recomma, -1), end;
      for (; it != end; ++it) {
          deg_angles.push_back(std::stod(it->str()));
      }
  }

  //deg2rad
  std::vector<double> rad_vec = deg2rad(deg_angles);

  for (auto r : rad_vec) {
      std::cout << r << " ";
  }
  std::cout << std::endl;

  for (size_t i=0; i<joint_position_.size(); ++i){
      joint_position_[i] = rad_vec[i];
  } 

  for (auto & vel : joint_velocities_)
  {
    vel = 0.0;
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  //std::cout << joint_position_command_[0] << std::endl;
  // joint_velocities_command_;
  //rad2deg
  std::vector<double> rad_angles = rad2deg(joint_position_command_);

  //set gain
  int gainPercentage = 100; //percentage
  std::string gain_command = createSetSpeedCommand(gainPercentage);

  const char* gain_data = gain_command.c_str();
  char recvDataGain[1024]; // 受信データ用バッファ

  if (send(sockfd, gain_data, gain_command.size(), 0) < 0) {
      std::cerr << "Failed to send command." << std::endl;
      close(sockfd);
      return return_type::ERROR;
  }

  memset(recvDataGain, 0, sizeof(recvDataGain));
  int recvLenGain = recv(sockfd, recvDataGain, sizeof(recvDataGain) - 1, 0);
  if (recvLenGain > 0) {
      recvDataGain[recvLenGain] = '\0'; // 文字列として終端
      std::cout << "Response: " << recvDataGain << std::endl;
  } else {
      std::cerr << "Failed to receive response." << std::endl;
  }


  //set angle
  //std::vector<double> angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
  int speed = 1999; // 速度

  std::string command = createSetAnglesCommand(rad_angles, speed);
  const char* data = command.c_str();
  char recvData[1024]; // 受信データ用バッファ

  if (send(sockfd, data, command.size(), 0) < 0) {
      std::cerr << "Failed to send command." << std::endl;
      close(sockfd);
      return return_type::ERROR;
  }

  memset(recvData, 0, sizeof(recvData));
  int recvLen = recv(sockfd, recvData, sizeof(recvData) - 1, 0);
  if (recvLen > 0) {
      recvData[recvLen] = '\0'; // 文字列として終端
      std::cout << "Response: " << recvData << std::endl;
  } else {
      std::cerr << "Failed to receive response." << std::endl;
  }

  return return_type::OK;

}

}  

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mycobot_control::RobotSystem, hardware_interface::SystemInterface)
