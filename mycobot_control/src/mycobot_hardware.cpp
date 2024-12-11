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

#include <arpa/inet.h>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <netinet/in.h>
#include <regex>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace
{
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr int JOINT_COUNT = 6;
constexpr int SPEED_PERCENT = 100;
constexpr int SPEED_VALUE = 1999; 

// Helper functions
inline std::vector<double> rad2deg(const std::vector<double> &radians)
{
  std::vector<double> degrees;
  degrees.reserve(radians.size());
  for (auto r : radians)
  {
    degrees.push_back(r * RAD2DEG);
  }
  return degrees;
}

inline std::vector<double> deg2rad(const std::vector<double> &degrees)
{
  std::vector<double> radians;
  radians.reserve(degrees.size());
  for (auto d : degrees)
  {
    radians.push_back(d * DEG2RAD);
  }
  return radians;
}

inline std::string createSetAnglesCommand(const std::vector<double> &angles, int speed)
{
  std::ostringstream command;
  command << "set_angles(";
  for (size_t i = 0; i < angles.size(); ++i)
  {
    command << std::fixed << std::setprecision(3) << angles[i];
    if (i < angles.size() - 1)
    {
      command << ",";
    }
  }
  command << "," << std::fixed << std::setprecision(3) << speed << ")\n";
  return command.str();
}

inline std::string createSetSpeedCommand(int percentage)
{
  std::ostringstream command;
  command << "set_speed(" << percentage << ")\n";
  return command.str();
}

inline std::vector<double> parseAnglesFromResponse(const std::string &response)
{
  std::regex re(R"(\[([^\]]+)\])");
  std::smatch match;
  std::vector<double> deg_angles;

  if (std::regex_search(response, match, re))
  {
    std::string numbers = match[1].str();
    std::regex recomma(",");
    std::sregex_token_iterator it(numbers.begin(), numbers.end(), recomma, -1), end;
    for (; it != end; ++it)
    {
      deg_angles.push_back(std::stod(it->str()));
    }
  }

  return deg_angles;
}

inline bool sendCommand(int sockfd, const std::string &command, std::string &out_response)
{
  if (send(sockfd, command.c_str(), command.size(), 0) < 0)
  {
    return false;
  }

  // Receive response
  char recvData[1024];
  memset(recvData, 0, sizeof(recvData));
  int recvLen = recv(sockfd, recvData, sizeof(recvData) - 1, 0);
  if (recvLen > 0)
  {
    recvData[recvLen] = '\0';
    out_response = recvData;
  }
  else
  {
    out_response.clear();
  }

  return true;
}

}  // namespace

namespace mycobot_control
{

CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Create socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardware"), "Failed to create socket.");
    return CallbackReturn::ERROR;
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(addrNum.c_str());

  if (connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardware"), "Failed to connect to robot controller.");
    close(sockfd);
    return CallbackReturn::ERROR;
  }

  // Initialize joint states and commands
  joint_position_.resize(JOINT_COUNT, 0.0);
  joint_velocities_.resize(JOINT_COUNT, 0.0);
  joint_position_command_.resize(JOINT_COUNT, 0.0);
  joint_velocities_command_.resize(JOINT_COUNT, 0.0);

  // Map joint interfaces
  for (const auto &joint : info_.joints)
  {
    for (const auto &interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  //TODO
  // Set initial pose (in radians)
  std::vector<double> init_pose = {0.0, -1.57, 0.0, 0.0, -1.57, 0.0}; 
  joint_position_ = init_pose;
  joint_position_command_ = init_pose;
  //

  RCLCPP_INFO(rclcpp::get_logger("MyCobotHardware"), "MyCobot hardware interface initialized successfully.");

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(JOINT_COUNT * 2);

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(JOINT_COUNT * 2);

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  return command_interfaces;
}


//read
return_type RobotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{

  const std::string angles_cmd = "get_angles()\n";
  std::string response;

  if (!sendCommand(sockfd, angles_cmd, response))
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardware"), "Failed to send 'get_angles' command.");
    return return_type::ERROR;
  }

  auto deg_angles = parseAnglesFromResponse(response);
  if (deg_angles.size() != JOINT_COUNT)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardware"), "Invalid angle response from robot controller.");
    return return_type::ERROR;
  }

  auto rad_vec = deg2rad(deg_angles);
  for (size_t i = 0; i < joint_position_.size(); ++i)
  {
    joint_position_[i] = rad_vec[i];
  }


  return return_type::OK;
}

//write
return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{


  auto rad_angles = rad2deg(joint_position_command_);

  ////set speed
  std::string response;

  const std::string gain_command = createSetSpeedCommand(SPEED_PERCENT);
  if (!sendCommand(sockfd, gain_command, response))
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardware"), "Failed to send speed command.");
    return return_type::ERROR;
  }

  //set angle
  const std::string angle_command = createSetAnglesCommand(rad_angles, SPEED_VALUE);
  if (!sendCommand(sockfd, angle_command, response))
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardware"), "Failed to send set_angles command.");
    return return_type::ERROR;
  }

  return return_type::OK;
}

}  // namespace mycobot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mycobot_control::RobotSystem, hardware_interface::SystemInterface)

