// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Authors: Taehun Lim (Darby), Ryan Shim

#ifndef TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
#define TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.25
#define ANGULAR_VELOCITY 0.50

using namespace std;

// #define GET_TB3_DIRECTION   0
// #define TB3_FIND_WALL       1
// #define TB3_TURN_LEFT       2
// #define TB3_FOLLOW_THE_WALL 3


// #define TB3_DRIVE_FORWARD 1
// #define TB3_RIGHT_TURN    2
// #define TB3_LEFT_TURN     3

class Turtlebot3Drive : public rclcpp::Node
{
public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[3];
  
  float left_side;
  float right_side;
  float range_min; 
  float range_max;

  float side_min;
  float front_side_min;
  float front_left_side_min;
  float front_right_side_min;
  float left_side_min;
  float right_side_min;
  float behind_side_min;

  bool crashed = false;

  char const *robot_pos_state_;
 
  float alpha, theta;
  float A1, B1;
  float TD;
  float left_distance, right_distance;

  float error, prev_error;
  float pid, integral, derivative, setpoint, measured_value;
  float PID;
 
  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void tb3_full_stop();
  void tb3_find_wall();
  void tb3_turn_left();
  void tb3_turn_right();
  void tb3_follow_the_wall();
  void tb3_move_backward();
  
  // To plot data using ros2 /topic
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  void timer_callback_();
};
#endif  // TURTLEBOT3_GAZEBO__TURTLEBOT3_DRIVE_HPP_
