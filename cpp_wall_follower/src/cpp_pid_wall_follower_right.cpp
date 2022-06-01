#include "turtlebot3_drive.hpp"

/*******************************************************************************
** Main
*******************************************************************************/

Turtlebot3Drive::Turtlebot3Drive()
: Node("turtlebot3_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &Turtlebot3Drive::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, std::bind(&Turtlebot3Drive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Turtlebot3Drive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised");

  // To plot data
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos);
  timer_ = this->create_wall_timer(100ms, std::bind(&Turtlebot3Drive::timer_callback_, this));

}

void Turtlebot3Drive::timer_callback_()
{
  auto message = std_msgs::msg::String();
  message.data = std::to_string(right_distance) + " " + std::to_string(TD) + " " + std::to_string(PID) + " " + std::to_string(alpha) ;
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void Turtlebot3Drive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::vector<float> laser_ranges;
  laser_ranges = msg->ranges;
 
  range_min = msg->range_max; 
  range_max = msg->range_min;

  side_min = 3.5; // maximum distance of the laser sensor
  front_side_min = side_min;
  all_side_min = side_min;

  right_distance = msg->ranges[270];
  
  if (laser_ranges[345] > 0 ){
    if(laser_ranges[345] > side_min){
      A1 = side_min;
    }
    else{
      A1 = laser_ranges[345];
    }
  }
  
  if (laser_ranges[300] > 0){
    if(laser_ranges[300] > side_min){
      B1 = side_min;
    }
    else{
      B1 = laser_ranges[300];
    }
  }

  for (int a = 0; a < 360; a++) {
    if (a >= 30 && a <= 330){
      continue;
    }
    if (laser_ranges[a] < front_side_min && laser_ranges[a] > 0) {
          front_side_min = laser_ranges[a];
      }
  }

  for (int a = 0; a < 360; a++) {
    if (laser_ranges[a] < all_side_min && laser_ranges[a] > 0) {
          all_side_min = laser_ranges[a];
      }
  }

 }

void Turtlebot3Drive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void Turtlebot3Drive::update_callback()
{
  theta = 45; // theta is the angle difference between A1 and B1

  // To-Do: adjust the kp, ki, and kd values!
  float kp = 3.0; // 4.0
  float ki = 0.2; // 0.5
  float kd = 2.0; // 5.0

  float AC = 0.2; // forward distance which is user-defined value
  float DT = 0.8; // desired trajectory

  float error, prev_error;
  float pid, integral, derivative, setpoint, measured_value;

  alpha = (RAD2DEG * (atan((A1 * cos(DEG2RAD * theta) - B1) / (A1 * sin(DEG2RAD * theta))))) - 30; // heading angle
  
  float AB = B1 * cos(DEG2RAD * (alpha + 30));
  float CD = AB + AC * sin(DEG2RAD * (alpha + 30)); // the distance to the wall that needs to be maintained

  setpoint = DT;
  measured_value = CD;

  error = setpoint - measured_value;
  integral += error;
  derivative = error - prev_error;

  if(setpoint == 0 && error == 0)
  {
      integral = 0;
      derivative = 0;
  }

  pid = (kp * error) + (ki * integral) + (kd * derivative);
  prev_error = error;

  if(pid > 1.0){
    pid = 1.0;
  }
  if(pid < - 1.0){
    pid = - 1.0;
  }

  if(all_side_min > 1.0){
    update_cmd_vel(0.10, 0.0);
  }
  else{
    if (front_side_min > 0.4){
      update_cmd_vel(0.10, pid);
    }
    else {
      update_cmd_vel(0.0, 1.0);
    }
  }

  //RCLCPP_INFO(this->get_logger(), "PID: %lf, alpha: %lf, wall distance: %lf, distance at 90 deg: %lf", pid, alpha, CD, right_distance);
  PID = pid;
  TD = CD;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtlebot3Drive>()); 
  rclcpp::shutdown();
  return 0;
}
