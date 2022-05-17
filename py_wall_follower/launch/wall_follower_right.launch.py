import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='py_wall_follower', executable='robot_controller_right',
      output='screen'),
    Node(package='py_wall_follower', executable='robot_estimator',
      output='screen'),
  ])
