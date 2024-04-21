#! /usr/bin/env python3

import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

 
'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Set the robot's initial pose if necessary
  initial_pose = PoseStamped()
  initial_pose.header.frame_id = 'map'
  initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  initial_pose.pose.position.x = 0.0
  initial_pose.pose.position.y = 0.0
  initial_pose.pose.position.z = 0.05
  initial_pose.pose.orientation.x = 0.0
  initial_pose.pose.orientation.y = 0.0
  initial_pose.pose.orientation.z = 0.0
  initial_pose.pose.orientation.w = 1.0
  navigator.setInitialPose(initial_pose)
 
  # Activate navigation, if not autostarted. This should be called after setInitialPose()
  # or this will initialize at the origin of the map and update the costmap with bogus readings.
  # If autostart, you should `waitUntilNav2Active()` instead.
  # navigator.lifecycleStartup()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()
 
  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')
 
  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()
 
  # Set the robot's goal poses
  goal_poses = []
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.9999997
  goal_pose.pose.orientation.w = 0.0007963
  goal_poses.append(goal_pose)

   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 2.0
  goal_pose.pose.position.y = -7.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.7073883
  goal_pose.pose.orientation.w = -0.7068252
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -3.0
  goal_pose.pose.position.y = 2.5
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.7068252
  goal_pose.pose.orientation.w = 0.7073883
  goal_poses.append(goal_pose)
   
  
  # sanity check a valid path exists
  # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
 
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(goal_poses)
 
  i = 0
  while not navigator.isTaskComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Executing current waypoint: ' +
            str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
      now = navigator.get_clock().now()
 
      # Some navigation timeout to demo cancellation
      if now - nav_start > Duration(seconds=100000000.0):
        navigator.cancelNav()
 
      # # Some follow waypoints request change to demo preemption
      # if now - nav_start > Duration(seconds=500000.0):
      #   goal_pose_alt = PoseStamped()
      #   goal_pose_alt.header.frame_id = 'map'
      #   goal_pose_alt.header.stamp = now.to_msg()
      #   goal_pose_alt.pose.position.x = -6.5
      #   goal_pose_alt.pose.position.y = -4.2
      #   goal_pose_alt.pose.position.z = 0.0
      #   goal_pose_alt.pose.orientation.x = 0.0
      #   goal_pose_alt.pose.orientation.y = 0.0  
      #   goal_pose_alt.pose.orientation.z = 0.0
      #   goal_pose_alt.pose.orientation.w = 1.0
      #   goal_poses = [goal_pose_alt]
      #   nav_start = now
      #   navigator.followWaypoints(goal_poses)
 
  # Do something depending on the return code
  result = navigator.getResult()
  if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
  elif result == TaskResult.FAILED:
    print('Goal failed!')
  else:
    print('Goal has an invalid return status!')
 
  navigator.lifecycleShutdown()
 
  exit(0)
 
if __name__ == '__main__':
  main()