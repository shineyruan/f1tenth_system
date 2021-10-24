#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import atexit
import tf
import os
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

rospack = rospkg.RosPack()
particle_filter_pkg_path = rospack.get_path("particle_filter")
waypoint_dir = os.path.join(particle_filter_pkg_path, "csv")
if not os.path.exists(waypoint_dir):
  os.makedirs(waypoint_dir, exist_ok=True)
file = open(os.path.join(waypoint_dir, "waypoint.csv"), 'w')


def save_waypoint(data):
  quaternion = np.array([
      data.pose.pose.orientation.x, data.pose.pose.orientation.y,
      data.pose.pose.orientation.z, data.pose.pose.orientation.w
  ])

  euler = tf.transformations.euler_from_quaternion(quaternion)
  speed = LA.norm(
      np.array([
          data.twist.twist.linear.x, data.twist.twist.linear.y,
          data.twist.twist.linear.z
      ]), 2)
  if data.twist.twist.linear.x > 0.:
    print(data.twist.twist.linear.x)

  file.write(
      '%f, %f, %f, %f\n' %
      (data.pose.pose.position.x, data.pose.pose.position.y, euler[2], speed))


def shutdown():
  file.close()
  print('Goodbye')


def listener():
  rospy.init_node('waypoints_logger', anonymous=True)
  rospy.Subscriber('pf/pose/odom', Odometry, save_waypoint)
  rospy.spin()


if __name__ == '__main__':
  atexit.register(shutdown)
  print('Saving waypoints...')
  try:
    listener()
  except rospy.ROSInterruptException:
    pass
