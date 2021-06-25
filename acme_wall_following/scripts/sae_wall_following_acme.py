#!/usr/bin/env python3

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import pdb

# Vehicle parameters
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DISTANCE_RIGHT_THRESHOLD = 0.5  # (m)
VELOCITY = 0.1  # meters per second
FREQUENCY = 10  # 10 Hz

# Controller parameters
kp = 0.7
ki = 0.2
kd = 0.0

# Other global variables
error = 0.0
prev_error = 0.0


def control(centerline_error):
  global kp
  global kd
  global VELOCITY

  # TO-DO: Implement controller
  # ---
  if abs(centerline_error) > 0:
    try:
      prop_term = centerline_error
      int_term = prev_error + centerline_error * (1 / FREQUENCY)
      der_term = (centerline_error - prev_error) / (1 / FREQUENCY)
      STEERING_ANGLE = prop_term * kp + int_term * ki + der_term * kd
    except:
      STEERING_ANGLE = 0

    prev_error = centerline_error
  # ---

  # Set maximum thresholds for steering angles
  if STEERING_ANGLE > 0.5:
    STEERING_ANGLE = 0.5
  elif STEERING_ANGLE < -0.5:
    STEERING_ANGLE = -0.5

  print("Steering Angle is = %f" % STEERING_ANGLE)

  # TO-DO: Publish the message
  # ---
  msg = AckermannDriveStamped()
  msg.drive.speed = VELOCITY
  msg.drive.steering_angle = STEERING_ANGLE
  pub.publish(msg)

# ---  # Aaron will code this.  Note that he will need to implement a publisher down at the bottom (we think?)
# ---

def get_index(angle, data):
  # 	# TO-DO: For a given angle, return the corresponding index for the data.ranges array
  # ---
  angle_limit = 2
  angle_inc = data.angle_increment * 180 / np.pi
  angle_min = data.angle_min * 180 / np.pi
  angle_max = data.angle_max * 180 / np.pi

  all_angles = np.arange(angle_min, angle_max, angle_inc)

  # Find the places where the angle is within the specified limits
  index = np.where((angle - angle_limit < all_angles) & (all_angles < angle + angle_limit))[0]
  print(index)

  print('Current index: [', index[0],', ',index[-1], ']')

  return index
# ---

def get_scan_distance(angle,data):
  # returns the average distance from the scanner
  index = get_index(angle,data)
  scan = np.array(data.ranges[index[0]:index[-1]])
  clean_scan = scan[np.isfinite(scan)]
  ave_distance = np.mean(clean_scan)

  return ave_distance


def distance(angle_right, angle_lookahead, data):
  global ANGLE_RANGE
  global DISTANCE_RIGHT_THRESHOLD

  # TO-DO: Find index of the two rays, and calculate a, b, alpha and theta. Find the actual distance from the right wall.
  # ---
  distance_a = get_scan_distance(angle_lookahead,data)
  distance_b = get_scan_distance(angle_right,data)
  print('Distance Ahead: ',distance_a)
  print('Distance Right: ',distance_b)

  theta = angle_right - angle_lookahead
  theta_rad = theta * np.pi / 180

  distance_c = pow(distance_a ** 2 + distance_b ** 2 - 2 * distance_a * distance_b * np.cos(theta_rad),
                   0.5)  # Ray between end of a and b
  distance_r = distance_a * distance_b / distance_c * np.sin(theta_rad)  # Distance from wall

  if (distance_a ** 2 - distance_r ** 2 < distance_c ** 2):
    alpha = -np.arccos(distance_r / distance_b)
  else:
    alpha = np.arccos(distance_r / distance_b)

  l = VELOCITY / FREQUENCY

  # ---

  print("Distance from right wall : %f" % distance_r)

  # Calculate error
  error = DISTANCE_RIGHT_THRESHOLD - distance_r + (l * np.cos(alpha * np.pi / 180))

  return error, distance_r


def follow_center(angle_right, angle_lookahead_right, data):
  #angle_lookahead_left = 180 + angle_right
  #angle_left = 180 - angle_lookahead_right
  angle_lookahead_left = 45
  angle_left = 90

  er, dr = distance(angle_right, angle_lookahead_right, data)
  el, dl = distance(angle_left, angle_lookahead_left, data)

  # Find Centerline error
  # ---
  centerline_error = el - er
  # ---

  print("Centerline error = %f " % centerline_error)

  return centerline_error


def callback(data):
  # Pick two rays at two angles
  angle_right = -90  # arbirary
  angle_lookahead = -45  # arbitrary

  # To follow right wall
  # er, dr = distance(angle_right,angle_lookahead, data)

  # To follow the centerline
  ec = follow_center(angle_right, angle_lookahead, data)

  control(ec)

  rospy.Rate(FREQUENCY)


if __name__ == '__main__':
  print("Wall following started")
  rospy.init_node('wall_following', anonymous=True)

  # TO-DO: Implement the publishers and subscribers
  # ---
  sub = rospy.Subscriber("/scan", LaserScan, callback)
  pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
  # ---

  rospy.spin()
