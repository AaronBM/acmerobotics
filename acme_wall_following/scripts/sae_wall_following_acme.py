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
MAX_STEERING_ANGLE = 0.5
MAX_VELOCITY = 1.0

# Controller parameters
kp = .5
ki = 0.0
kd = 0.0

kp_vel = 0.5

# Other global variables
error = 0.0
prev_error = 0.0


def control(centerline_error):
  global kp
  global kd
  global VELOCITY
  global prev_error
  global MAX_STEERING_ANGLE
  global MAX_VELOCITY
  global kp_vel

  # TO-DO: Implement controller
  # ---

  if abs(centerline_error) > 0:
      prop_term = centerline_error
      int_term = prev_error + centerline_error * (1 / FREQUENCY)
      der_term = (centerline_error - prev_error) / (1 / FREQUENCY)
      STEERING_ANGLE = prop_term * kp + int_term * ki + der_term * kd

  prev_error = centerline_error # Store current centerline to old error state
  print('P: ',centerline_error,', I:', int_term, 'D: ',der_term)
  print('Unattenuated Command: ', STEERING_ANGLE)
  # ---

  # Set maximum thresholds for steering angles
  if STEERING_ANGLE > MAX_STEERING_ANGLE:
    STEERING_ANGLE = MAX_STEERING_ANGLE
  elif STEERING_ANGLE < -MAX_STEERING_ANGLE:
    STEERING_ANGLE = -MAX_STEERING_ANGLE

  print("Steering Angle is = %f" % STEERING_ANGLE)

  # Velocity controller
  VELOCITY = MAX_VELOCITY - kp_vel * abs(STEERING_ANGLE) / MAX_STEERING_ANGLE

  # Command at least a little velocity
  if VELOCITY <= 0.001:
    VELOCITY = 0.1

  print('Commanded velocity = %f' % VELOCITY)

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
  #print(index)

  #print('Current index: [', index[0],', ',index[-1], ']')

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
  global VELOCITY

  # TO-DO: Find index of the two rays, and calculate a, b, alpha and theta. Find the actual distance from the right wall.
  # ---
  distance_a = get_scan_distance(angle_lookahead,data)
  distance_b = get_scan_distance(angle_right,data)
  print('Distance (45): ',distance_a)
  print('Distance (90): ',distance_b)

  theta = angle_right - angle_lookahead
  theta_rad = theta * np.pi / 180

  alpha_rad = np.arctan( (distance_a * np.cos(theta_rad) - distance_b) / distance_a * np.sin(theta_rad) )
  alpha = alpha_rad * 180 / np.pi
  distance_r = distance_b * np.cos(alpha_rad)
  print('Distance R: ',distance_r)
  l = VELOCITY / FREQUENCY

  # ---

  print('Distance from wall(', angle_right,'): ', distance_r)

  # Calculate error
  error = DISTANCE_RIGHT_THRESHOLD - distance_r + (l * np.cos(alpha_rad))

  if np.isnan(error):
    error = 0

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
  print('Error right: ', er)
  print('Error left: ', el)

  centerline_error = er - el
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
