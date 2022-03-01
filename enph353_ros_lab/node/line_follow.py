#!/usr/bin/env python
from __future__ import print_function

import roslib
import numpy as np
roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class line_follower:

  def __init__(self):
    self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.move = Twist()
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw',Image,self.callback, queue_size = 1)
    #self.rate = rospy.Rate(20)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    cropped_frame = cv_image[rows-10:rows, 0:cols]
    threshold = 120
    _, img_bin = cv2.threshold(cropped_frame, threshold, 255, cv2.THRESH_BINARY_INV)
    sumup = 0
    sumdown = 0
    for i in range(0,10):
      for j in range(0,cols):
        value = img_bin[i,j,0]
        sumup = sumup + value*j
        sumdown = sumdown + value
    #print(sumup)
    #print(sumdown)
    CW = -0.5
    CCW = 0.5
    if sumup == 0 or sumdown ==0:
      self.move.angular.z = CCW 
      self.move.linear.x = 0
    else:   
      centerwidth = sumup/sumdown
      print("centerwidth {}".format(centerwidth))
      print(cols/2)
      self.move.linear.x = 0.2
      if centerwidth >30 and centerwidth <cols-30:
        if centerwidth < cols/2:
          self.move.angular.z = CCW
          print('turning left')
        if centerwidth > cols/2:
          self.move.angular.z = CW
          print('turning right')
        if centerwidth == cols/2:
          self.move.angular.z = 0
    cv2.imshow('img_bin',img_bin)
    cv2.waitKey(3)

    try:
      self.move_pub.publish(self.move)
      #self.rate.sleep()
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('line_follower', anonymous=True)
  lf = line_follower()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)