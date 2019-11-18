#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.joints_pub2 = rospy.Publisher("joints2_pos",Float64MultiArray, queue_size=10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  #/////////
  def detect_red(self,image):
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])
 

  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  def pixel2meter(self,image):
      circle1Pos = self.detect_blue(image)
      circle2Pos = self.detect_green(image)
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)


  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    center = a * self.detect_yellow(image)
    circle1 = a * self.detect_yellow(image) 
    circle2 = a * self.detect_blue(image) 
    circle3 = a * self.detect_green(image)
    ja1 = np.arctan2(center[0]- circle1[0], center[1] - circle1[1])
    ja2 = np.arctan2(circle1[0]-circle2[0], circle1[1]-circle2[1]) - ja1
    ja3 = np.arctan2(circle2[0]-circle3[0], circle2[1]-circle3[1]) - ja2 - ja1
    return np.array([ja1, ja2, ja3])

  #///////

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    # im2=cv2.imshow('window2', cv_image2)
    # cv2.waitKey(1)

    a = self.detect_joint_angles(cv_image2)
    cv2.imshow('window', cv_image2)
    cv2.waitKey(3)

    self.joints2 = Float64MultiArray()
    self.joints2.data = a
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
      self.joints_pub2.publish(self.joints2)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


