#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import math
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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a publisher to send joints' angular position to a topic called joints_pos
    self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a publisher to send robot end-effector position
    self.end_effector_pub = rospy.Publisher("end_effector_prediction",Float64MultiArray, queue_size=10)

    #//////////////////

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
    # centres of blobs
    center = a * self.detect_yellow(image)
    circle1 = a * self.detect_yellow(image) 
    circle2 = a * self.detect_blue(image)
    circle3 = a * self.detect_blue(image) 
    circle4 = a * self.detect_green(image)
    ja1 = np.arctan2(center[0]- circle1[0], center[1] - circle1[1])
    ja2 = np.arctan2(circle1[0]-circle2[0], circle1[1]-circle2[1]) - ja1
    ja3 = np.arctan2(circle1[0]-circle2[0], circle1[1]-circle2[1]) - ja1
    ja4 = np.arctan2(circle3[0]-circle4[0], circle3[1]-circle4[1]) - ja2 - ja1
    return np.array([ja1, ja2, ja3, ja4])

  def transformation(self, DH_params):
        # DH_params = [alpha, a, d, theta]
        theta = DH_params[0]
        alpha = DH_params[1]
        a = DH_params[2]
        d = DH_params[3]

        trans_d = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, a],
                            [0, 0, 0, 1]])

        trans_a = np.array([[1, 0, 0, alpha],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0,0, 0, 1]])
        
        trans_theta = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                                [math.sin(theta), math.cos(theta), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0,0, 1]])

        trans_alpha = np.array([[1, 0, 0, 0],
                                [0, math.cos(alpha), -math.sin(alpha), 0],
                                [0, math.sin(alpha), math.cos(alpha), 0],
                                [0, 0, 0, 1]])

        T = np.dot(trans_d, np.dot(trans_theta, np.dot(trans_a, trans_alpha)))
        
        return T


#////////////////////////////////////////////////////////

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    cv2.imshow('window', cv_image)
    cv2.waitKey(3)
    
    self.angles = self.detect_joint_angles(cv_image)
    self.joints = Float64MultiArray()
    self.joints.data = self.angles

    t01 = self.transformation([math.pi, 0, 2, self.angles[0]])
    t12 = self.transformation([0, 0, 0, self.angles[1]])
    t23 = self.transformation([0, 3, 0, self.angles[2]])
    t34 = self.transformation([0, 2, 0, self.angles[3]])

    self.t40 = Float64MultiArray()
    self.t04 = np.dot(t01, np.dot(t12, np.dot(t23, t34))) # is this right order?
    
    self.end_effector=Float64MultiArray()
    print(self.t04[0,3])
    self.end_effector.data = [self.t04[0, 3], self.t04[1,3], self.t04[2,3]]

    # Publish the results    
    try: 
      # end = receive("/joints_pos", 5)
      # end_vals = end.data

      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.joints_pub.publish(self.joints)
      self.end_effector_pub.publish(self.end_effector)
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


