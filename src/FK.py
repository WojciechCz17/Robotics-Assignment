#!/usr/bin/env python
import math 
import roslib
import sys
# import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class FK:
    def __init__(self):
        self.thing = 1

    def transformation(theta, alpha, a, d):
        # DH_params = [alpha, a, d, theta]

        trans_d = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, a],
                            [0, 0, 0, 1]])

        trans_a = np.array([[1, 0, 0, alpha],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0,0, 0, 1]])
        
        theta = DH_params[3]
        trans_theta = np.array([[math.cos(theta), -math.sin(theta), 0, 0],
                                [math.sin(theta), math.cos(theta), 0, 0],
                                [0, 0, 1, 0],
                                [0, 0,0, 1]])

        alpha = DH_params[0]
        trans_alpha = np.array([[1, 0, 0, 0],
                                [0, math.cos(alpha), -math.sin(alpha), 0],
                                [0, math.sin(alpha), math.cos(alpha), 0],
                                [0, 0, 0, 1]])

        T = np.dot(trans_d, np.dot(trans_theta, np.dot(trans_a, trans_alpha)))
        
        return T

    def quick_trans(theta, alpha, a, d):
        T = [[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(theta), d],
            [0, 0, 0, 1]]

        return T
    # def detect_end_effector(self,image):
        # a = self.pixel2meter(image)
        # endPos = a * (self.detect_yellow(image) - self.detect_red(image))
        # print(endPos)
        # return endPos

def main():
    fk = FK()
    T = fk.transformation(1, 0, 3, 0)
    print(T)
    x = np.dot(T, [2, 8, 3])
    print(x)
    print("HI")

if __name__ == "__main__":
    main()
    

    