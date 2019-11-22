#!/usr/bin/env python

import math 
import roslib
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

# take transformation matrix for 0-joint. The last column gives co-ords of joint

class FK:
    def __init__(self):
        self.thing = 1

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

    def quick_trans(self, theta, alpha, a, d):
        T = [[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(theta), d]]

        return T
    # def detect_end_effector(self,image):
        # a = self.pixel2meter(image)
        # endPos = a * (self.detect_yellow(image) - self.detect_red(image))
        # print(endPos)
        # return endPos
def main():
    fk = FK()
    T = fk.transformation([1, 0, 3, 0])
    print(T)
    # add 1 to p then remove 1 from p'
    x = np.dot(T, [0, 0, 0, 0])
    x = x[:-1]
    print(x)

    # DH_params = [[]]

    # for i in range(0,4):
    #     theta = 
    # compute each transformation matrix and multiply together
    # will put in dummy values for now 
    # put the DH parameter table in
    
if __name__ == "__main__":
    main()
    

    