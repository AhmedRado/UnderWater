#!/usr/bin/env python
import tabnanny
import rospy
from dynamic_reconfigure.server import Server
from awesome_package.cfg import PidControlConfig
import numpy as np

from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64, Float32

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from fav_msgs.msg import RangeMeasurementArray
import sympy as sym

import threading
import  math

rospy.init_node("localization")
    
statesX_pub = rospy.Publisher("KalmanX", Float64, queue_size=1)
statesY_pub = rospy.Publisher("KalmanY", Float64, queue_size=1)
statesZ_pub = rospy.Publisher("KalmanZ", Float64, queue_size=1)

rangeX_pub = rospy.Publisher("RangeX", Float64, queue_size=1)
rangeY_pub = rospy.Publisher("RangeY", Float64, queue_size=1)
rangeZ_pub = rospy.Publisher("RangeZ", Float64, queue_size=1)



roll = 0.0
yaw = 0.0
pitch = 0.0
apriltag_1 = [0.50, 3.35, -0.50]
apriltag_2 = [apriltag_1[0]+0.6,apriltag_1[1],apriltag_1[2]]
apriltag_3 = [apriltag_1[0],apriltag_1[1],apriltag_1[2]-0.4]
apriltag_4 = [apriltag_1[0]+0.6,apriltag_1[1],apriltag_1[2]-0.4]

result = 0

result_msgX = Float64()
Kalman_msgX = Float64()
result_msgY = Float64()
Kalman_msgY = Float64()
result_msgZ = Float64()
Kalman_msgZ = Float64()

class KalmanFilter:
    def __init__(self):
        # Initial State
        self.x = np.matrix([[0.5],
                            [1.0],
                            [-0.5]])

        # Uncertainity Matrix
        self.P = np.matrix([[0.1, 0., 0.],
                            [0., 0.1, 0.],
                            [0., 0., 0.1]])

        # Next State Function
        self.A = np.matrix([[1., 0., 0.],
                            [0,  1., 0.],
                            [0., 0., 1.]])
        self.B = np.matrix([[0.],
                            [0.],
                            [0.]])
        # Measurement Function
        self.H = np.matrix([[1., 0., 0.],
                            [0., 1., 0.],
                            [0., 0., 1.]])

        # Measurement Uncertainty
        self.R = np.matrix([[0.1]])

        # Identity Matrix
        self.I = np.matrix([[1., 0., 0.],
                            [0., 1., 0.],
                            [0., 0., 1.]])
        # Steering Angle Input
        self.SteeringAng = 0.0

    def predict(self):
        self.x = (self.A * self.x) + (self.B * self.SteeringAng)
        self.P = self.A * self.P * np.transpose(self.A)
        self.P[0, 0] += 100
        self.P[1, 1] += 100
        self.P[2, 2] += 100
   
    def measure_and_update(self, measurements):
        Z = np.matrix(measurements)
        y = np.transpose(Z) - self.H * self.x
        S = self.H * self.P * np.transpose(self.H) + self.R
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)
        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P
        
        return self.x
    def recieve_inputs(self, u_steer):
        self.SteeringAng = u_steer
        return

def get_rotation(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)   
    roll = roll * (180/3.14)
    yaw = yaw * (180/3.14)
    pitch = pitch * (180*3.14)   


    


def range_callback(range_msg):
    global d0 
    global d1 
    global d2 
    global d3 
    eq0 =0
    eq1 =0
    eq2 =0
    eq3 =0
    array_size = len(range_msg.measurements)
    result=0
    x,y,z = sym.symbols('x,y,z')
    for i in range (array_size):
        if(range_msg.measurements[i].id==0):
            d0 = range_msg.measurements[i].range
            eq0 = sym.Eq((x-apriltag_1[0])**2+(y-apriltag_1[1])**2+(z-apriltag_1[2])**2,d0**2)
        if(range_msg.measurements[i].id==1):
            d1 = range_msg.measurements[i].range
            eq1 = sym.Eq((x-apriltag_2[0])**2+(y-apriltag_2[1])**2+(z-apriltag_2[2])**2,d1**2)
        if(range_msg.measurements[i].id==2):
            d2 = range_msg.measurements[i].range
            eq2 = sym.Eq((x-apriltag_3[0])**2+(y-apriltag_3[1])**2+(z-apriltag_3[2])**2,d2**2)
        if(range_msg.measurements[i].id==3):
            d3 = range_msg.measurements[i].range
            eq3 = sym.Eq((x-apriltag_4[0])**2+(y-apriltag_4[1])**2+(z-apriltag_4[2])**2,d3**2)

    
    if(array_size>2 and array_size<4):
       
      result = sym.solve([eq0,eq1,eq2,eq3],(x,y,z))
      measurements = np.matrix( [[result[0][0],0.,0.],
                              [0.,result[0][1]-0.2,0.],
                              [0.,0.,result[0][2]-0.1]])
      K = KalmanFilter()
      K.predict()
      X = K.measure_and_update(measurements)
      result_msgX.data = result[0][0]
      Kalman_msgX.data = X[0,0]

      result_msgY.data = result[0][1]-0.2
      Kalman_msgY.data = X[1,1]

      result_msgZ.data = result[0][2]-0.1
      Kalman_msgZ.data = X[2,2]


      rangeX_pub.publish(result_msgX)
      rangeY_pub.publish(result_msgY)
      rangeZ_pub.publish(result_msgZ)

      statesX_pub.publish(Kalman_msgX)
      statesY_pub.publish(Kalman_msgY)
      statesZ_pub.publish(Kalman_msgZ) 


if __name__ == "__main__":

    

    



    #orient_sub = rospy.Subscriber ('ground truth/state', Odometry, get_rotation)

    range_sub = rospy.Subscriber("ranges",RangeMeasurementArray, range_callback)

    
    while not rospy.is_shutdown():
        rate = rospy.Rate(50)
        rate.sleep()


    

            
