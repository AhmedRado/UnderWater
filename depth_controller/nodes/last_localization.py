#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from fav_msgs.msg import RangeMeasurementArray
import  math
from nav_msgs.msg import Odometry



statesX_pub = rospy.Publisher("KalmanX_velocity_pressure", Float64, queue_size=1)
statesY_pub = rospy.Publisher("KalmanY_velocity_pressure", Float64, queue_size=1)
statesZ_pub = rospy.Publisher("KalmanZ_velocity_pressure", Float64, queue_size=1)

apriltag_1 = [1.34, 4, -0.40]
apriltag_2 = [apriltag_1[0]+0.6,apriltag_1[1],apriltag_1[2]]
apriltag_3 = [apriltag_1[0],apriltag_1[1],apriltag_1[2]-0.4]
apriltag_4 = [apriltag_1[0]+0.6,apriltag_1[1],apriltag_1[2]-0.4]


MSE_x = Float64()
MSE_y = Float64()
MSE_z = Float64()

global rate

truth_x =0
truth_y =0
truth_z =0

rate = 7.0
dt = 1/rate


class KalmanFilter:
    def __init__(self):

        rospy.init_node("robot_localization_velocity_pressure")

        rospy.Subscriber("ranges",RangeMeasurementArray, self.range_callback)
        rospy.Subscriber("pressure",FluidPressure, self.pressure_callback)

        self.d0 = 2.35
        self.d1 = 2.42
        self.d2 = 2.38
        self.d3 = 2.45
        self.pressure = 98253

        self.Kalman_msgX = Float64()
        self.Kalman_msgY = Float64()
        self.Kalman_msgZ = Float64()

        x_var_init = (0.5)**2
        y_var_init = (0.5)**2
        z_var_init = (0.5)**2

        x_dot_var_init = (0.5)**2
        y_dot_var_init = (0.5)**2
        z_dot_var_init = (0.5)**2
        
        x_var_noise = (0.5)**2
        y_var_noise = (0.5)**2
        z_var_noise = (0.5)**2

        x_dot_var_noise = (0.5)**2
        y_dot_var_noise = (0.5)**2
        z_dot_var_noise = (0.5)**2

        noise_var = (0.01)**2
        noise_var_pressure = (100)**2

        self.raw_g = 1.0e4
        # self.measurements = np.empty(0)
        # self.h = np.empty(0)
        # self.h_fun = np.empty(0)
        # self.R = np.empty(0)

        self.x = np.matrix([[0.5],
                            [1.0],
                            [-0.5],
                            [0],
                            [0],
                            [0]])

        # Uncertainity Matrix
        self.P = np.diag([x_var_init,y_var_init,z_var_init,x_dot_var_init,y_dot_var_init,z_dot_var_init])
        # Next State Function
        self.A = np.matrix([[1., 0., 0., dt, 0., 0.],
                            [0,  1., 0., 0., dt, 0.],
                            [0., 0., 1., 0., 0., dt],
                            [0., 0., 0., 1., 0., 0.],
                            [0., 0., 0., 0., 1., 0.],
                            [0., 0., 0., 0., 0., 1.]])
        
        
        self.Q = np.diag([x_var_noise,y_var_noise,z_var_noise,x_dot_var_noise,y_dot_var_noise,z_dot_var_noise])

        # Measurement Uncertainty
        self.big_R = np.diag([noise_var, noise_var,noise_var,noise_var,noise_var_pressure])

        # Identity Matrix
        self.I = np.identity(n=6,dtype=float)

    def h_jacobian(self):

        sqrt1 = 2*math.sqrt((self.x[0,0]-apriltag_1[0])**2 + (self.x[1,0]-apriltag_1[1])**2 + (self.x[2,0]-apriltag_1[2])**2 )

        sqrt2 = 2*math.sqrt((self.x[0,0]-apriltag_2[0])**2 + (self.x[1,0]-apriltag_2[1])**2 + (self.x[2,0]-apriltag_2[2])**2 )

        sqrt3 = 2*math.sqrt((self.x[0,0]-apriltag_3[0])**2 + (self.x[1,0]-apriltag_3[1])**2 + (self.x[2,0]-apriltag_3[2])**2 )

        sqrt4 = 2*math.sqrt((self.x[0,0]-apriltag_4[0])**2 + (self.x[1,0]-apriltag_4[1])**2 + (self.x[2,0]-apriltag_4[2])**2 )

        h = np.matrix([ [2*(self.x[0,0]-apriltag_1[0])/sqrt1, 2*(self.x[1,0]-apriltag_1[1])/sqrt1, 2*(self.x[2,0]-apriltag_1[2])/sqrt1, 0., 0., 0.],
            
                        [2*(self.x[0,0]-apriltag_2[0])/sqrt2, 2*(self.x[1,0]-apriltag_2[1])/sqrt2, 2*(self.x[2,0]-apriltag_2[2])/sqrt2, 0., 0. ,0.],

                        [2*(self.x[0,0]-apriltag_3[0])/sqrt3, 2*(self.x[1,0]-apriltag_3[1])/sqrt3, 2*(self.x[2,0]-apriltag_3[2])/sqrt3, 0., 0. ,0.],

                        [2*(self.x[0,0]-apriltag_4[0])/sqrt4, 2*(self.x[1,0]-apriltag_4[1])/sqrt4, 2*(self.x[2,0]-apriltag_4[2])/sqrt4, 0., 0., 0.],

                        [0., 0., -self.raw_g, 0., 0., 0.]])

        return h


    def get_hfunc(self):
    
        h_1 = math.sqrt((self.x[0,0]-apriltag_1[0])**2 + (self.x[1,0]-apriltag_1[1])**2 + (self.x[2,0]-apriltag_1[2])**2) 
        h_2 = math.sqrt((self.x[0,0]-apriltag_2[0])**2 + (self.x[1,0]-apriltag_2[1])**2 + (self.x[2,0]-apriltag_2[2])**2)
        h_3 = math.sqrt((self.x[0,0]-apriltag_3[0])**2 + (self.x[1,0]-apriltag_3[1])**2 + (self.x[2,0]-apriltag_3[2])**2)
        h_4 = math.sqrt((self.x[0,0]-apriltag_4[0])**2 + (self.x[1,0]-apriltag_4[1])**2 + (self.x[2,0]-apriltag_4[2])**2)
        h_5 = -(self.raw_g *self.x[2,0]) + 101325

        h_fun = np.matrix([ [h_1],
                            [h_2],
                            [h_3],
                            [h_4],
                            [h_5]])
        return h_fun

    def predict(self):
        self.x = (self.A * self.x)
        self.P = self.A * self.P * np.transpose(self.A) + self.Q

    
    def measure_and_update(self,h ,h_fun , R , measurements):
        #self.H = self.h
        measurements = measurements.reshape(-1,1)
        h = h.reshape(-1,6)
        h_fun = h_fun.reshape(-1,1)
        R = np.diag(R)

        Z = measurements
        y = Z - h_fun 
        S = h * self.P * h.transpose() + R
        print('H: ', h.shape)
        print('H_fun: ', h_fun.shape)
        print('Z: ', Z.shape)
        K = self.P * h.transpose() * np.linalg.inv(S)
        self.x = self.x + K * y
        self.P = (self.I - K * h) * self.P
        self.x[0,0] = self.x[0,0] - 0.2
        self.x[2,0] = self.x[2,0] - 0.1
        return self.x

    def range_callback(self, range_msg):
        measurements_R = np.empty(0)
        h_R = np.empty(0)
        h_fun_R = np.empty(0)
        R_small_R = np.empty(0)
        R_R = np.empty(0)
        big_h = self.h_jacobian()
        big_h_fun = self.get_hfunc()
        array_size = len(range_msg.measurements)
        for i in range (array_size):
            if(range_msg.measurements[i].id==1):
                d0 = range_msg.measurements[i].range
                measurements_R = np.append(measurements_R,d0)
                h_R = np.append(h_R,big_h[0])
                h_fun_R = np.append(h_fun_R,big_h_fun[0])
                R_small_R = np.append(R_small_R,self.big_R[0,0])
            if(range_msg.measurements[i].id==2):
                d1 = range_msg.measurements[i].range
                measurements_R = np.append(measurements_R,d1)
                h_R = np.append(h_R,big_h[1])
                h_fun_R = np.append(h_fun_R,big_h_fun[1])
                R_small_R = np.append(R_small_R,self.big_R[1,1])
            if(range_msg.measurements[i].id==3):
                d2 = range_msg.measurements[i].range
                measurements_R = np.append(measurements_R,d2)
                h_R = np.append(h_R,big_h[2])
                h_fun_R = np.append(h_fun_R,big_h_fun[2])
                R_small_R = np.append(R_small_R,self.big_R[2,2])
            if(range_msg.measurements[i].id==4):
                d3 = range_msg.measurements[i].range
                measurements_R = np.append(measurements_R,d3)
                h_R = np.append(h_R,big_h[3])
                h_fun_R = np.append(h_fun_R,big_h_fun[3])
                R_small_R = np.append(R_small_R,self.big_R[3,3])                
        # print(h_fun_R)
        self.Kalman_function(h_R, h_fun_R, R_small_R, measurements_R)


    def pressure_callback(self, pressure_msg):
        # measurements_P = np.empty(0)
        # R_small_P = np.empty(0)
        # h_P = np.empty(0)
        # h_fun_P = np.empty(0)
        # pressure = pressure_msg.fluid_pressure
        big_h = self.h_jacobian()
        # big_h_fun = self.get_hfunc()
        # measurements_P = np.append(measurements_P,pressure)
        # h_P = np.append(h_P,big_h[4])
        # h_fun_P=np.append(h_fun_P,big_h_fun[4])
        # R_small_P=np.append(R_small_P,self.big_R[4,4])    
        # # h_P = h_P.reshape(-1,6)
        # # h_fun_P = h_fun_P.reshape(-1,1)
        # # measurements_P = measurements_P.reshape(-1,1)
        # R_P = np.diag(R_small_P)
        # # print('h_fun:',h_fun_P.shape)
        # self.Kalman_function(h_P, h_fun_P, R_P, measurements_P)

    def Kalman_function(self, h ,h_fun , R , measurements):
        # self.measurements = np.matrix([ [self.d0],
        #                         [self.d1],
        #                         [self.d2],
        #                         [self.d3],
        #                         [self.pressure]])
        
        X = self.measure_and_update(h ,h_fun , R , measurements)


        
        self.Kalman_msgX.data = X[0,0]
        statesX_pub.publish(self.Kalman_msgX)
        self.Kalman_msgY.data = X[1,0]
        statesY_pub.publish(self.Kalman_msgY)
        self.Kalman_msgZ.data = X[2,0]
        statesZ_pub.publish(self.Kalman_msgZ)


            
             



def groundtruth_callback(grountruth_msg):
    global truth_x
    global truth_y
    global truth_z
    truth_x = grountruth_msg.pose.pose.position.x
    truth_y = grountruth_msg.pose.pose.position.y
    truth_z = grountruth_msg.pose.pose.position.z



if __name__ == "__main__":
    #orient_sub = rospy.Subscriber ('ground truth/state', Odometry, get_rotation)


    K = KalmanFilter()
    

    MSE_x_pub = rospy.Publisher("MSE_X_velocity_pressure",Float64,queue_size=1)
    MSE_y_pub = rospy.Publisher("MSE_Y_velocity_pressure",Float64,queue_size=1)
    MSE_z_pub = rospy.Publisher("MSE_Z_velocity_pressure",Float64,queue_size=1)

    rospy.Subscriber("ground_truth/state",Odometry,groundtruth_callback)

    rate = rospy.Rate(7.0)
    while not rospy.is_shutdown():
        # print(truth_x)
        K.predict()
        MSE_x = math.sqrt(np.square(np.subtract( K.Kalman_msgX.data,truth_x)).mean())
        MSE_y = math.sqrt(np.square(np.subtract( K.Kalman_msgY.data,truth_y)).mean())
        MSE_z = math.sqrt(np.square(np.subtract( K.Kalman_msgZ.data,truth_z)).mean())
        
        MSE_x_pub.publish(MSE_x)
        MSE_y_pub.publish(MSE_y)
        MSE_z_pub.publish(MSE_z)

        rate.sleep()
