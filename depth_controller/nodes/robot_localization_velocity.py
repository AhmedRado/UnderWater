#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from fav_msgs.msg import RangeMeasurementArray
from nav_msgs.msg import Odometry
from sklearn.metrics import mean_squared_error
import  math

rospy.init_node("robot_localization_velocity")



statesX_pub = rospy.Publisher("KalmanX_velocity", Float64, queue_size=1)
statesY_pub = rospy.Publisher("KalmanY_velocity", Float64, queue_size=1)
statesZ_pub = rospy.Publisher("KalmanZ_velocity", Float64, queue_size=1)


rmseX_pub = rospy.Publisher("rmseX_velocity", Float64, queue_size=1)
rmseY_pub = rospy.Publisher("rmseY_velocity", Float64, queue_size=1)
rmseZ_pub = rospy.Publisher("rmseZ_velocity", Float64, queue_size=1)

apriltag_1 = [1.34, 4, -0.40]
# apriltag_1 = [0.50, 3.35, -0.50]
apriltag_2 = [apriltag_1[0]+0.6,apriltag_1[1],apriltag_1[2]]
apriltag_3 = [apriltag_1[0],apriltag_1[1],apriltag_1[2]-0.4]
apriltag_4 = [apriltag_1[0]+0.6,apriltag_1[1],apriltag_1[2]-0.4]


Kalman_msgX = Float64()
Kalman_msgY = Float64()
Kalman_msgZ = Float64()

rmseX  = Float64()
rmseY  = Float64()
rmseZ  = Float64()

raw_g = 1.0e4
p0 = 101325

global rate
rate = 7
dt = 1/rate
global d0 
global d1 
global d2 
global d3 
global pressure

d0 =2.35
d1 = 2.42
d2= 2.38
d3 =2.45
pressure = 96325


global true_x
global true_y
global true_z

true_x = 0
true_y = 0
true_z = 0

class KalmanFilter:
    def __init__(self):
        
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

        noise_var= (0.001)**2


        self.x = np.matrix([[0.5],
                            [1.0],
                            [-0.5],
                            [0.0],
                            [0.0],
                            [0.0]])

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
        self.R = np.diag([noise_var, noise_var,noise_var,noise_var])

        # Identity Matrix
        self.I = np.identity(n=6,dtype=float)
    
    def h_jacobian(self):

        sqrt_1 = 2*math.sqrt((self.x[0,0]-apriltag_1[0])**2 + (self.x[1,0]-apriltag_1[1])**2 + (self.x[2,0]-apriltag_1[2])**2 )
        sqrt_2 = 2*math.sqrt((self.x[0,0]-apriltag_2[0])**2 + (self.x[1,0]-apriltag_2[1])**2 + (self.x[2,0]-apriltag_2[2])**2 )
        sqrt_3 = 2*math.sqrt((self.x[0,0]-apriltag_3[0])**2 + (self.x[1,0]-apriltag_3[1])**2 + (self.x[2,0]-apriltag_3[2])**2 )
        sqrt_4 = 2*math.sqrt((self.x[0,0]-apriltag_4[0])**2 + (self.x[1,0]-apriltag_4[1])**2 + (self.x[2,0]-apriltag_4[2])**2 )

        h = np.matrix([ [2*(self.x[0,0]-apriltag_1[0])/sqrt_1, 2*(self.x[1,0]-apriltag_1[1])/sqrt_1, 2*(self.x[2,0]-apriltag_1[2])/sqrt_1, 0., 0., 0.],
            
                        [2*(self.x[0,0]-apriltag_2[0])/sqrt_2, 2*(self.x[1,0]-apriltag_2[1])/sqrt_2, 2*(self.x[2,0]-apriltag_2[2])/sqrt_2, 0., 0. ,0.],

                        [2*(self.x[0,0]-apriltag_3[0])/sqrt_3, 2*(self.x[1,0]-apriltag_3[1])/sqrt_3, 2*(self.x[2,0]-apriltag_3[2])/sqrt_3, 0., 0. ,0.],

                        [2*(self.x[0,0]-apriltag_4[0])/sqrt_4, 2*(self.x[1,0]-apriltag_4[1])/sqrt_4, 2*(self.x[2,0]-apriltag_4[2])/sqrt_4, 0., 0., 0.]])

        return h

    def get_hfunc(self):
    
        h_1 = math.sqrt((self.x[0,0]-apriltag_1[0])**2 + (self.x[1,0]-apriltag_1[1])**2 + (self.x[2,0]-apriltag_1[2])**2) 
        h_2 = math.sqrt((self.x[0,0]-apriltag_2[0])**2 + (self.x[1,0]-apriltag_2[1])**2 + (self.x[2,0]-apriltag_2[2])**2)
        h_3 = math.sqrt((self.x[0,0]-apriltag_3[0])**2 + (self.x[1,0]-apriltag_3[1])**2 + (self.x[2,0]-apriltag_3[2])**2)
        h_4 = math.sqrt((self.x[0,0]-apriltag_4[0])**2 + (self.x[1,0]-apriltag_4[1])**2 + (self.x[2,0]-apriltag_4[2])**2)
        

        h_fun = np.matrix([ [h_1],
                            [h_2],
                            [h_3],
                            [h_4]])
        return h_fun

    def predict(self):
        self.x = (self.A * self.x)
        self.P = self.A * self.P * np.transpose(self.A) + self.Q

    
    def measure_and_update(self, measurements):
        self.H = self.h_jacobian()
        Z = measurements
        y = Z - self.get_hfunc()
        S = self.H * self.P * self.H.transpose() + self.R
        K = self.P * self.H.transpose() * np.linalg.inv(S)
        self.x = self.x + K * y
        self.P = (self.I - K * self.H) * self.P
        self.x[0,0] = self.x[0,0] - 0.2
        self.x[2,0] = self.x[2,0] - 0.1
        return self.x
def range_callback(range_msg):
    
    
    global d0 

    global d1 

    global d2 

    global d3 
    

    global pressure

    array_size = len(range_msg.measurements)

    
    for i in range (array_size):
        if(range_msg.measurements[i].id==1):
            d0 = range_msg.measurements[i].range

        if(range_msg.measurements[i].id==2):
            d1 = range_msg.measurements[i].range
        
        if(range_msg.measurements[i].id==3):
            d2 = range_msg.measurements[i].range
            
        if(range_msg.measurements[i].id==4):
            d3 = range_msg.measurements[i].range 
           
    Kalman_function()
    

def ground_truth_callback(groud_msg):

   global true_x
   global true_y
   global true_z 

   true_x = groud_msg.pose.pose.position.x

   true_y = groud_msg.pose.pose.position.y

   true_z = groud_msg.pose.pose.position.z
                                    

def Kalman_function():

    global d0 
    global d1 
    global d2 
    global d3 


    global pressure
    #global K

    measurements = np.matrix([ [d0],
                               [d1],
                               [d2],
                               [d3]])
   
   
    X = K.measure_and_update(measurements)

    
    Kalman_msgX.data = X[0,0]


    Kalman_msgY.data = X[1,0]


    Kalman_msgZ.data = X[2,0]


    statesX_pub.publish(Kalman_msgX)
    statesY_pub.publish(Kalman_msgY)
    statesZ_pub.publish(Kalman_msgZ) 


if __name__ == "__main__":
    #orient_sub = rospy.Subscriber ('ground truth/state', Odometry, get_rotation)

   # global K
    K = KalmanFilter()

    range_sub = rospy.Subscriber("ranges",RangeMeasurementArray, range_callback)

    ground_truth_sub = rospy.Subscriber("ground_truth/state",Odometry, ground_truth_callback)

    rate = rospy.Rate(7.0)

    count = 0
    mean_x = np.zeros(1000)
    mean_y = np.zeros(1000)
    mean_z = np.zeros(1000)

    while not rospy.is_shutdown():

        
        
        K.predict()

        if(count<1000):
            mean_x[count] = (Kalman_msgX.data - true_x)**2
            mean_y[count] = (Kalman_msgY.data - true_y)**2
            mean_z[count] = (Kalman_msgZ.data - true_z)**2
            count = count +1
        else:
            count =0



        rmseX.data = math.sqrt(np.sum(mean_x)/1000)
        rmseY.data = math.sqrt(np.sum(mean_y)/1000)
        rmseZ.data = math.sqrt(np.sum(mean_z)/1000)
        
      
        #rmseX.data = math.sqrt(np.square(np.subtract(Kalman_msgX.data,true_x)).mean())
        #rmseY.data = math.sqrt(np.square(np.subtract(Kalman_msgY.data,true_y)).mean())
        #rmseZ.data = math.sqrt(np.square(np.subtract(Kalman_msgZ.data,true_z)).mean())((
        print('X:')
        print(rmseX)
        print('Y:')
        print(rmseY)
        print('Z')
        print(rmseZ)
        rmseX_pub.publish(rmseX)
        rmseY_pub.publish(rmseY)
        rmseZ_pub.publish(rmseZ)
        rate.sleep()
