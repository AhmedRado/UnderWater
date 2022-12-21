#!/usr/bin/env python
import tabnanny
import rospy
from dynamic_reconfigure.server import Server
from awesome_package.cfg import PidControlConfig
import numpy as np

from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64, Float32
from nav_msgs.msg import Odometry


import threading
import  math


depth_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)
horizontal_pub = rospy.Publisher("lateral_thrust", Float64, queue_size=1)
lateral_pub = rospy.Publisher("thrust", Float64, queue_size=1)


estimated_x = Float64()
estimated_y = Float64()
estimated_z = Float64()

desired_msg_x= Float64()
desired_msg_y= Float64()
desired_msg_z= Float64()
PID_Control = Float64()
disturbance_msg = Float64()
disturbance_flag = False


global previous_error_x 
global previous_error_y 
global previous_error_z 
previous_error_x = 0
previous_error_y = 0
previous_error_z = 0

global true_x
global true_y
global true_z 

true_x =0
true_y =0
true_z =0
    

# def recieved_reference(pressure_msg):    
#     depth_msg.data = pressure_msg.data
  
def recieved_depth_setPoint(point_msg):    
    desired_msg_z.data = point_msg.data

def recieved_horizontal_setPoint(point_msg):
    desired_msg_x.data = point_msg.data
    print("desired_x:")
    print(desired_msg_x.data)

def recieved_lateral_setPoint(point_msg):
    desired_msg_y.data = point_msg.data

def recieved_KalmanX(msg):
    estimated_x.data = msg.data

def recieved_KalmanY(msg):
    estimated_y.data = msg.data    

def recieved_KalmanZ(msg):
    estimated_z.data = msg.data       
      
error_pid_x= Float64()
error_pid_y= Float64()
error_pid_z= Float64()


def pid_x(p_gain, i_gain, d_gain, disturbance):
    
    global previous_error_x 

    sampling_time = 1/7.0
    
    cumError = 0
    
    error_pid_x.data = (desired_msg_x.data - estimated_x.data)

    if((desired_msg_x.data)<2 and (estimated_x.data) < 2 and (desired_msg_x.data)>0 and (estimated_x.data) >0):
        
        cumError   = cumError+ (error_pid_x.data*sampling_time*i_gain)

        rateError = ((error_pid_x.data-previous_error_x)/(sampling_time)) 

        if disturbance:
                    
            disturb = 1
        else:
            disturb = 0.0      

        PID_Control.data = -((p_gain * error_pid_x.data) + (cumError) + (d_gain * rateError) + disturbance_msg.data)
        print("PID: ")
        print(PID_Control.data)
        previous_error_x = error_pid_x.data
        horizontal_pub.publish(PID_Control) 
        # error_pub.publish(error) 
        # disturbance_pub.publish(disturbance_msg) 
             
    else:
                
        horizontal_pub.publish(0) 
        # error_pub.publish(error) 
        # disturbance_pub.publish(disturbance_msg) 
             


def pid_y(p_gain, i_gain, d_gain, disturbance):


    global previous_error_y 

    global true_x
    global true_y
    global true_z 

  
    sampling_time = 1/50.0
    previous_error = 0
    cumError = 0
    
    error_pid_y.data = (desired_msg_y.data - estimated_y.data)

    if((desired_msg_y.data)<4 and (estimated_y.data) < 4 and (desired_msg_y.data)>0 and (estimated_y.data)>0):
        
        cumError = cumError+ (error_pid_y.data*sampling_time*i_gain)

        rateError = ((error_pid_y.data-previous_error_y)/(sampling_time)) 

        if disturbance:
                    
            disturb = 1
        else:
            disturb = 0.0      

        PID_Control.data = (p_gain * error_pid_y.data) + (cumError) + (d_gain * rateError) + disturbance_msg.data
        previous_error_y = error_pid_y.data
        lateral_pub.publish(PID_Control) 
        # error_pub.publish(error) 
        # disturbance_pub.publish(disturbance_msg) 
             
    else:
                
        lateral_pub.publish(0) 
        # error_pub.publish(error) 
        # disturbance_pub.publish(disturbance_msg) 
             
def pid_z(p_gain, i_gain, d_gain, disturbance):


    global previous_error_z 

    sampling_time = 1/50.0
    previous_error = 0
    cumError = 0
    

    error_pid_z.data = (desired_msg_z.data -  estimated_z.data)

    if((desired_msg_z.data) < -0.1 and (estimated_z.data)< -0.1 and (desired_msg_z.data) > -0.8 and (estimated_z.data)>-0.8):
        
        cumError   = cumError+ (error_pid_z.data*sampling_time*i_gain)

        rateError = ((error_pid_z.data-previous_error_z)/(sampling_time)) 

        if disturbance:
                    
            disturb = 1
        else:
            disturb = 0.0      

        PID_Control.data = (p_gain * error_pid_z.data) + (cumError) + (d_gain * rateError) + disturbance_msg.data
        previous_error_z = error_pid_z.data
        depth_pub.publish(PID_Control) 
        # error_pub.publish(error) 
        # disturbance_pub.publish(disturbance_msg) 
             
    else:
                
        depth_pub.publish(0) 
        # error_pub.publish(error) 
        # disturbance_pub.publish(disturbance_msg)
def ground_truth_callback(groud_msg):

        global true_x
        global true_y
        global true_z 

        true_x = groud_msg.pose.pose.position.x
        print("actual_x:")
        print(true_x)

        true_y = groud_msg.pose.pose.position.y

        true_z = groud_msg.pose.pose.position.z
  
class MyControlNode():

     
     
     def __init__(self):
         
         self.data_lock = threading.RLock()
         # the assigned values do not matter. They get overwritten by
         # dynamic_reconfigure as soon as the dynamic_reconfigure server is
         # created.
         self.p_gain_x = 0.0
         self.i_gain_x = 0.0
         self.d_gain_x = 0.0

         self.p_gain_y = 0.0
         self.i_gain_y = 0.0
         self.d_gain_y = 0.0        

         self.p_gain_z = 0.0
         self.i_gain_z = 0.0
         self.d_gain_z = 0.0      


         self.disturbance = 0.0
         self.filtered = 0.0
 
         self.dyn_server = Server(PidControlConfig, self.on_pid_dyn_reconfigure)
 
     def on_pid_dyn_reconfigure(self, config, level):
         # the config parameters are provided as dictionary. The keys are the
         # parameter names we specified in cfg/PidControl.cfg
 
         # use data_lock to avoid parallel modifications of the variables
         # from different threads (here the main thread running the loop in the
         # run() method and the thread runing the dynamic_reconfigure callback).
         with self.data_lock:
             self.p_gain_x = config["p_gain_x"]
             self.i_gain_x = config["i_gain_x"]
             self.d_gain_x = config["d_gain_x"]

             self.p_gain_y = config["p_gain_y"]
             self.i_gain_y = config["i_gain_y"]
             self.d_gain_y = config["d_gain_y"]
             
             self.p_gain_z = config["p_gain_z"]
             self.i_gain_z = config["i_gain_z"]
             self.d_gain_z = config["d_gain_z"]


             self.disturbance = config["disturbance"]
             self.filtered = config["filtered"]
         return config

     def run(self):
         
         r = rospy.Rate(7)
        #  cumError = Float64()
        #  t_ellapsed =0  
        #  t_current= 0
        #  error = Float64()
        #  previous_error = 0
        #  previous_filteredError = 0
         
         
         while not rospy.is_shutdown():
             # use data_lock to avoid parallel modifications of the variables
             # from different threads (here the main thread running this loop
             # and the thread runing the dynamic_reconfigure callback)
             with self.data_lock:
                #  print("p_gain_x: {}\ni_gain_x: {}\nd_gain_x: {}\np_gain_y: {}\ni_gain_y: {}\nd_gain_y: {}\np_gain_z: {}\ni_gain_z: {}\nd_gain_z: {}\ndisturbance: {}\nfiltered: {}".format(
                #            self.p_gain_x, self.i_gain_x, self.d_gain_x,self.p_gain_y, self.i_gain_y, self.d_gain_y,self.p_gain_z, self.i_gain_z, self.d_gain_z ,self.disturbance, self.filtered
                #           ))

                 pid_x(self.p_gain_x,self.i_gain_x,self.d_gain_x,self.disturbance)
                 pid_y(self.p_gain_y,self.i_gain_y,self.d_gain_y,self.disturbance)
                 pid_z(self.p_gain_z,self.i_gain_z,self.d_gain_z,self.disturbance)         

                 r.sleep()
 

 
if __name__ == "__main__":

    rospy.init_node("depth_controller")

    error_pub = rospy.Publisher("error", Float64, queue_size=1)

    filtered_error_pub = rospy.Publisher("filtered", Float64, queue_size=1)

    #cumError_pub = rospy.Publisher("cumError", Float64, queue_size=1)

    disturbance_pub = rospy.Publisher("disturbance", Float64, queue_size=1)
  

    depth_setpoint_sub = rospy.Subscriber("depth_setpoint", Float64,
                                     recieved_depth_setPoint)

    horizontal_setpoint_sub = rospy.Subscriber("horizontal_setpoint", Float64,
                                     recieved_horizontal_setPoint)  

    lateral_setpoint_sub = rospy.Subscriber("lateral_setpoint", Float64,
                                     recieved_lateral_setPoint) 

    KalmanX_sub = rospy.Subscriber("KalmanX_velocity_pressure", Float64,
                                     recieved_KalmanX)
    
    KalmanY_sub = rospy.Subscriber("KalmanY_velocity_pressure", Float64,
                                     recieved_KalmanY)

    KalmanZ_sub = rospy.Subscriber("KalmanZ_velocity_pressure", Float64,
                                     recieved_KalmanZ)

    ground_truth_sub = rospy.Subscriber("ground_truth/state",Odometry, ground_truth_callback)                                  



    node = MyControlNode()                                 

    node.run()

    rospy.spin()
