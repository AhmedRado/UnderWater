#!/usr/bin/env python
from distutils.log import error
import tabnanny
import rospy
from dynamic_reconfigure.server import Server
from awesome_package.cfg import PidControlConfig

from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64, Float32

import threading
import  math


depth_msg = Float32()
desired_msg = Float64()
PID_Control = Float64()
disturbance_msg = Float64()
disturbance_flag = False

def recieved_reference(pressure_msg):    
    depth_msg.data = pressure_msg.data
  
def recieved_setPoint(point_msg):    
    desired_msg.data = point_msg.data
                   
  
class MyControlNode():
     
     def __init__(self):
         
         self.data_lock = threading.RLock()
         # the assigned values do not matter. They get overwritten by
         # dynamic_reconfigure as soon as the dynamic_reconfigure server is
         # created.
         self.p_gain = 0.0
         self.i_gain = 0.0
         self.d_gain = 0.0        
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
             self.p_gain = config["p_gain"]
             self.i_gain = config["i_gain"]
             self.d_gain = config["d_gain"]
             self.disturbance = config["disturbance"]
             self.filtered = config["filtered"]
         return config
     def run(self):
         
         r = rospy.Rate(50)
         cumError = Float64()
         t_ellapsed =0  
         t_current= 0
         error = Float64()
         previous_error = 0
         previous_filteredError = 0
         
         
         while not rospy.is_shutdown():
             # use data_lock to avoid parallel modifications of the variables
             # from different threads (here the main thread running this loop
             # and the thread runing the dynamic_reconfigure callback)
             with self.data_lock:
                 print("p_gain: {}\ni_gain: {}\nd_gain: {}\ndisturbance: {}\nfiltered: {}".format(
                           self.p_gain, self.i_gain, self.d_gain, self.disturbance, self.filtered
                          ))

             t_current= rospy.get_time()

             

             
             sampling_time = 1/50
             #sampling_time = t_current - t_ellapsed
            
             error.data = (desired_msg.data - depth_msg.data)+ 0.1*math.sin(2*3.14*100*t_current) 
            
             if (self.filtered is True):

             
               current_filteredError = 0.5219*previous_filteredError + 0.2391 * error.data + 0.2391 * previous_error 
             else:
               current_filteredError = error.data     
             
    
             



             if(desired_msg.data< -0.1 and desired_msg.data>-0.8):

                
             
                cumError.data   = cumError.data+ (current_filteredError*sampling_time*self.i_gain)

                             
                rateError = ((current_filteredError-previous_filteredError)/(sampling_time)) 

                if self.disturbance:
                    disturbance_msg.data = 1
                else:
                    disturbance_msg.data = 0.0      

                PID_Control.data = (self.p_gain * current_filteredError) + (cumError.data) + ( self.d_gain * rateError) + disturbance_msg.data

                t_ellapsed = t_current  

                
                previous_filteredError = current_filteredError
                previous_error = error.data

                depth_pub.publish(PID_Control) 

                error_pub.publish(error) 

                filtered_error_pub.publish(current_filteredError) 

                disturbance_pub.publish(disturbance_msg) 
             
             else:
                
               
                depth_pub.publish(0) 
              
                error_pub.publish(error) 

                disturbance_pub.publish(disturbance_msg) 

                cumError_pub.publish(cumError)   

                cumError.data = 0
             
             r.sleep()
 

 
if __name__ == "__main__":

    rospy.init_node("depth_controller")
    
    depth_pub = rospy.Publisher("vertical_thrust", Float64, queue_size=1)

    error_pub = rospy.Publisher("error", Float64, queue_size=1)

    filtered_error_pub = rospy.Publisher("filtered", Float64, queue_size=1)

    cumError_pub = rospy.Publisher("cumError", Float64, queue_size=1)

    disturbance_pub = rospy.Publisher("disturbance", Float64, queue_size=1)
  
    pressure_sub = rospy.Subscriber("deep", Float32,
                                     recieved_reference)

    setpoint_sub = rospy.Subscriber("depth_setpoint", Float64,
                                     recieved_setPoint)

    node = MyControlNode()                                 

    node.run()

    rospy.spin()
