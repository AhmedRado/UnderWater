#!/usr/bin/env python
import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32
 
 
def pressure_callback(pressure_msg, publisher):
    pascal_per_meter = 1.0e4
    # what kind of pressure data do we get? relative/absolute? What about
    # atmospheric pressure?
    depth = -(pressure_msg.fluid_pressure-103253) / pascal_per_meter
    depth_msg = Float32()
    depth_msg.data = depth
    publisher.publish(depth_msg)
 
 
def main():
    rospy.init_node("depth_estimator")
    depth_pub = rospy.Publisher("deep", Float32, queue_size=1)
    pressure_sub = rospy.Subscriber("pressure", FluidPressure,
                                     pressure_callback, depth_pub)

    
    r = rospy.Rate(50)

    while not rospy.is_shutdown():
        r.sleep() 
 
 
if __name__ == "__main__":
    main()

