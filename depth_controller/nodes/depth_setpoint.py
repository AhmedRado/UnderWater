#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math

class DepthSetpointNode():
    def __init__(self):
        rospy.init_node("depth_setpoint_publisher") 

        self.start_time = rospy.get_time()

        # change these parameters to adjust setpoint(s)
        # either constant setpoint (setpoint_1), or jumping between two setpoints
        self.constant = False # constant or square wave?
        self.constant_sin = False
        self.duration = 30.0  # in s
        
        self.setpoint_horizontal=1.6
        self.setpoint_lateral= 3
        self.setpoint_depth = -0.4

        self.setpoint_depth_pub = rospy.Publisher("depth_setpoint",
                                            Float64,
                                            queue_size=1)

        self.setpoint_horizontal_pub = rospy.Publisher("horizontal_setpoint",
                                            Float64,
                                            queue_size=1)

        self.setpoint_lateral_pub = rospy.Publisher("lateral_setpoint",
                                            Float64,
                                            queue_size=1)

    #def get_setpoint(self):
        # if self.constant:
        #     setpoint = self.setpoint_1
        # else:  #  square wave

            
        #     now = rospy.get_time()
        #     time = self.start_time - now
             
        #     if not self.constant_sin:
        #       i = time % (self.duration * 2)
        #       if i > (self.duration):
        #           setpoint = self.setpoint_1
        #       else:
        #           setpoint = self.setpoint_2
        #     else:
        #         setpoint  = (-0.25 * math.sin(now))-0.6
        # setpoint= [self.setpoint_horizontal, self.setpoint_lateral, self.setpoint_depth]
        # self.publish_setpoint(setpoint)

    def publish_setpoint_depth(self, setpoint_depth):
        msg_depth = Float64()
        msg_depth.data = setpoint_depth
        self.setpoint_depth_pub.publish(msg_depth)
    
    def publish_setpoint_horizontal(self, setpoint_horizontal):
        msg_horizontal = Float64()
        msg_horizontal.data = setpoint_horizontal
        self.setpoint_horizontal_pub.publish(msg_horizontal)

    def publish_setpoint_lateral(self, setpoint_lateral):
        msg_lateral = Float64()
        msg_lateral.data = setpoint_lateral
        self.setpoint_lateral_pub.publish(msg_lateral)

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            #self.get_setpoint()
            self.publish_setpoint_depth(self.setpoint_depth)
            self.publish_setpoint_horizontal(self.setpoint_horizontal)
            self.publish_setpoint_lateral(self.setpoint_lateral)
            rate.sleep()


def main():
    node = DepthSetpointNode()
    node.run()


if __name__ == "__main__":
    main()
