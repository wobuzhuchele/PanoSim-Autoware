from Init_ROS2 import *
ros2 = winros2()
ros2.init()

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

from DataInterfacePython import *

class SimClock(Node):
    def __init__(self,name='clock'):
        super().__init__(name)
        self.format = "sec@i,nanosec@i" 
        self.key = "simtime"
        self.bus_ = BusAccessor(0,self.key,self.format)
        self.subscriber = self.create_subscription(Clock, "/clock", self.subCallback, 1)
        

    def subCallback(self,msg):
        self.bus_.writeHeader(*(msg.clock.sec,msg.clock.nanosec))
        
        
    def getClock(self):
        sec_, nanosec_ = self.bus_.readHeader()
        return Time(sec=sec_,nanosec=nanosec_)
        
def main(args=None):
    rclpy.init(args=args) 
    node = SimClock() 
    rclpy.spin(node) 
    rclpy.shutdown() 

if __name__ == "__main__": 
    main()