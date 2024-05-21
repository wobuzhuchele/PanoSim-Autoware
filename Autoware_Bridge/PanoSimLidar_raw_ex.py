# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()

import rclpy
from rclpy.qos import *
from rclpy.clock import Clock
from rclpy.clock import ROSClock
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2,PointField
from DataInterfacePython import *
from RecvSimClock import SimClock

# MaxCount = (Measurements per Rotation) * (Number of Beams)
MaxCount = (1440)*(16)

# PanoSim Bus name and data format required for creating a bus
lidar_format = "Timestamp@i,%d@[,x@f,y@f,z@f,intensity@f" % int(MaxCount)
lidar_key = "SurroundLidarPointCloudSensor.0"

class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher_ex')
        # qos = qos_profile_sensor_data
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
    
        self.publisher_ = self.create_publisher(PointCloud2, '/points_raw_ex',qos)
        
        # Create the PanoSim data bus
        self.bus_ = BusAccessor(0,lidar_key,lidar_format)
        
        # hz
        timer_period = 1/10   
                                      
        self.timer_raw = self.create_timer(timer_period, self.timer_callback_raw)
        
        # Used for timestamp synchronization
        self.simClock = SimClock()

    def timer_callback_raw(self):
        _,data_width = self.bus_.readHeader()
        if data_width == 0:
            return
        
        data_ = self.bus_.getBus()[8:]
        new_data = self.eliminate_center_point(data_)
        
        data_width = int(len(new_data)/16)
        header_ = Header()
        header_.frame_id="velodyne"
        header_.stamp = self.simClock.getClock()
        height_ = 1
        width_ = data_width
        fields_ = [
                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                    ]
        point_step_ = 16
        row_step_ = data_width * point_step_
      
        msg = PointCloud2(
            header=header_,
            height=height_, 
            width=width_,
            is_dense=False,
            is_bigendian=False,
            fields=fields_,
            point_step=point_step_,
            row_step=row_step_,
            data=new_data
        )
        
        self.publisher_.publish(msg)

    def eliminate_center_point(self,data_):
        new_data = b''
        
        for i in range(int(len(data_)/16)):
            if struct.unpack("<f",data_[12+i*16:16+i*16])[0] != 0.0 :
                new_data = new_data + data_[0+i*16:16+i*16]
                
        return new_data 

 
def main(args=None):
    rclpy.init(args=args)
    lidar_publisher = LidarPublisher()
   
    rclpy.spin(lidar_publisher)
    lidar_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()