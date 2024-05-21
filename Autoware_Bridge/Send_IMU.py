# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()
import rclpy
from rclpy.qos import *

from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from DataInterfacePython import *
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time

from RecvSimClock import SimClock

import math

imu_format = "Timestamp@i,ACC_X@d,ACC_Y@d,ACC_Z@d,Gyro_X@d,Gyro_Y@d,Gyro_Z@d,Yaw@d,Pitch@d,Roll@d" 
imu_key = "IMU.0"

class SendIMU(Node):
    def __init__(self):
        super().__init__('send_imu')
        # qos = qos_profile_system_default
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
    
        self.publisher_ = self.create_publisher(Imu, '/sensing/imu/tamagawa/imu_raw',qos)

        # create PanoSim data bus
        self.bus_imu = BusAccessor(0,imu_key,imu_format)
        # frequency
        frequency = 30
        timer_period = 1/frequency        

        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Used for timestamp synchronization
        self.simClock = SimClock()

    def convert_angle(self,angle):
        """
        convert angle into (-pi pi]
        """
        while True:
            if angle >= math.pi:
                angle -= math.pi
            elif angle < -math.pi:
                 angle += math.pi
            else:
                break
        return angle


    def timer_callback(self):
        #publish imu messages
        _,ACC_X,ACC_Y,ACC_Z,Gyro_X,Gyro_Y,Gyro_Z,Yaw,Pitch,Roll = self.bus_imu.readHeader()
        Yaw = self.convert_angle(Yaw)
        Pitch = self.convert_angle(Pitch)
        Roll = self.convert_angle(Roll)
        
        header_ = Header()
        header_.frame_id = 'imu'
        header_.stamp = self.simClock.getClock()

        Imu_ = Imu()
        Imu_.header = header_
        Imu_.linear_acceleration.x = -ACC_X
        Imu_.linear_acceleration.y = ACC_Y
        Imu_.linear_acceleration.z = -ACC_Z

        Imu_.angular_velocity.x = -Gyro_X
        Imu_.angular_velocity.y = Gyro_Y
        Imu_.angular_velocity.z = -Gyro_Z
        print(Imu_.angular_velocity)
     
        Imu_.orientation.x = 0.0
        Imu_.orientation.y = 0.0
        Imu_.orientation.z = 0.0
        Imu_.orientation.w = 1.0

        Imu_.header = header_
        self.publisher_.publish(Imu_)
        self.get_logger().info('Imu_publish' )
 
    
def main(args=None):
    rclpy.init(args=args)
    send_imu= SendIMU()
    rclpy.spin(send_imu)
    send_imu.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
