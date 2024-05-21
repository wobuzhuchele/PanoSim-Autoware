# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()

import rclpy
from rclpy.qos import *

from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from DataInterfacePython import *
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import time

from RecvSimClock import SimClock

import math

ego_format = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d" 
ego_key = "ego"
ego_extra_format = "time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d" 
ego_extra_key = "ego_extra"

# x offset
offset_x = 45.54
# y offset
offset_y = 20.07
# wheelbase
wheel = 2.578

class SendGNSS(Node):
    def __init__(self):
        super().__init__('send_imu')
        # qos = qos_profile_system_default
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
    
        self.publisher_PoseWithCovarianceStamped = self.create_publisher(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance',qos)
        self.publisher_PoseStamped = self.create_publisher(PoseStamped, '/sensing/gnss/pose',qos)

        # create PanoSim data bus
        self.bus_ego = BusAccessor(0,ego_format,ego_key)
        self.bus_ego_extra = BusAccessor(0,ego_extra_key,ego_extra_format)

        # frequency
        frequency = 5
        timer_period = 1/frequency

        self.timer_PoseWithCovarianceStamped = self.create_timer(timer_period, self.timer_callback_PoseWithCovarianceStamped)
        self.timer_PoseStamped = self.create_timer(timer_period, self.timer_callback_PoseStamped)

        # Used for timestamp synchronization
        self.simClock = SimClock()

    def timer_callback_PoseWithCovarianceStamped(self):
        # Calculate the vehicle location and publish PoseWithCovarianceStamped message
        ts, x, y, z, yaw, pitch, roll, speed = self.bus_ego.readHeader()
        header_ = Header()
        header_.frame_id = 'map'
        header_.stamp = self.simClock.getClock()
        GNSS_cov = PoseWithCovarianceStamped()
        GNSS_cov.pose.pose.position.x = x+offset_x - wheel*math.cos(yaw)
        GNSS_cov.pose.pose.position.y = y+offset_y - wheel*math.sin(yaw)
        GNSS_cov.pose.pose.position.z = z
        euler_angles = [roll, pitch, yaw] 
        rot_mat = Rotation.from_euler('xyz', euler_angles).as_matrix()
        quaternion = Rotation.from_matrix(rot_mat).as_quat()
        GNSS_cov.pose.pose.orientation.x = quaternion[0]
        GNSS_cov.pose.pose.orientation.y = quaternion[1]
        GNSS_cov.pose.pose.orientation.z = quaternion[2]
        GNSS_cov.pose.pose.orientation.w = quaternion[3]
        GNSS_cov.header = header_
        self.publisher_PoseWithCovarianceStamped.publish(GNSS_cov)
        self.get_logger().info('GNSS_cov_publish' )
    
    def timer_callback_PoseStamped(self):
        # Calculate the vehicle location and publish PoseWithCovarianceStamped message
        ts, x, y, z, yaw, pitch, roll, speed = self.bus_ego.readHeader()
        header_ = Header()
        header_.frame_id = 'map'

        header_.stamp = self.simClock.getClock()
        GNSS_Pose = PoseStamped()
        GNSS_Pose.pose.position.x = x+offset_x - wheel*math.cos(yaw)
        GNSS_Pose.pose.position.y = y+offset_y - wheel*math.sin(yaw)
        GNSS_Pose.pose.position.z = z
        euler_angles = [roll, pitch, yaw]
        rot_mat = Rotation.from_euler('xyz', euler_angles).as_matrix()
        quaternion = Rotation.from_matrix(rot_mat).as_quat()
        GNSS_Pose.pose.orientation.x = quaternion[0]
        GNSS_Pose.pose.orientation.y = quaternion[1]
        GNSS_Pose.pose.orientation.z = quaternion[2]
        GNSS_Pose.pose.orientation.w = quaternion[3]
        GNSS_Pose.header = header_
        self.publisher_PoseStamped.publish(GNSS_Pose)
 
    
def main(args=None):
    rclpy.init(args=args)
    send_gnss= SendGNSS()
    rclpy.spin(send_gnss)
    send_gnss.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
