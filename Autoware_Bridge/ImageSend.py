# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()
import rclpy
from rclpy.qos import *

import array

from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo,Image
from DataInterfacePython import *
from RecvSimClock import SimClock

# The height and width of the image output by the camera
Height = 400 
Width = 800

# PanoSim Bus name and data format required for creating a bus
camera_format = "time@i,%d@[,r@b,g@b,b@b" % (int(Width) * int(Height))
camera_key = "MonoCameraSensor.0"

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # qos = qos_profile_sensor_data
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
      
        self.publisher_image = self.create_publisher(Image, '/sensing/camera/traffic_light/image_raw',qos)
        self.publisher_camera = self.create_publisher(CameraInfo, '/sensing/camera/traffic_light/camera_info',qos)
      
        # Create the PanoSim data bus
        self.bus_ = BusAccessor(0,camera_key,camera_format)
        
        # Frequency
        Frequency = 30
        timer_period = 1/Frequency   
                                   
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Used for timestamp synchronization
        self.simClock = SimClock()


    def timer_callback(self):
        # The getBus() function returns the image data output from the camera sensor
        mono_data = self.bus_.getBus()[8:]
        
        header_ = Header()
        header_.stamp = self.simClock.getClock()
        header_.frame_id = 'camera'
        
        image_ = Image()
        image_.header = header_
        image_.height = int(Height)
        image_.width = int(Width)
        image_.encoding = 'rgb8'
        image_.is_bigendian = 1
        image_.step = image_.width * 3         
        image_.data=array.array('B',mono_data)
        
        CameraInfo_ = CameraInfo()
        CameraInfo_.header = header_
        CameraInfo_.height = int(Height)
        CameraInfo_.width = int(Width)
        CameraInfo_.distortion_model = 'plumb_bob'
        CameraInfo_.d = [0.0,0.0,0.0,0.0,0.0]
        cx = int(Width)/2
        cy = int(Height)/2
        fx = 500.0
        fy = 500.0
        CameraInfo_.k = [fx,0.0,cx,0.0,fy,cy,0.0,0.0,1.0]        
        CameraInfo_.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]        
        CameraInfo_.p = [fx,0.0,cx,0.0,0.0,fy,cy,0.0,0.0,0.0,1.0,0.0]   
        
        self.publisher_image.publish(image_)
        self.publisher_camera.publish(CameraInfo_)

 
def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
   
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
