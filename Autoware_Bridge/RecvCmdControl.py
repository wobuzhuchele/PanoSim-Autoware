# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()
import rclpy
from rclpy.qos import *
import message_filters

from RecvSimClock import SimClock

from rclpy.node import Node
from DataInterfacePython import *
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from autoware_auto_control_msgs.msg import AckermannControlCommand

# PanoSim Bus name and data format required for creating a bus
ego_throttle_format = "time@i,valid@b,throttle@d"
key_ego_throttle = "ego_control.throttle"
ego_brake_format = "time@i,valid@b,brake@d"
key_ego_brake = "ego_control.brake"
ego_steer_format = "time@i,valid@b,steer@d"
key_ego_steer = "ego_control.steer"
ego_format = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d" 
key_ego = "ego"
ego_mode_format = "time@i,valid@b,mode@i"
key_ego_mode = "ego_control.mode"


class EgoControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Subscribe to the control topic
        self.subscriber_cmd_lon = message_filters.Subscriber(self, ActuationCommandStamped, '/control/command/actuation_cmd')
        self.subscriber_cmd_lat = message_filters.Subscriber(self, AckermannControlCommand, '/control/command/control_cmd')
        ts = message_filters.TimeSynchronizer([self.subscriber_cmd_lon, self.subscriber_cmd_lat], 1)
        ts.registerCallback(self.actuation_cmd_callback)

        # Create the PanoSim data bus
        self.bus_ego_throttle = BusAccessor(0,key_ego_throttle,ego_throttle_format)
        self.bus_ego_brake = BusAccessor(0,key_ego_brake,ego_brake_format)
        self.bus_ego_steer = BusAccessor(0,key_ego_steer,ego_steer_format)
        self.bus_ego = BusAccessor(0,key_ego,ego_format)
        self.bus_ego_mode = BusAccessor(0,key_ego_mode,ego_mode_format)

        # Used for timestamp synchronization
        self.simClock = SimClock()

    def actuation_cmd_callback(self,lon_data,lat_data):
        """
        Write control signals into the PanoSim data bus
        """
        throttle = lon_data.actuation.accel_cmd
        if self.bus_ego_mode.readHeader()[2] == -1:
            brake = lon_data.actuation.brake_cmd*7
        else:
            brake = lon_data.actuation.brake_cmd*8
        steer = lat_data.lateral.steering_tire_angle*57.3*17.33563215
        ts = self.bus_ego.readHeader()[0]

        self.bus_ego_throttle.writeHeader(*(ts,1,throttle))
        self.bus_ego_brake.writeHeader(*(ts,1,brake))
        self.bus_ego_steer.writeHeader(*(ts,1,steer))


     
         
def main(args=None):
    rclpy.init(args=args)
    ego_control = EgoControl() 
    rclpy.spin(ego_control)
    ego_control.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()


