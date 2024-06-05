# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()
import rclpy
from rclpy.qos import *
from rclpy.node import Node
from DataInterfacePython import *
from unique_identifier_msgs.msg import UUID
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand,GearCommand
import message_filters
from RecvSimClock import SimClock



ego_light_format = "time@i,variable@d"
key_ego_light = "global.9"
ego_gear_format = "time@i,valid@b,gear@i"
key_ego_gear = "ego_control.gear"
ego_mode_format = "time@i,valid@b,mode@i"
key_ego_mode = "ego_control.mode"
ego_format = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d" 
key_ego = "ego"


class EgoLight(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
        self.subscriber_mode = self.create_subscription(GearCommand, "/control/command/gear_cmd", self.check_mode_gear_data, 10)
        self.subscriber_turn_light = self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.check_light_data, 10)

        self.bus_ego_light = BusAccessor(0,key_ego_light,ego_light_format)
        self.bus_ego_gear = BusAccessor(0,key_ego_gear,ego_gear_format)
        self.bus_ego_mode = BusAccessor(0,key_ego_mode,ego_mode_format)
        self.bus_ego = BusAccessor(0,key_ego,ego_format)
        self.simClock = SimClock()



    def check_light_data(self,light_data):
        ts = self.bus_ego.readHeader()[0]
        if light_data.command == 3: #右转
            self.bus_ego_light.writeHeader(*(ts,32))
        elif light_data.command == 2:
             self.bus_ego_light.writeHeader(*(ts,16))
        elif light_data.command == 1:
            self.bus_ego_light.writeHeader(*(ts,0))
        else:
            self.bus_ego_light.writeHeader(*(ts,0))
    
    def check_mode_gear_data(self,gear_data):
        ts = self.bus_ego.readHeader()[0]
        if gear_data.command == 22:
            self.bus_ego_mode.writeHeader(*(ts,1,0))
            self.bus_ego_gear.writeHeader(*(ts,1,0))
        
        elif gear_data.command == 2:
            self.bus_ego_mode.writeHeader(*(ts,1,1))
            self.bus_ego_gear.writeHeader(*(ts,1,1)) 


        elif gear_data.command == 20:
            self.bus_ego_mode.writeHeader(*(ts,1,-1))
            self.bus_ego_gear.writeHeader(*(ts,1,1))    
        
        
def main(args=None):
    rclpy.init(args=args)
    ego_light = EgoLight()
    rclpy.spin(ego_light)
    ego_light.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()


