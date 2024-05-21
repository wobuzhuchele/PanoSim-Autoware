# encoding: utf-8
from Init_ROS2 import *
ros2 = winros2()
ros2.init()
import rclpy
from rclpy.qos import *
from rclpy.node import Node
from std_msgs.msg import Header
from unique_identifier_msgs.msg import UUID
from autoware_auto_vehicle_msgs.msg import VelocityReport,SteeringReport,ControlModeReport,GearReport,TurnIndicatorsReport
from builtin_interfaces.msg import Time
from tier4_vehicle_msgs.msg import ActuationStatusStamped,ActuationStatus
from RecvSimClock import SimClock
from DataInterfacePython import *
import math

ego_extra_format = "time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d"
key_ego_extra = "ego_extra"
ego_format = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d" 
key_ego = "ego"
ego_all_format = "time@i,295@[,data@d"
key_ego_all = "ego_all"
ego_control_format = "time@i,valid@b,throttle@d,brake@d,steer@d,mode@i,gear@i"
key_ego_control = "ego_control"
ego_light_format = "time@i,variable@d"
key_ego_light = "global.9"
ego_mode_format = "time@i,valid@b,mode@i"
key_ego_mode = "ego_control.mode"
ego_steer_format = "time@i,valid@b,steer@d"
key_ego_steer = "ego_control.steer"

rad2degre_coefficient = 180/math.pi
transmission_ratio = 17.33563215

class VehicleStatus(Node):
    def __init__(self):
        super().__init__('vehiclestatus_publisher')
        # qos = qos_profile_sensor_data
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )
    
        self.publisher_velocity_status = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status',qos)
        self.publisher_steering_status = self.create_publisher(SteeringReport, '/vehicle/status/steering_status',qos)
        self.publisher_control_mode_status = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode',qos)
        self.publisher_gear_status = self.create_publisher(GearReport, '/vehicle/status/gear_status',qos)
        self.publisher_turn_indicators_status = self.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status',qos)
        self.publisher_actuation_status = self.create_publisher(ActuationStatusStamped, '/vehicle/status/actuation_status',qos)
        
        #create PanoSim data bus
        self.bus_ego_extra = BusAccessor(0,key_ego_extra,ego_extra_format)
        self.bus_ego = BusAccessor(0,key_ego,ego_format)
        self.bus_ego_all = BusAccessor(0,key_ego_all,ego_all_format)
        self.bus_ego_control = BusAccessor(0,key_ego_control,ego_control_format)
        self.bus_ego_light = BusAccessor(0,key_ego_light,ego_light_format)
        self.bus_ego_mode = BusAccessor(0,key_ego_mode,ego_mode_format)
        self.bus_ego_steer = BusAccessor(0,key_ego_steer,ego_steer_format)

        #frequency 
        frequency = 30
        timer_period = 1/frequency                                
        self.timer_velocity_status = self.create_timer(timer_period, self.velocity_status_callback)
        self.timer_steering_status = self.create_timer(timer_period, self.steering_status_callback)
        self.timer_actuation_status = self.create_timer(timer_period, self.actuation_status_callback)
        self.timer_control_mode_status = self.create_timer(timer_period, self.control_mode_status_callback)
        self.timer_gear_status = self.create_timer(timer_period, self.gear_status_callback)
        self.timer_turn_indicators_status = self.create_timer(timer_period, self.turn_indicators_status_callback)
        
        self.simClock = SimClock('minimal_publisher')
        


    def velocity_status_callback(self):
        """
        publish velocity status message 
        """
        ts,VX,VY,VZ,AVx,AVy,AVz,Ax,Ay,Az,AAx,AAy,AAz = self.bus_ego_extra.readHeader()
        VelocityReport_= VelocityReport()
        VelocityReport_.header.frame_id = 'base_link'
        
        VelocityReport_.header.stamp = self.simClock.getClock()
        VelocityReport_.longitudinal_velocity = VX
        VelocityReport_.lateral_velocity = VY
        VelocityReport_.heading_rate = AVz
        self.publisher_velocity_status.publish(VelocityReport_)

    def steering_status_callback(self):
        """
        publish steering status message 
        """
        _,_,steer = self.bus_ego_steer.readHeader()
        steer = steer/rad2degre_coefficient/17.33563215
        SteeringReport_ =SteeringReport()
    
        SteeringReport_.stamp = self.simClock.getClock()
        if math.isnan(steer):
            steer = 0
        SteeringReport_.steering_tire_angle = float(steer)
        self.publisher_steering_status.publish(SteeringReport_)
    


    def actuation_status_callback(self):
        """
        publish actuation status message 
        """
        throttle = self.bus_ego_all.readBody(102)[0]
        brake = self.bus_ego_all.readBody(19)[0] /15000000
        steer = self.bus_ego_steer.readHeader()[2]
        steer = steer/rad2degre_coefficient/17.33563215
        if math.isnan(steer):
            steer = 0
        ActuationStatusStamped_ = ActuationStatusStamped()
        ActuationStatusStamped_.header.frame_id = "base_link"
        ActuationStatusStamped_.header.stamp = self.simClock.getClock()
        ActuationStatus_ = ActuationStatus()
        ActuationStatus_.accel_status = throttle
        ActuationStatus_.brake_status = brake
        ActuationStatus_.steer_status = steer
        ActuationStatusStamped_.status = ActuationStatus_
        self.publisher_actuation_status.publish(ActuationStatusStamped_)



    def control_mode_status_callback(self):
        """
        publish control mode status message 
        """
        ControlModeReport_ = ControlModeReport()
        ControlModeReport_.stamp = self.simClock.getClock()
        ControlModeReport_.mode = 1
        self.publisher_control_mode_status.publish(ControlModeReport_)

    def gear_status_callback(self):
        """
        publish control gear status message 
        """
        autoware_gear = 22
        _,_,mode = self.bus_ego_mode.readHeader()
        if mode == -1:
            autoware_gear= 20
        elif mode == 0:
            autoware_gear = 22
        elif mode == 1:
            autoware_gear = 2   
        elif mode == 5:
            autoware_gear = 2 

        GearReport_ = GearReport()
        GearReport_.stamp = self.simClock.getClock()
        GearReport_.report = autoware_gear
        self.publisher_gear_status.publish(GearReport_)

    def turn_indicators_status_callback(self):
        """
        publish turn indicators status message 
        """
        report_ = 1
        _,value = self.bus_ego_light.readHeader()
        if value == 32:
            report_ = 3
        elif value == 16:
            report_ = 2
        elif value == 0:
            report_ = 1
     
        TurnIndicatorsReport_ = TurnIndicatorsReport()
        TurnIndicatorsReport_.stamp = self.simClock.getClock()
        TurnIndicatorsReport_.report = report_  
        self.publisher_turn_indicators_status.publish(TurnIndicatorsReport_)     
            
         
def main(args=None):
    rclpy.init(args=args)
    VehicleStatus_publisher = VehicleStatus()
   
    rclpy.spin(VehicleStatus_publisher)
    VehicleStatus_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


