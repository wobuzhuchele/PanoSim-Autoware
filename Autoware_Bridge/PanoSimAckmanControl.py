# encoding: utf-8
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.




from Init_ROS2 import *
ros2 = winros2()
ros2.init()
import rclpy
from rclpy.qos import *

from rclpy.node import Node
from DataInterfacePython import *
from autoware_auto_control_msgs.msg import AckermannControlCommand
from simple_pid import PID  
import ego_control_physics as phys
from rclpy.node import Node
import numpy
from RecvSimClock import SimClock

ego_format = "time@i,x@d,y@d,z@d,yaw@d,pitch@d,roll@d,speed@d" 
key_ego = "ego"
ego_extra_format = "time@i,VX@d,VY@d,VZ@d,AVx@d,AVy@d,AVz@d,Ax@d,Ay@d,Az@d,AAx@d,AAy@d,AAz@d" 
key_ego_extra = "ego_extra"
ego_throttle_format = "time@i,valid@b,throttle@d"
key_ego_throttle = "ego_control.throttle"
ego_brake_format = "time@i,valid@b,brake@d"
key_ego_brake = "ego_control.brake"
ego_steer_format = "time@i,valid@b,steer@d"
key_ego_steer = "ego_control.steer"
ego_mode_format = "time@i,valid@b,mode@i"
key_ego_mode = "ego_control.mode"
ego_gear_format = "time@i,valid@b,gear@i"
key_ego_gear = "ego_control.gear"

class EgoVehicleControlTarget:
    def __init__(self) -> None:
        self.steering_angle = 0.
        self.speed = 0.
        self.speed_abs = 0.
        self.accel = 0.
        self.jerk = 0.

class EgoVehicleControlCurrent:
    def __init__(self) -> None:
        self.time_sec = 0.
        self.speed = 0.
        self.speed_abs = 0.
        self.accel = 0.

class EgoVehicleControlStatus:
    def __init__(self) -> None:
        self.status = 'n/a'
        self.speed_control_activation_count = 0
        self.speed_control_accel_delta = 0.
        self.speed_control_accel_target = 0.
        self.accel_control_pedal_delta = 0.
        self.accel_control_pedal_target = 0.
        self.brake_upper_border = 0.
        self.throttle_lower_border = 0.


class EgoVehicleControlInfo:
    def __init__(self) -> None:
        self.status = EgoVehicleControlStatus()
        self.target = EgoVehicleControlTarget()
        self.current = EgoVehicleControlCurrent()
        self.output = EgoVehicleControl()
        self.restrictions = EgoVehicleControlMaxima()

class EgoVehicleControlMaxima:
    def __init__(self) -> None:
        self.max_steering_angle = 0.7
        self.max_speed = 50/3.6
        self.max_accel = 3.
        self.max_decel = 8.
        self.min_accel = 1.
        self.max_pedal = 0.
    

class EgoVehicleStatus:
    def __init__(self) -> None:
        self.velocity = 0.
        self.acceleration = 0.
        self.control = EgoVehicleControl()

class EgoVehicleControl:
    def __init__(self) -> None:
        self.throttle = 0.
        self.steer = 0.
        self.brake = 0.
        self.hand_brake = False
        self.reverse = False
        self.gear = None
        self.manual_gear_shift = False

class EgoVehicleInfo:
    def __init__(self) -> None:
        self.max_rpm = 0.
        self.moi = 0.
        self.damping_rate_full_throttle = 0.
        self.damping_rate_zero_throttle_clutch_engaged = 0.
        self.damping_rate_zero_throttle_clutch_disengaged = 0.
        self.use_gear_autobox = False
        self.gear_switch_time = 0
        self.mass = 1416.0
        self.drag_coefficient = 0
        self.wheel = EgoVehicleInfoWheel()

class EgoVehicleInfoWheel:
    def __init__(self) -> None:
        self.tire_friction = 0.
        self.damping_rate = 0.
        self.max_steer_angle = 0.7
        self.radius = 0.
        self.max_brake_torque = 0.
        self.max_handbrake_torque = False
 


class PanoSimAckermannControl(Node):
    def __init__(self):
        super().__init__('PanoSimAckermannControl_node')
        
        # Used for timestamp synchronization
        self.simClock = SimClock()

        #create PID controller 
        self.speed_controller = PID(Kp=0.1,Ki=0.,Kd=0.0,sample_time=0.05,output_limits=(-1., 1.))
        self.accel_controller = PID(Kp=0.1,Ki=0.,Kd=0.00,sample_time=0.05,output_limits=(-1, 1))
        self.control_loop_rate = 0.05
        self.info = EgoVehicleControlInfo()

        self.vehicle_status = EgoVehicleStatus()
        self.vehicle_info = EgoVehicleInfo()
        self.info = EgoVehicleControlInfo()

        self.vehicle_info_updated(self.vehicle_info)

        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.

        # current values
        self.info.current.time_sec = self.get_time()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.

        # control values
        self.info.status.status = 'n/a'
        self.info.status.speed_control_activation_count = 0
        self.info.status.speed_control_accel_delta = 0.
        self.info.status.speed_control_accel_target = 0.
        self.info.status.accel_control_pedal_delta = 0.
        self.info.status.accel_control_pedal_target = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.

        # Create the PanoSim data bus
        self.bus_ego = BusAccessor(0,key_ego,ego_format)
        self.bus_ego_extra = BusAccessor(0,key_ego_extra,ego_extra_format)
        self.bus_ego_throttle = BusAccessor(0,key_ego_throttle,ego_throttle_format)
        self.bus_ego_brake = BusAccessor(0,key_ego_brake,ego_brake_format)
        self.bus_ego_steer = BusAccessor(0,key_ego_steer,ego_steer_format)
        self.bus_ego_mode = BusAccessor(0,key_ego_mode,ego_mode_format)
        self.bus_ego_gear = BusAccessor(0,key_ego_gear,ego_gear_format)
        self.ts = self.bus_ego.readHeader()[0]

        # Subscribe to the control topic
        self.control_subscriber = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.ackermann_command_updated, 10)
        self.timer = self.create_timer(self.control_loop_rate, self.run)

        

    def ackermann_command_updated(self,ros_ackermann_drive):
        """
        set target values
        """
        self.last_ackermann_msg_received_sec = self.get_time()
        self.set_target_steering_angle(ros_ackermann_drive.lateral.steering_tire_angle)
        self.set_target_speed(ros_ackermann_drive.longitudinal.speed)
        self.set_target_accel(ros_ackermann_drive.longitudinal.acceleration)
        self.set_target_jerk(ros_ackermann_drive.longitudinal.jerk)
    
    def set_target_steering_angle(self, target_steering_angle):
        """
        set target steering angle (rad)
        """
        self.info.target.steering_angle = target_steering_angle 
        if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
            self.info.target.steering_angle = numpy.clip(self.info.target.steering_angle,-self.info.restrictions.max_steering_angle,\
                self.info.restrictions.max_steering_angle)
            
    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        epsilon = 0.00001
        # if speed is set to zero, then use max decel value
        if self.info.target.speed_abs < epsilon:
            self.info.target.accel = -self.info.restrictions.max_decel
        else:
            self.info.target.accel = numpy.clip(target_accel, -self.info.restrictions.max_decel, self.info.restrictions.max_accel)
    
    def set_target_jerk(self, target_jerk):
        """
        set target accel
        """
        self.info.target.jerk = target_jerk


    def vehicle_info_updated(self, vehicle_info):
        # set target values
        self.vehicle_info = vehicle_info
        # calculate restrictions
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(self.vehicle_info)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(self.vehicle_info)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(self.vehicle_info)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(self.vehicle_info)
        self.info.restrictions.min_accel = 1.
        self.info.restrictions.max_pedal = min(self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def get_time(self):
        """
        set current SimTime
        """
        return self.simClock.getClock()

    def run(self):
        """
        update current values and perform a vehicle control cycle
        """
        self.update_current_values()
        self.vehicle_control_cycle()

    def update_current_values(self):
        current_time_sec = self.get_time()
        current_speed = self.bus_ego.readHeader()[7]
        self.info.current.accel = self.bus_ego_extra.readHeader()[7]
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and write control signals into the PanoSim data bus
        """
        self.control_steering()
        self.control_stop_and_reverse()
        if self.bus_ego_mode != -1:
            self.run_speed_control_loop()
            self.run_accel_control_loop()
            self.update_drive_vehicle_control_command()
        else:
            self.info.output.brake = 0.0
            self.info.output.throttle = 0.0
            self.bus_ego_throttle.writeHeader(*(self.ts,1,self.info.output.throttle))
            self.bus_ego_brake.writeHeader(*(self.ts,1,self.info.output.brake*8))


    def control_steering(self):
        """
        Basic steering control
        """
        self.ts = self.bus_ego.readHeader()[0]
        self.info.output.steer = self.info.target.steering_angle
        steer = self.info.output.steer*57.3*17.33563215 /1.5
        self.bus_ego_steer.writeHeader(*(self.ts,1,steer))
        

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """
        standing_still_epsilon = 0.1
        full_stop_epsilon = 0.00001
        self.info.output.hand_brake = False
        if self.info.current.speed_abs < standing_still_epsilon:
            self.info.status.status = "standing"
            if self.info.target.speed < 0:
                if not self.info.output.reverse:
                    self.info.output.reverse = True
                    self.bus_ego_mode.writeHeader(*self.ts,1,-1)
                    self.bus_ego_gear.writeHeader(*self.ts,1,1)
            elif self.info.target.speed > 0:
                if self.info.output.reverse:
                    self.info.output.reverse = False
                    self.bus_ego_mode.writeHeader(*self.ts,1,1)
                    self.bus_ego_gear.writeHeader(*self.ts,1,1)
            if self.info.target.speed_abs < full_stop_epsilon:
                self.info.status.status = "full stop"
                self.info.status.speed_control_accel_target = 0.
                self.info.status.accel_control_pedal_target = 0.
                self.set_target_speed(0.)
                self.info.output.hand_brake = True
                self.info.output.brake = 1.0
                self.info.output.throttle = 0.0

        elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
            self.set_target_speed(0.)

    def run_speed_control_loop(self):
        """
        Run the PID control loop for the speed
        """
        epsilon = 0.00001
        target_accel_abs = abs(self.info.target.accel)
        if target_accel_abs < self.info.restrictions.min_accel:
            if self.info.status.speed_control_activation_count < 5:
                self.info.status.speed_control_activation_count += 1
        else:
            if self.info.status.speed_control_activation_count > 0:
                self.info.status.speed_control_activation_count -= 1
        self.speed_controller.auto_mode = self.info.status.speed_control_activation_count >= 5

        if self.speed_controller.auto_mode:
            self.speed_controller.setpoint = self.info.target.speed_abs
            self.info.status.speed_control_accel_delta = float(self.speed_controller(self.info.current.speed_abs))
            # clipping borders
            clipping_lower_border = -target_accel_abs
            clipping_upper_border = target_accel_abs
            # per definition of ackermann drive: if zero, then use max value
            if target_accel_abs < epsilon:
                clipping_lower_border = -self.info.restrictions.max_decel
                clipping_upper_border = self.info.restrictions.max_accel
            self.info.status.speed_control_accel_target = numpy.clip(self.info.status.speed_control_accel_target +self.info.status.speed_control_accel_delta,\
                clipping_lower_border, clipping_upper_border)
        else:
            self.info.status.speed_control_accel_delta = 0.
            self.info.status.speed_control_accel_target = self.info.target.accel



    def run_accel_control_loop(self):
        """
        Run the PID control loop for the acceleration
        """
        self.accel_controller.setpoint = self.info.status.speed_control_accel_target
        self.info.status.accel_control_pedal_delta = float(self.accel_controller(self.info.current.accel))
        self.info.status.accel_control_pedal_target = numpy.clip(self.info.status.accel_control_pedal_target +self.info.status.accel_control_pedal_delta,\
            -self.info.restrictions.max_pedal, self.info.restrictions.max_pedal)

    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """
        self.info.status.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(self.info.current.speed,self.vehicle_info)
        self.info.status.brake_upper_border = self.info.status.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.vehicle_info)
        
        if self.info.status.accel_control_pedal_target > self.info.status.throttle_lower_border:
            self.info.status.status = "accelerating"
            self.info.output.brake = 0.0
            self.info.output.throttle = ((self.info.status.accel_control_pedal_target -self.info.status.throttle_lower_border) / abs(self.info.restrictions.max_pedal))

        else: 
            self.info.status.status = "braking"
            self.info.output.brake = ((self.info.status.brake_upper_border -self.info.status.accel_control_pedal_target) / abs(self.info.restrictions.max_pedal))
            self.info.output.throttle = 0.0
        self.info.output.brake = numpy.clip(self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(self.info.output.throttle, 0., 1.)
        self.bus_ego_throttle.writeHeader(*(self.ts,1,self.info.output.throttle))
        self.bus_ego_brake.writeHeader(*(self.ts,1,self.info.output.brake*8))



def main(args=None):
    rclpy.init(args=args)
    panosim_ackermann_control = PanoSimAckermannControl()
    rclpy.spin(panosim_ackermann_control)
    panosim_ackermann_control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
