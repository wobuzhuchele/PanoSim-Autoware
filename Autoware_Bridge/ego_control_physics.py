#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#


"""
Tool functions to calculate vehicle physics
"""

import math



def get_vehicle_lay_off_engine_acceleration(vehicle_info):
    """
    Calculate the acceleration vehicle faces by the engine on lay off
    """
    return -get_engine_brake_force(vehicle_info) / get_vehicle_mass(vehicle_info)


def get_engine_brake_force(_):
    """
    Calculate the engine brake force of vehicle if the gas pedal would be layed off
    """
    return 500.0


def get_vehicle_mass(vehicle_info):
    """
    Get the mass of vehicle (defaults to 1416kg)
    """
    mass = 1416.0
    if vehicle_info.mass:
        mass = vehicle_info.mass

    return mass


def get_vehicle_driving_impedance_acceleration(speed,vehicle_info):
    """
    Calculate the acceleration vehicle faces by the driving impedance
    """

    rolling_resistance_force = get_rolling_resistance_force(vehicle_info)
    aerodynamic_drag_force = get_aerodynamic_drag_force(speed)
    deceleration = -(rolling_resistance_force + aerodynamic_drag_force ) / get_vehicle_mass(vehicle_info)

    return deceleration


def get_rolling_resistance_force(vehicle_info):
    """
    Calculate the rolling resistance force of vehicle
    """

    rolling_resistance_coefficient = 0.01
    normal_force = get_acceleration_of_gravity(vehicle_info)*get_vehicle_mass(vehicle_info)

    rolling_resistance_force = rolling_resistance_coefficient * normal_force

    return rolling_resistance_force





def get_acceleration_of_gravity(_):
    """
    Get the acceleration of gravity for vehicle
    """
    acceleration = 9.81

    return acceleration


def get_aerodynamic_drag_force(speed):
    """
    Calculate the aerodynamic drag force of vehicle
    """
    default_aerodynamic_drag_coefficient = 0.3  
    default_drag_reference_area = 2.578  
    drag_area = default_aerodynamic_drag_coefficient * default_drag_reference_area
    rho_air_25 = 1.206  
    speed_squared = speed* speed

    aerodynamic_drag_force = 0.5 * drag_area * rho_air_25 * speed_squared
    return aerodynamic_drag_force


def get_slope_force(vehicle_info, vehicle_status):
    """
    Calculate the force of vehicle faces when driving on a slope.
    """
    dummy_roll, pitch, dummy_yaw = quat2euler(
        [vehicle_status.orientation.w, vehicle_status.orientation.x,
         vehicle_status.orientation.y, vehicle_status.orientation.z])  # 四元数转欧拉角
    slope_force = get_acceleration_of_gravity(
        vehicle_info) * get_vehicle_mass(vehicle_info) * math.sin(-pitch)
    return slope_force


def get_vehicle_max_steering_angle(vehicle_info):
    """
    Get the maximum steering angle of vehicle
    """
    max_steering_angle = 0.7
    return max_steering_angle


def get_vehicle_max_speed(_):
    """
    Get the maximum speed of vehicle
    """
    max_speed = 50.0 / 3.6

    return max_speed


def get_vehicle_max_acceleration(_):
    """
    Get the maximum acceleration of vehicle
    default: 3.0 m/s^2: 0-100 km/h in 9.2 seconds
    """
    max_acceleration = 3.0

    return max_acceleration


def get_vehicle_max_deceleration(_):
    """
    Get the maximum deceleration of vehicle
    """
    max_deceleration = 8.0

    return max_deceleration
