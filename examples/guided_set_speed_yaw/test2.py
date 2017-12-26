# """
#  Copyright 2015-2016, 3D Robotics.
# guided_set_speed_yaw.py: (Copter Only)
#
# This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.
#
# Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
# """
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

# Set up option parsing to get connection string
import argparse



#connection_string = '/dev/ttyACM0'
sitl = None

# Start SITL if no connection string specified

import dronekit_sitl

sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    # """
    # Arms vehicle and fly to aTargetAltitude.
    # """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


# Arm and take of to altitude of 5 meters
arm_and_takeoff(5)

#--------------------------------------------------------------------#
#----After taking-off we align the copter to the desired heading-----#
#--------------------------------------------------------------------#



def condition_yaw(heading, relative):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    # """
    # Move vehicle in direction based on specified velocity vectors.
    # """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

        # Set up velocity mappings
        # velocity_x > 0 => fly North
        # velocity_x < 0 => fly South
        # velocity_y > 0 => fly East
        # velocity_y < 0 => fly West
        # velocity_z < 0 => ascend
        # velocity_z > 0 => descend
        # SOUTH = -2
        # UP = -0.5  # NOTE: up is negative!
        #
        # # Fly south and up.
        # send_ned_velocity(SOUTH, 0, UP, DURATION)
angle = 0
condition_yaw(angle,0)

while angle <= 360:
    angle+=20
    vehicle.groundspeed = 2.5
    send_ned_velocity(5*math.cos(angle*math.pi/180),5*math.sin(angle*math.pi/180),0,1)
    time.sleep(5)
    condition_yaw(angle,0)

vehicle.mode = VehicleMode("RTL")

while vehicle.location.global_relative_frame.alt >= 0.1:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt <= 0.1:
        print("LANDED")
        break
    time.sleep(1)

vehicle.close()



print("Completed")