#!/usr/bin/env python


from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import sys
import argparse  


def Drone_details():
   vehicle = connect('127.0.0.1:14550', baud=921600, wait_ready=True)
   while(True):
    time.sleep(1)
    # Get all vehicle attributes (state)
    print("\nGet all vehicle attribute values:")
    print(" Global Latitude: %s" % vehicle.location.global_frame.lat)
    print(" Global Longtitude: %s" % vehicle.location.global_frame.lon)
    print(" Global Altitude: %s" % vehicle.location.global_frame.lat)
    print(" Relative Altitude: %s m" % vehicle.location.global_relative_frame.alt)
    print(" Yaw Angle: %s" % vehicle.attitude.yaw)
    print(" Velocity: %s" % vehicle.velocity)
    print(" Battery: %s" % vehicle.battery)
    print(" Groundspeed: %s" % vehicle.groundspeed)    
    print(" Airspeed: %s" % vehicle.airspeed)    
    print(" Mode: %s" % vehicle.mode.name)    
    print(" Armed: %s" % vehicle.armed)   

    if vehicle.close() == True:
        break 


def Drone_exit():
    sys.exit()

if __name__ == '__main__':
    Drone_details()

