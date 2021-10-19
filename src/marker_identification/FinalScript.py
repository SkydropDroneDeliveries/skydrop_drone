from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time
import argparse  

import numpy as np
import cv2
from cv2 import aruco

#- Importing Tkinter: sudo apt-get install python-tk
import Tkinter as tk



cap = cv2.VideoCapture(0)

#-- Setup the commanded flying speed
gnd_speed = 1 # [m/s]

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=921600, wait_ready=True)

# Get some vehicle attributes (state)
print ("Get some vehicle attribute values:")
print ("  GPS: %s" % vehicle.gps_0)
print ("  Battery: %s" % vehicle.battery)
print ("  Last Heartbeat: %s" % vehicle.last_heartbeat)
print ("  Is Armable?: %s" % vehicle.is_armable)
print ("  System status: %s" % vehicle.system_status.state)
print ("  Mode: %s" % vehicle.mode.name )   # settable


# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)
        
  print "Arming motors"
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

  print "Taking off!"
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print " Altitude: ", vehicle.location.global_relative_frame.alt 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print "Reached target altitude"
      break
    time.sleep(1)

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

    
#-- Key event function
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
        elif event.keysym == 'u':
            print("Going UP")
            set_velocity_body(vehicle, gnd_speed, 0, 0)
            
    else: #-- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(vehicle,-gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)
            
    
    



#Takeoff height in meters
arm_and_takeoff(8)
print("Take off complete")
# Hover for 10 seconds
time.sleep(2)

#- Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
 
while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    for rejectedPolygons in rejectedImgPoints:
         for points in rejectedPolygons:
            cv2.line(frame_markers, tuple(points[0]), tuple(points[1]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[2]), tuple(points[1]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[2]), tuple(points[3]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[0]), tuple(points[3]), [100, 0, 100])


  #  cv2.imshow('frame_marker',frame_markers)
  #  "53" is the index id of that particular Aruco Marker ( tetsed with 4X4 matrix marker)
    if (ids==53):
       print("Now let's land")
       # Get some vehicle attributes (state)
       print ("Get some vehicle attribute values LANDING:")
       print ("  GPS: %s" % vehicle.gps_0)
       print ("  Battery: %s" % vehicle.battery)
       print ("  Last Heartbeat: %s" % vehicle.last_heartbeat)
       print ("  Is Armable?: %s" % vehicle.is_armable)
       print ("  System status: %s" % vehicle.system_status.state)
       print ("  Mode: %s" % vehicle.mode.name )    

       vehicle.mode = VehicleMode("LAND")

       break
    else:
      print("  Detected ID: %s" % ids)  

cap.release()
cv2.destroyAllWindows()





# Close vehicle object
vehicle.close()