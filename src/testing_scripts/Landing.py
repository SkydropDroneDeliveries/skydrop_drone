import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  
from cv2 import aruco



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



class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/webcam/image_raw",Image,self.callback)

    def callback(self, msg):
        #rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)

       
        # Capture frame-by-frame
        frame = self.image
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
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
        if (ids==5):
            print("Now let's land")
            print("  Detected ID: %s" % ids) 
            # Get some vehicle attributes (state)
            print ("  Last Heartbeat: %s" % vehicle.last_heartbeat)
            vehicle.mode = VehicleMode("LAND")
            cv2.destroyAllWindows()
            # Close vehicle object
            vehicle.close()
            #exit()

        else:
            print("  Detected ID: %s" % ids) 

    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            br = CvBridge()
            if self.image is not None:
                self.pub.publish(br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()



    
                                


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


if __name__ == '__main__':
    #Takeoff height in meters
    arm_and_takeoff(4)

    print("Take off complete")

    vehicle.gimbal.rotate(-90, 0, 0)

    # Hover for 10 seconds
    time.sleep(2)

    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()

