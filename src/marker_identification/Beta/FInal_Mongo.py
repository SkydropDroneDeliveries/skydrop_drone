from pymongo import MongoClient


from operator import truediv
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2, PIL, os
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import time
import random
import subprocess


# Connect to the Vehicle
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
parser.add_argument('apartmentNo')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)


######################################### Connecting to Mongo Client #########################################

CONNECTION_STRING = "mongodb://localhost:27017"
HOST = 'localhost'
PORT = 27017
DATABASE = 'skydrop'
COLLECTION = 'apartments'

client = MongoClient(HOST,PORT)
collection = client[DATABASE][COLLECTION]

APARTMENT_NO = args.apartmentNo

query = {"apartmentNo": APARTMENT_NO}

apartmentDetails = collection.find_one(query)

LONG = apartmentDetails["location"]["coordinates"][0]
LAT = apartmentDetails["location"]["coordinates"][1]
ALTITUDE = apartmentDetails["altitude"]
APARTMENT_ID = apartmentDetails["arucoID"]
ORIENATATION = apartmentDetails["orientation"]


# LONG = 149.1647052
# LAT = -35.3627122
# ALTITUDE = 8 
# APARTMENT_ID = 7

print("....................................dwd")
print(LONG)
print(LAT)
print(ALTITUDE)
print(APARTMENT_ID)
print(ORIENATATION)


INITIAL_HEIGHT = 20




############################### Log File Info ###############
LOG_StartTime = time.time()
NAV_SUCCESS = False
DELIVERY_SUCCESS = True
LOG_IsMarkerIdentified = False
DELIVERY_NO = random.randint(0,100)
LOG_DeliveryTIme = 0

aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
deliveryDone = False

# Add callibration paths
# Camera Callibration Matrix
drone_mtx = np.array([[1.74213359e+03, 0.00000000e+00, 1.27150514e+03],
 [0.00000000e+00, 1.74213359e+03, 1.02516982e+03],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

drone_dist = np.array([[-1.69684883e+00],
 [-6.85717812e+00],
 [ 9.93624014e-03],
 [ 6.20144084e-04],
 [-1.18739065e+01],
 [-1.69460711e+00],
 [-6.99110211e+00],
 [-1.13633464e+01],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00],
 [ 0.00000000e+00]])
mtx, dist = drone_mtx, drone_dist

#Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
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

def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

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
        global sub
        sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback)

        
        rospy.spin()
 

    def callback(self, msg):
        #rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        gndSpeedDistance = 0.2 # [m/s]
       
        # Capture frame-by-frame
        frame = self.image
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters =  aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict,
                                                            parameters=parameters)
        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        size_of_marker =  0.15 # side lenght of the marker in meter
        rvecs,tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, size_of_marker, mtx, dist)

        length_of_axis = 0.1
        imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        if  (ids == APARTMENT_ID):
            if tvecs is not None:
                for i in range(len(tvecs)):
                    imaxis = aruco.drawAxis(imaxis, mtx, dist, rvecs[i], tvecs[i], length_of_axis)
                    x = (corners[0][0][0][0] + corners[0][0][2][0])/2
                    y = (corners[0][0][0][1] + corners[0][0][2][1])/2
                    square_side_dimension = math.sqrt(math.pow(corners[0][0][3][1] - corners[0][0][0][1], 2) +
                                                math.pow(corners[0][0][3][0] - corners[0][0][0][0], 2))

                    cv2.circle(imaxis, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), 12, (255, 255, 0), 1)
                    cv2.circle(imaxis, (int(corners[0][0][3][0]), int(corners[0][0][3][1])), 12, (0, 0, 255), 1)
                    cv2.circle(imaxis, (int(x), int(y)), 12, (0, 255, 255), 2)


            try:
                
                distance_cm_pc = 3317 * math.pow(square_side_dimension, -0.7468) + (-45.95)
                distance_cm_drone = 1.129e+04 * math.pow(square_side_dimension, -0.9631) + (-11.26)

                distance_cm = distance_cm_drone

                # add send velocity function with small speed....
                
                imaxis = cv2.putText(imaxis, str(distance_cm), (100, 200), 5, 5, (50, 255, 100))
                print("Square side in pixels:", square_side_dimension)
                print("calculated distance in cm:", distance_cm)
                
                if(distance_cm >= 250):
                    set_velocity_body(vehicle, gndSpeedDistance, 0, 0)
                    print("MOVING Forward")
                    
                elif(distance_cm < 180):
                    set_velocity_body(vehicle, -gndSpeedDistance, 0, 0)
                    print("MOVING")

                elif(180 < distance_cm and distance_cm <= 250):
                    print("Ready to Deliver")
                    LOG_IsMarkerIdentified = True
                    time.sleep(5)
                    print("Delivering.........................")
                    deliveryDone = True

                print("  Detected ID: %s" % ids) 
                print("=================================")
            except:
                print("No marker Detected")


    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            br = CvBridge()
            if self.image is not None:
                self.pub.publish(br.cv2_to_imgmsg(self.image))
            if deliveryDone == True:
                print("Topic shutdown") 
                sub.shutdown()
                rospy.signal_shutdown()
            self.loop_rate.sleep()
            

def return_navigation():

    print("Delivery Complted")
    print("Starting navigation back to the landing pad ...")
    LOG_IsMarkerIdentified = True

    time.sleep(3)
    vehicle.gimbal.rotate(0, 0, ORIENATATION)
    time.sleep(3)

    point2 = LocationGlobalRelative(LAT, LONG, ALTITUDE)
    vehicle.simple_goto(point2)
    time.sleep(3)

    point3 = LocationGlobalRelative(LAT, LONG, 30)
    vehicle.simple_goto(point3)
    time.sleep(12)

    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    print("Land")
    # vehicle.mode = VehicleMode("LAND")

    # Delivery Elpased Time
    LOG_EndTime = time.time()
    LOG_EndTime = time.time()
    hours, rem = divmod(LOG_EndTime-LOG_StartTime, 3600)
    minutes, seconds = divmod(rem, 60)
    LOG_DeliveryTIme = "{:0>2}:{:0>2}:{:05.2f}".format(int(hours),int(minutes),seconds)

    # Close vehicle object before exiting script
    print("Close vehicle object")
    print(LOG_DeliveryTIme)

    vehicle.close()





def attitude_callback(self, attr_name, value):
    print(vehicle.attitude)

#########################################  Takeoff drone to point above apartment #########################################

#Takeoff height in meters
print("Drone started and takeoff to the altitutde of") #add altidue var
arm_and_takeoff(INITIAL_HEIGHT) #add altitude var
print("Take off complete")

print("Set default/target airspeed to 3") 
vehicle.airspeed = 3

#########################################  Navigating drone to point above excact apartment balcony #########################################

print("Going towards the apartment buliding and placed over the building.....")
point1 = LocationGlobalRelative(LAT, LONG, INITIAL_HEIGHT) #add altidue var
vehicle.simple_goto(point1)

# # sleep so we can see the change in map
time.sleep(30)
print("GPS Navigation Succesfully completed")


######################################### VO Downward Navigation #########################################

vehicle.gimbal.rotate(0, 0, ORIENATATION)
time.sleep(6)
point1 = LocationGlobalRelative(LAT, LONG, INITIAL_HEIGHT)
time.sleep(6)


print("Descending to the apartment balcony height using VO")
point2 = LocationGlobalRelative(LAT, LONG, ALTITUDE)
vehicle.simple_goto(point2)

# # sleep so we can see the change in map
time.sleep(12)
print("VO NAvigation succesfully completed")
NAV_SUCCESS = True

#########################################  Scanning for relavnt Aruco Marker #########################################
print("Waiting for marker identification to start")


# print("..........................................")
# print("Adding an attitude listener")
# vehicle.add_attribute_listener('attitude', attitude_callback) #-- message type, callback function
# time.sleep(5)


rospy.init_node("imagetimer", anonymous=True)
my_node = Nodo()
my_node.start()


print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")



#############################  Align and move inside the balcony to the relavnt Aruco Marker #########################





# vehicle.remove_attribute_listener('attitude', attitude_callback) #(.remove)





# def get_distance_metres(aLocation1, aLocation2):
#     """
#     Returns the ground distance in metres between two LocationGlobal objects.

#     This method is an approximation, and will not be accurate over large distances and close to the 
#     earth's poles. It comes from the ArduPilot test code: 
#     https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
#     """
#     dlat = aLocation2.lat - aLocation1.lat
#     dlong = aLocation2.lon - aLocation1.lon
#     return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5




################################### Return Navigation to the drone landing pad ###################################






return_navigation()

def print_logfile():

    file1 = open("LOG.txt","a")
    L = ["-Log entry Time: "+ str(LOG_StartTime) +"\n",
        "-Apartemtn ID: "+ str(APARTMENT_ID) +" \n"
        "-Navigation Phase  "+ str(NAV_SUCCESS)+"\n",
        "-Marker Identification \n",
        "      -Marker identified/Alignment: "+ "True" +"\n",
        "-Delivery time:"+str(LOG_DeliveryTIme)+ "\n",
        "\n",
        "Delivery Success:  "+ str(DELIVERY_SUCCESS) +"\n"     
        "\n",
        "=========================================\n",     
        "\n",
        "\n",
        "\n"
        ]

    file1.write("DELIVERY NUMBER: ")
    file1.write('%d' % DELIVERY_NO)
    file1.write("\n")
    file1.write("**************************************\n")

    file1.writelines(L)
    file1.close() 

    file1 = open("LOG.txt","r+")

    print("Log Output")
    print(file1.read())
    print()

print_logfile()