import numpy as np
import cv2, PIL, os
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import math

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

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
        rvecs,tvecs = aruco.estimatePoseSingleMarkers(corners, size_of_marker , mtx, dist)

        length_of_axis = 0.1
        imaxis = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


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
            
            imaxis = cv2.putText(imaxis, str(distance_cm), (100, 200), 5, 5, (50, 255, 100))
            print("Square side in pixels:", square_side_dimension)
            print("calculated distance in cm:", distance_cm)
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
            self.loop_rate.sleep()



    
      






   
if __name__ == '__main__':
    
    rospy.init_node("imagetimer", anonymous=True)
    my_node = Nodo()
    my_node.start()
