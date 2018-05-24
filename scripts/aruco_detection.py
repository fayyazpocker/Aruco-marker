#!/usr/bin/env python
'''
ArUco ROS package for aruco detection.
rvec and tvec (returned by aruco.estimatePoseSingleMarkers) are respectively the rotational and translational vector of ArUco markers

'''
import roslib
import rospy
import sys
import cv2
import cv2.aruco as aruco
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


bridge = CvBridge()
# camera_info = CameraInfo()
distortion_matrix = np.zeros((1,5),np.float64)
camera_matrix = np.zeros((3,3),np.float64)
#print camera_matrix


class Aruco_detection():
	def __init__(self):
		rospy.init_node('aruco_detection',anonymous=True)
		self.image_pub = rospy.Publisher("/aruco/marked_image_out",Image,queue_size=1)
		self.cam_info = rospy.Subscriber("/usb_cam/camera_info",CameraInfo,self.set_camera_parameters)
		# rospy.sleep(2)
		rospy.Subscriber("/usb_cam/image_rect_color",Image,self.mark_aruco)

	def set_camera_parameters(self,data):
		global distortion_matrix,camera_matrix
		# distortion_matrix = np.array(list(data.D))
		# camera_matrix = np.array(list(data.K))
		# print list(data.K[3:6])
		list1 = list(data.K[0:3])
		list2 = list(data.K[3:6])
		list3 = list(data.K[6:9])

		camera_matrix = np.array([list1,list2,list3])
		distortion_matrix = np.array([list(data.D)])

		# camera_matrix = np.array([[534.34144579,0,339.15527836],
									# [0,534.68425882,233.8459493],
									# [0,0,1]])
		# distortion_matrix = np.array([[-2.88320983e-01, 5.41079685e-02,1.73501622e-03,-2.61333895e-04,2.04110465e-01]])
		# print camera_matrix
		# print distortion_matrix
		# if(not distortion_matrix):
		# if np.all(distortion_matrix !=None):
		# 	while True:
		# 		pass
		#print type(camera_matrix.dtype)

	def mark_aruco(self,data):
		global bridge,distortion_matrix,camera_matrix
		frame = bridge.imgmsg_to_cv2(data,"bgr8")

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		parameters = aruco.DetectorParameters_create()

		#lists of ids and the corners beloning to each id
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

		font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)


		if np.all(ids != None):

			# print distortion_matrix
			# print camera_matrix
			# while True:
			# 	pass
			rvec, tvec,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion_matrix) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
			# print tvec
			#(rvec-tvec).any() # get rid of that nasty numpy value array error

			for i in range(0, ids.size):
				aruco.drawAxis(frame, camera_matrix, distortion_matrix, rvec[i], tvec[i], 0.1)  # Draw Axis
			aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers


			###### DRAW ID #####
			strg = ''
			for i in range(0, ids.size):
				strg += str(ids[i][0])+', '

			cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


		else:
			##### DRAW "NO IDS" #####
			cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)



		self.image_pub.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))




if __name__=="__main__":
	aruco_det = Aruco_detection()
	while not rospy.is_shutdown():
		rospy.spin()

