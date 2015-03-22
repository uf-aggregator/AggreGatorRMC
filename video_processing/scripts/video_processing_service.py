#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from common_files.srv import Coordinates, CoordinatesResponse

debug = False

def video_processing_service():
	rospy.init_node('video_processing_service')
	coord_xy_service = rospy.Service('get_target_coordinates_in_xy', Coordinates, get_point_xy)
	coord_m_service = rospy.Service('get_target_coordinates_in_meters', Coordinates, get_point_meters);
	rospy.spin()

def process_video_input(req):
	#create the windows if debugging
	if debug:
		cv2.namedWindow('color',1)
		cv2.namedWindow('red',1)
		cv2.moveWindow('color', 300, 0)
		cv2.moveWindow('red',600, 0)

	#get the default video feed
	capture = cv2.VideoCapture(0)

	#get the return value and the video stream- e.g. frame
	ret, frame = capture.read()

	#get the dimensions
	height, width, depth = frame.shape

	#blur frame to handle noise
	frame = cv2.GaussianBlur(frame, (3,3), 0)

	#convert RGB to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# HSV bounds for red image
	lower_red = np.array([150, 80, 20])
	upper_red = np.array([190, 255, 255])

	#create a binary image for masking purposes
	red_mask = cv2.inRange(hsv, lower_red, upper_red)

	#denoise the binary image- e.g. remove little dots
	red_mask = cv2.medianBlur(red_mask, 7)

	#overlay binary image with color image
	red_isolated = cv2.bitwise_and(frame, frame, mask=red_mask)

	#get the contours from the mask
	contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	#get the contour with the best likelihood
	max_area = 0
	best_cnt = 0
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > max_area:
			max_area = area
			best_cnt = cnt

    #get the centerpoint of the best contour
	output = [];
	moments = cv2.moments(best_cnt)
	print moments
	if int(moments['m00']) != 0: #zero division check
		cx = int(moments['m10']/moments['m00'])
		cy = int(moments['m01']/moments['m00'])
		if cx > 0 and cy > 0:
			output = [cx, cy]

			#draw the center point if debugging
			if debug: 
				cv2.circle(red_isolated,(cx, cy), 5, (0, 255, 255), -1)

	#set feed if debugging
	if debug:
		while True:
			cv2.imshow('color', frame)
			cv2.imshow('red', red_isolated)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				cv2.destroyAllWindows()
				break

	#free the video feed
	capture.release()

	# acts like quadrant 1 coordinates
	result = [float(width - output[0]), float(height - output[1])]
	return result

def get_point_xy(req):
	response = CoordinatesResponse()
	response.output = process_video_input(req)
	return response

def get_point_meters(req):
	response = CoordinatesResponse()
	response.output = convertToMeters(process_video_input(req))
	return response

def convertToMeters(xyArr):
	pixelToCm = 0.026458333;
	return [xyArr[0]*pixelToCm/100, xyArr[1]*pixelToCm/100]

if __name__ == '__main__':
	try:
		video_processing_service()
	except rospy.ROSInterruptException:
		pass