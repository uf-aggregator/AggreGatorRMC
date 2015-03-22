#!/usr/bin/env python

import sys
import rospy
from common_files.srv import Coordinates

debug = False

def get_results_in_xy():
	rospy.wait_for_service('get_target_coordinates_in_xy')
	try:
		video_processing_service = rospy.ServiceProxy('get_target_coordinates_in_xy', Coordinates)
		response = video_processing_service()
		return response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_results_in_meters():
	rospy.wait_for_service('get_target_coordinates_in_meters')
	try:
		video_processing_service = rospy.ServiceProxy('get_target_coordinates_in_meters', Coordinates)
		response = video_processing_service()
		return response
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	

def usage():
	return "%s"%sys.argv[0]

def formatDecimal(decimal):
	if decimal == int(decimal):	return str(int(decimal))
	else: return str(decimal)

if __name__ == "__main__":
	if len(sys.argv) == 1:
		print "Requesting coordinates..."
		try: 
			coords = get_results_in_xy()
			if debug:
				while True:
					coords = get_results_in_xy()
					print "%d, %d"%(coords[0], coords[1])

			print "%d px, %d px"%(coords.output[0], coords.output[1])

			coords = get_results_in_meters()
			print "%s m, %s m"%(formatDecimal(coords.output[0]), formatDecimal(coords.output[1]))
		except Exception: 
			print "Something went wrong with the service."
			
	else:
		print usage()
		sys.exit(1)