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


###########################
# MAIN METHOD
###########################
if __name__ == "__main__":
	if len(sys.argv) == 1:
		print "Requesting coordinates..."
		try: 
			coords = get_results_in_xy()
			if debug:
				while True:
					coords = get_results_in_xy()
					print "%d px, %d px"%(coords.output[0], coords.output[1])
			print "For a camera of res: %dx%d"%(coords.width, coords.height)
			print "Point 1: %d px, %d px"%(coords.output[0], coords.output[1])
			print "Point 2: %d px, %d px"%(coords.output[2], coords.output[3])

			coords = get_results_in_meters()
			print "For a camera of res: %fm x%fm"%(coords.width, coords.height)
			print "Point 1: %s m, %s m"%(formatDecimal(coords.output[0]), formatDecimal(coords.output[1]))
			print "Point 2: %s m, %s m"%(formatDecimal(coords.output[2]), formatDecimal(coords.output[3]))

		except Exception: 
			print "Something went wrong with the service."
			
	else:
		print usage()
		sys.exit(1)