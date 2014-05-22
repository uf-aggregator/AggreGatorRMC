#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "camera_feedback/camera_feedback.h"
#include "image_transport/image_transport.h"
#include <sensor_msgs/image_encodings.h>
/*#include "opencv/cv.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
*/
void imageCallback(const sensor_msgs::ImageConstPtr& img){
	/*
	sensor_msgs::CvBridge bridge; //needed to convert ROS images to OpenCV images
	try
	{	
		//upon receiving a new image, show this image in OpenCV
		cvShowImage("view", bridge.imgMsgToCv(msg, "bgr8"));
		//TODO: Some code to save every x-th image to Command computer's HD
		//OpenCV has functions for saving images, but we can write it
			//once we get OpenCV working
	}
	catch (sensor_msgs::CvBridgeException& e)
	{	
		//if the image could not be converted to OpenCV, provide an error message
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	*/
}

int main(int argc, char **argv){
	CameraNode *cam_node = new CameraNode(argc, argv);
	/*
	cvNamedWindowThread("view");
	cvStartWindowThread();
	image_transport::ImageTransport it(cam_node->n);
	cam_node->imageSub = it.subscribe("/usb_cam/image_raw",1, imageCallback);
	ros::spin();
	cvDestroyWindow("view");
	*/
}


