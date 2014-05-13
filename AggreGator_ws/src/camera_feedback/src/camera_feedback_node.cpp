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


