#ifndef _CAMERA_NODE_
#define _CAMERA_NODE_

#include "ros/ros.h"
#include "image_transport/image_transport.h"

class CameraNode {
	public:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::Subscriber sub;
		image_transport::Subscriber imageSub;

		CameraNode(int argc, char **argv){
			ros::init(argc, argv, "camera_feedback_node");
		}
		~CameraNode(){}
};

#endif