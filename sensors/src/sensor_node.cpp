/*
This node is meant to poll any and all sensors on the i2c bus
	-The only data returned from the teensy is encoders
	-The only data returned from the mega is current sense

*/

#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "common_files/ReadI2C.h"
#include "common_files/Encoders.h"
#include "std_msgs/Float32.h"
//#include "common_files/CurrentSense.h"

common_files::Encoders encoder_total;

ros::ServiceClient read_i2c;
ros::Publisher encoders;
ros::Publisher motor_current;
ros::Publisher overall_current;

void sensorCallback(const ros::TimerEvent&){
	int curr_left_pos = 0;
	int curr_right_pos = 0;
	//First read Teensy 3.1 at address 1
        common_files::ReadI2C teensy_srv;
        teensy_srv.request.addr = 1;
        teensy_srv.request.size = 4;
        if(read_i2c.call(teensy_srv)){
        //      ROS_INFO("teensy_srv: %d, %d, %d, %d", teensy_srv.response.data[0], teensy_srv.response.data[1], teensy_srv.response.data[2], teensy_srv.response.data[3]);
                curr_left_pos = teensy_srv.response.data[0] << 24;
                curr_left_pos = curr_left_pos | (teensy_srv.response.data[1] << 16);
		curr_left_pos = curr_left_pos | (teensy_srv.response.data[2] << 8);
		curr_left_pos = curr_left_pos | (teensy_srv.response.data[3]);

                curr_right_pos = 0;

        //      ROS_INFO("curr_left_pos: %d, curr_right_pos: %d", curr_left_pos, curr_right_pos);
                encoder_total.encoder0Pos = encoder_total.encoder0Pos + curr_left_pos;
                encoder_total.encoder1Pos = 0;
		encoders.publish(encoder_total);
        }else{
                ROS_INFO("Failed to call read_i2c teensy service");
        }

	common_files::ReadI2C mega_srv;
        mega_srv.request.addr = 2; //read from mega
        mega_srv.request.size = 10; //ten bytes to read
        unsigned int ladder_lift_cs = 0;
        unsigned int ladder_conv_cs = 0;
        unsigned int bucket_lift_cs = 0;
        unsigned int bucket_dump_cs = 0;
	unsigned int overall_cs = 0;
        if(read_i2c.call(mega_srv)){
                ladder_lift_cs = mega_srv.response.data[0] << 8;
                ladder_lift_cs = ladder_lift_cs | mega_srv.response.data[1];
                ladder_conv_cs = mega_srv.response.data[2] << 8;
                ladder_conv_cs = ladder_conv_cs | mega_srv.response.data[3];
                bucket_lift_cs = mega_srv.response.data[4] << 8;
                bucket_lift_cs = bucket_lift_cs | mega_srv.response.data[5];
                bucket_dump_cs = mega_srv.response.data[6] << 8;
                bucket_dump_cs = bucket_dump_cs | mega_srv.response.data[7];
		overall_cs = mega_srv.response.data[8] << 8;
		overall_cs = overall_cs | mega_srv.response.data[9];
		
		std_msgs::Float32 overall_current_msg;
		overall_current_msg.data = (-30.0/65536.0)*overall_cs + 30;
		overall_current.publish(overall_current_msg);

		ROS_INFO("Raw overall current value: %d; Computed current: %f", overall_cs, overall_current_msg.data);
	
        //      for(int i = 0; i < 8; i++){
        //              ROS_INFO("data[%d]: %d", i, mega_srv.response.data[i]);
        //      }
	//	ROS_INFO("Ladder lift: %d, ladder conv: %d, bucket lift: %d, bucket dump: %d", ladder_lift_cs, ladder_conv_cs, bucket_lift_cs, bucket_dump_cs);
        }else{
                ROS_ERROR("Failed to call read_i2c mega service");
        }
}

void rosoutCallback(const ros::TimerEvent&){
        ROS_INFO("total_left_pos: %d, total_right_pos: %d", encoder_total.encoder0Pos, encoder_total.encoder1Pos);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sensor_node"); 
	ros::NodeHandle n;
		
	read_i2c = n.serviceClient<common_files::ReadI2C>("read_i2c");
	encoders = n.advertise<common_files::Encoders>("encoders", 1);
	overall_current = n.advertise<std_msgs::Float32>("overall_current", 1);
	
	//read sensors every tenth of a second
	ros::Timer sensor_timer = n.createTimer(ros::Duration(0.1), sensorCallback);
	
	ros::Timer ros_out_timer = n.createTimer(ros::Duration(1.0), rosoutCallback);

	while(ros::ok()){
		ros::spinOnce();
	}
}
