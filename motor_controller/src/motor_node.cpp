
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "common_files/Drive.h"
#include "common_files/Ladder.h"
#include "common_files/Bucket.h"
#include "common_files/WriteI2C.h"

common_files::Drive last_drive_msg;
common_files::Bucket last_bucket_msg;
common_files::Ladder last_ladder_msg;

ros::Time last_drive_sub_time;
ros::Time last_drive_pub_time;
ros::Time last_bucket_sub_time;
ros::Time last_bucket_pub_time;
ros::Time last_ladder_sub_time;
ros::Time last_ladder_pub_time;

ros::Subscriber drive_sub;
ros::Subscriber bucket_sub;
ros::Subscriber ladder_sub;

void driveSubCallback (const common_files::Drive drive_msg){
	last_drive_msg = drive_msg;
	last_drive_sub_time = ros::Time::now();
}

void bucketSubCallback(const common_files::Bucket bucket_msg){
	last_bucket_msg = bucket_msg;
	last_bucket_sub_time = ros::Time::now();
}

void ladderSubCallback(const common_files::Ladder ladder_msg){
	last_ladder_msg = ladder_msg;
	last_ladder_sub_time = ros::Time::now();
}

//generate a stop message for the desired address
common_files::WriteI2C generate_stop_msg(unsigned char address, unsigned char direction){
        common_files::WriteI2C i2c_msg;
	//TODO: Will unsigned char work in a ros UInt8 message?
        i2c_msg.addr = address; //TODO: Check if this breaks
        i2c_msg.data.push_back(direction); //TODO: check if this breaks as well
        i2c_msg.data.push_back(0);
        i2c_msg.data.push_back(0);
        return i2c_msg;
}

common_files::WriteI2C generate_drive_msg(){
	//this function always uses the last drive msg
	common_files::WriteI2C i2c_msg;
	//Teensy 3.1 controls drive motors at address 1
	i2c_msg.addr = 1;
	/* Note about direction byte
		Left Forward, Right Forward: 0
		Left Forward, Right Backward: 1
		Left Backward, Right Forward: 2
		Left Backward, Right Backward: 3
	*/
	if(last_drive_msg.left >= 0 && last_drive_msg.right >= 0){
		//Left Forward, Right Forward
		i2c_msg.data.push_back(0);
	}
	if(last_drive_msg.left >= 0 && last_drive_msg.right < 0){
		//Left Forward, Right Backward
		i2c_msg.data.push_back(1);
	}
	if(last_drive_msg.left < 0 && last_drive_msg.right >= 0){
		//Left Backward, Right Forward
		i2c_msg.data.push_back(2);
	}
	if(last_drive_msg.left < 0 && last_drive_msg.right < 0){
		//Left Backward, Right Backward
		i2c_msg.data.push_back(3);
	}
	char left = (char) (abs(last_drive_msg.left*255.0));
	char right = (char) (abs(last_drive_msg.right*255.0));
	i2c_msg.data.push_back( left );
	i2c_msg.data.push_back( right );
	return i2c_msg;	
}

common_files::WriteI2C generate_bucket_msg(){
        common_files::WriteI2C i2c_msg;

	//Mega controls bucket and ladder at address 2
        i2c_msg.addr = 2;
        /* Note about direction byte
                Lift postive, Dump positive: 0
                Lift positive, Dump negative: 1
                Lift negative, Dump positive: 2
                Lift negative, Dump negative: 3
		Bucket commands: 4-7
        */
        if(last_bucket_msg.lift >= 0 && last_bucket_msg.dump >= 0){
                //Both positive
                i2c_msg.data.push_back(0);
        }
        if(last_bucket_msg.lift >= 0 && last_bucket_msg.dump < 0){
                //Lift positive, dump negative
                i2c_msg.data.push_back(1);
        }
        if(last_bucket_msg.lift < 0 && last_bucket_msg.dump >= 0){
                //Lift negative, dump positive
                i2c_msg.data.push_back(2);
        }
        if(last_bucket_msg.lift < 0 && last_bucket_msg.dump < 0){
                //Lift negative, dump negative
                i2c_msg.data.push_back(3);
        }
	//scale motor pwm values to 0-255
	//sign is handled in direction byte, so take absolute value
	char lift = (char) (abs(last_bucket_msg.lift*255.0));
        char dump = (char) (abs(last_bucket_msg.dump*255.0));
        i2c_msg.data.push_back( lift );
        i2c_msg.data.push_back( dump );
        return i2c_msg;
}

common_files::WriteI2C generate_ladder_msg(){
        common_files::WriteI2C i2c_msg;
	
	//Mega controls bucket and ladder at address 2
        i2c_msg.addr = 2;
        /* Note about direction byte
		Bucket commands: 0-3
                Lift postive, Convey positive: 4
                Lift positive, Convey negative: 5
                Lift negative, Convey positive: 6
                Lift negative, Convey negative: 7
        */
        if(last_ladder_msg.lift >= 0 && last_ladder_msg.conv >= 0){
                //Both positive
                i2c_msg.data.push_back(4);
        }
        if(last_ladder_msg.lift >= 0 && last_ladder_msg.conv < 0){
                //Lift positive, dump negative
                i2c_msg.data.push_back(5);
        }
        if(last_ladder_msg.lift < 0 && last_ladder_msg.conv >= 0){
                //Lift negative, dump positive
                i2c_msg.data.push_back(6);
        }
        if(last_ladder_msg.lift < 0 && last_ladder_msg.conv < 0){
                //Lift negative, dump negative
                i2c_msg.data.push_back(7);
        }
        //scale motor pwm values to 0-255
        //sign is handled in direction byte, so take absolute value
        char lift = (char) (abs(last_ladder_msg.lift*255.0));
        char conv = (char) (abs(last_ladder_msg.conv*255.0));
        i2c_msg.data.push_back( lift );
        i2c_msg.data.push_back( conv );
        return i2c_msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "motor_node"); 
	ros::NodeHandle n;
	
	//Subscribe to motor value messages
	drive_sub = n.subscribe("drive_vals", 10, driveSubCallback);
	bucket_sub = n.subscribe("bucket_vals", 10, bucketSubCallback);
	ladder_sub = n.subscribe("ladder_vals", 10, ladderSubCallback);

	//Publish values  on I2C
	ros::Publisher i2c_pub = n.advertise<common_files::WriteI2C>("write_i2c", 100);

	ros::Duration motor_update_rate(.25); 
	ros::Time curr_time = ros::Time::now();
	
	while(ros::ok()){
		//constantly check if we need to write new messages
			//if we do, check if the message is recent
				//convert the motor message to an I2C message if it is recent, and publish
				//else, write an I2C message of 0's to stop the motors			
	
		//start with drive message
		curr_time = ros::Time::now();
		if(curr_time - last_drive_pub_time > motor_update_rate){
			//it's time to update the motors
			//check if we've received a motor_msg recently
			if(curr_time - last_drive_sub_time > motor_update_rate){
				//the last drive msg has expired
				//send a message to stop Teensy 3.1 motors
				common_files::WriteI2C i2c_msg = generate_stop_msg(1, 0);
				i2c_pub.publish(i2c_msg);
				last_drive_pub_time = ros::Time::now();
			}else{
				//the last drive msg is recent enough to send
				common_files::WriteI2C i2c_msg = generate_drive_msg();
				i2c_pub.publish(i2c_msg);
				last_drive_pub_time = ros::Time::now();
			}
		}
		
		//handle bucket message	
		curr_time = ros::Time::now();
		if(curr_time - last_bucket_pub_time > motor_update_rate){
			//it's time to update bucket motors
			//check if we've received a bucket msg recently
			if(curr_time - last_bucket_sub_time > motor_update_rate){
				//last message has expired
				//send a stop message to the Mega (address 2)
				common_files::WriteI2C i2c_msg = generate_stop_msg(2, 0);
				i2c_pub.publish(i2c_msg);
				last_bucket_pub_time = ros::Time::now();
			}else{
				//the last message is recent enough to send
				common_files::WriteI2C i2c_msg = generate_bucket_msg();
				i2c_pub.publish(i2c_msg);
				last_bucket_pub_time = ros::Time::now();
			}
		}
		
		//handle ladder message
		curr_time = ros::Time::now();
                if(curr_time - last_ladder_pub_time > motor_update_rate){
                        //it's time to update ladder motors
                        //check if we've received a ladder msg recently
                        if(curr_time - last_ladder_sub_time > motor_update_rate){
                                //last message has expired
                                //send a stop message to the Mega (address 2)
                                common_files::WriteI2C i2c_msg = generate_stop_msg(2, 4);
                                i2c_pub.publish(i2c_msg);
                                last_ladder_pub_time = ros::Time::now();
                        }else{
                                //the last message is recent enough to send
                                common_files::WriteI2C i2c_msg = generate_ladder_msg();
                                i2c_pub.publish(i2c_msg);
                                last_ladder_pub_time = ros::Time::now();
                        }
                }
		
		ros::spinOnce();
	}
}
