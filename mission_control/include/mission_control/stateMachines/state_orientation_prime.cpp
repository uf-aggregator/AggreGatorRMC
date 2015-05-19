
#include "state_orientation_prime.h"
//#define LidarCoordinates 2 
using namespace std;

OrientationStatePrime::OrientationStatePrime(){
	
Position = ""; // Determines the Position for the Next State 
Gyroscope = 0; //Gyroscope Angle
Starting_Line = 2; // The Distance from Lidar to the Starting Line, should be known "fixed"
// Initial Coordinates, they will get updated every time the Robot is moving 
	
}	
double OrientationStatePrime::Initial_x = 0; 
double OrientationStatePrime::Initial_y = 0;

const ros::Duration move_time(3.0);

ros::Subscriber Read_Lidar;
ros::Subscriber Read_Gyro;
ros::Publisher	Gyro_Reset;


/* Initializing STATIC Variables __________________________*/
common_files::Centroid OrientationStatePrime::LidarCoordinates;
std_msgs::Float32 OrientationStatePrime::GyroAngle;

/* CALLBACKS ______________________________________________*/
void OrientationStatePrime::LidarCoordinatesCallback(const common_files::Centroid::ConstPtr& msg) {
	LidarCoordinates.x = msg->x;
	LidarCoordinates.y = msg->y;
}
void OrientationStatePrime::GyroCallback(const std_msgs::Float32::ConstPtr& msg) {
	GyroAngle.data = msg->data;
	
}
void OrientationStatePrime::Move_A_Bit(){
	ros::Time startTime = ros::Time::now();
	while(startTime - ros::Time::now() < move_time){
	motor_utility::write(1,1);
	Initial_x = LidarCoordinates.x;
	Initial_y = LidarCoordinates.y;
	}
}

int OrientationStatePrime::Determine_Location( double x, double y){
	
	Move_A_Bit();
	if(LidarCoordinates.x == Initial_x && LidarCoordinates.y <= Initial_y){
		
		return 1;//"Wall_I";
	}
	else if(LidarCoordinates.x == Initial_x && LidarCoordinates.y >= Initial_y){
		
		return 2;//"Open_I";
	}
	else if(LidarCoordinates.x <= Initial_x && LidarCoordinates.y == Initial_y){
		
		return 3;//"Open_II";
	}
	else if(LidarCoordinates.x >= Initial_x && LidarCoordinates.y == Initial_y){
		
		return 4;//"Wall_II";
	}
	else if(LidarCoordinates.x >= Initial_x && LidarCoordinates.y <= Initial_y){
		
		return 5;//"Star";
	}
	else if(LidarCoordinates.x <= Initial_x && LidarCoordinates.y >= Initial_y){
		
		return 6;//"Opposite_Of_Star";
	}
}
double OrientationStatePrime::Distance_Traveled(){

	if(Determine_Location(Initial_x,Initial_y) == 1){
		return (LidarCoordinates.y - Initial_y);
	}
	if(Determine_Location(Initial_x,Initial_y) == 3){
		return (LidarCoordinates.x - Initial_x);
	}
}

int OrientationStatePrime::OrientToStart(){
	
	int argc; char** argv; 	
	ros::init(argc, argv, "Determine_Position");
	ros::NodeHandle n;
/*
	common_files::Drive msg;
	msg.left = 
	msg.right = 
	motor_utility::publish_to_wheels(msg);
*/
	Read_Lidar = n.subscribe("Centroid", 1, LidarCoordinatesCallback);
	Read_Gyro = n.subscribe("orientation_angle", 1, GyroCallback);
	Gyro_Reset = n.advertise<std_msgs::Int8>("gyro_command", 1);
	ros::spinOnce();

	int temp = Determine_Location(Initial_x, Initial_y);
	while(ros::ok()){ 
		
		switch(temp){
			case 1: 
				while(Distance_Traveled() <= 1.0){
						//Motor_Backup;					
						motor_utility::write(-1,-1);
				}
				// motor_utility::stop_wheels();

				while(GyroAngle.data <= 90){
						// Motor_Rotate_ClockWise;
						 motor_utility::write(1,-1);
						// Reset_Gyroscope;
						std_msgs::Int8 Reset;
						Reset.data = 0;
						Gyro_Reset.publish(Reset);
				} 
				while(LidarCoordinates.x >= 0){
						// Motor_Move_Forward;
						 motor_utility::write(1,1);
				}
				while(GyroAngle.data <= 90){
						// Motor_Rotate_ClockWise;
						 motor_utility::write(1,-1);
						// Reset_Gyroscope;
						std_msgs::Int8 Reset;
						Reset.data = 0;
						Gyro_Reset.publish(Reset);
				} 
				while(LidarCoordinates.y <= Starting_Line){
						// Motor_Move_Forward;
						motor_utility::write(1,1);
				}
				
				break;
	
			case 2: 
				
				break;
	
			case 3: 
				 
				break;
	
			case 4: 
				 
				break;
			case 5: 
				 
				break;
	
			case 6: 
				 
				break;
		}
		
		//next iteration, Read new Data
		ros::spinOnce();	
	}
	return 0;	
}

/*___________________________________________________________________________________________*/
/*
int main(int argc, char** argv){
	
	// the Client name is: calling_Lidar_service
	ros::init(argc, argv, "calling_Lidar_service");
	// Check if the Lidar Service is returning x,y "two parameters"
	if(argc <= LidarCoordinates){
		// ROS Feedback 
		ROS_INFO("Usage: Getting the x,y from Lidar");
		return 1;
	}
	ros::NodeHandle n;
	
	ros::ServiceClient client = n.serviceClient<common_files::ReadLidar>("read_lidar");
	
	common_files:: ReadLidar srv;
	
	//srv.request.x = atoll(argv[1]);
	//srv.request.y = atoll(argv[2]);
	
while (ros::ok())
    {
	// Check if the service is responding 
	if(client.call(srv)){
		ROS_INFO("Service call successful");
	}
	else {
		ROS_ERROR("FAILD to call service: get_coordinates");
		return 1;
	}
	ros::spinOnce();
	ros::Duration(.5).sleep(); //sleep for half a second
 }
	
	return 0;
}
*/
