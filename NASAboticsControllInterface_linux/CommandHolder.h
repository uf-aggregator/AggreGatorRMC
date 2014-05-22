#pragma once


#include "OperationModeEnum.h"
#include <string>
#include <sstream>
#include <qobject.h>

/*
    Bitmask for command flags
*/
typedef unsigned short bitmask;
enum CommandFlagEnum
{
    NONE					=		0x0000,			//no flags!

    TAKE_PIC				=		0x0001,			//Polls a picture from the camera on the NASAbot
    POLL_LADAR				=		0x0002,			//Polls the proccesed LadarData from the NASAbot
    SWITCH_OPERATION_MODE	=		0x0004,			//If true then the NASAbot will change opperation mode

    ALL						=		0XFFFF			//all flags!
};


/*
    Class to contain and maintain a command set
*/
class CommandHolder : public QObject
{
    Q_OBJECT
    /*
        Private variables
    */
private:
    /*
        Mechanical actuation controlls
    */
    //Wheels
    //controls the speed and direction of the wheels
    int front_left_wheel_velocity_;
    int front_right_wheel_velocity_;
    int back_left_wheel_velocity_;
    int back_right_wheel_velocity_;

    //Bucket Drum
    int bucket_pitch_control_;			//Controls the speed and direction of the bucket drum arm
    int bucket_mine_control_;			//Controls the speed of mining or dumping (negative values)
                                        //Corespond to dumping while positive values are mining

    //constants for integer maping
    static const int kWheelsMaxValue = 32768;		//2 byte signed integer -32768 to 32767
    static const int kBucketMaxValue = 128;			//1 byte signed integer -128 t0 127, for both pitch and mining

    /*
        Electrical/Computer controlls
    */
    bitmask flags_;						//flags for different commands see enum for bit maping
    OperationMode next_operation_mode_;	//If operation mode switch is true then switch to this mode

    /*
        Constructor and Destructors
    */
public:
    CommandHolder():
        front_left_wheel_velocity_(0),	front_right_wheel_velocity_(0),
        back_left_wheel_velocity_(0),	back_right_wheel_velocity_(0),
        bucket_pitch_control_(0),		bucket_mine_control_(0),
        flags_(0),						next_operation_mode_(AUTONOMOUS)

    {}

public:
    //copy function (Note can't use assignment opperator)
    void CopyValues(CommandHolder* cmd);

    /*
        Public Functions
    */
public:
    std::string DebugOutput();			//Outputs a debug info
    std::string SendCommand();			//Outputs the command in the form of a string

    void Clear();						//Clears the command back to default values

    void RunningAverage(CommandHolder* cmd, unsigned int count);		//makes a running avearage by adding cmd to this

    /*
        Private functions
    */
private:
    float ScaleFloat(float num);						//Scales floats to -1 to 1
    int ScaleInt(float num, int max_value);				//Scales floats to an int between -max_value and max_value
    int ScaleInt(int num, int max_value);				//Makes sure that ints are in the appropriate range between -max_value and max_value

/*
    Signals and slots
*/
signals:
    void FrontLeftWheelChanged(int value);
    void FrontRightWheelChanged(int value);
    void BackLeftWheelChanged(int value);
    void BackRightWheelChanged(int value);

    void BucketPitchChanged(int value);
    void BucketMineChanged(int value);

    void TakingPicture(bool pic);
    void PollingLadar(bool poll);

    void SwitchingOperationMode(OperationMode nextMode);

public slots:
    void SetFrontLeftWheelVelocity(int velocity);
    void SetFrontRightWheelVelocity(int velocity);
    void SetBackLeftWheelVelocity(int velocity);
    void SetBackRightWheelVelocity(int velocity);

    void SetBucketPitchControl(int rate);
    void SetBucketMineControl(int rate);					//negative values indicate dumping

    void SetOperationMode(OperationMode nextMode);			//changes the operation mode

    void TakePicture();										//polls the NASAbot for a picture
    void PollLadarData();									//polls the NASAbot for LADAR data

    void SetFlags(bitmask);									//set specified flags at once
    void ClearFlags(bitmask);								//reset spcified flags

    /*
        Set Functions: most of these functions call a slot which is why they aren't slots themself
    */
public:
    //all float values are between -1 and 1 unless otherwise stated
    void SetLeftWheelsVelocity(float velocity);				//sets both left wheels to velocity
    void SetRightWheelsVelocity(float velocity);			//sets both right wheels to velocity
    void SetLeftWheelsVelocity(int velocity);				//sets both left wheels to velocity
    void SetRightWheelsVelocity(int velocity);				//sets both right wheels to velocity

    void SetFrontLeftWheelVelocity(float velocity);
    void SetFrontRightWheelVelocity(float velocity);
    void SetBackLeftWheelVelocity(float velocity);
    void SetBackRightWheelVelocity(float velocity);

    void SetBucketPitchControl(float rate);
    void SetBucketMineControl(float rate);					//negative values indicate dumping
    /*
        Operator overloading
    */
public:
    //CommandHolder& operator+=(CommandHolder& cmd);
};
