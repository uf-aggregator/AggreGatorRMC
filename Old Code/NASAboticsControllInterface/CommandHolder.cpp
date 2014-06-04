
#include "CommandHolder.h"

/*
	Private functions
*/
float CommandHolder::ScaleFloat(float num)
{
	if(num > 1.0f)
		num = 1.0f;
	else if (num < -1.0f)
		num = -1.0f;

	return num;
}

int CommandHolder::ScaleInt(float num, int max_value)
{
	return (int)(num * max_value);
}

int CommandHolder::ScaleInt(int num, int max_value)
{
	if(num > max_value)
		return max_value - 1;
	else if(num < -max_value)
		return - max_value;
	
	return num;
}

/*
	Operator Overloading
*/
//adds two cmds together by averaging floats and or-ing flags
/* ______Can't use operator overloading with a QObject__________
CommandHolder& CommandHolder::operator+=(CommandHolder& cmd)
{
	//average floats
	left_wheels_velocity_ = (left_wheels_velocity_ + cmd.left_wheels_velocity_) / 2;
	right_wheels_velocity_ = (right_wheels_velocity_ + cmd.right_wheels_velocity_) / 2;
	bucket_pitch_control_ = (bucket_pitch_control_ + cmd.bucket_pitch_control_) / 2;
	bucket_mine_control_ = (bucket_mine_control_ + cmd.bucket_mine_control_) / 2;

	//or flags
	flags_ |= cmd.flags_;

	//change next operation mode
	next_operation_mode_ = cmd.next_operation_mode_;

	return *this;
}
*/

/*
	Public functions
*/
/*
	Find the running average with following formula:
	((count - 1) * last_value + current_value) / count
	the count must be kept externaly
*/
void CommandHolder::RunningAverage(CommandHolder* cmd, unsigned int count)
{
	//debug only
	count = 1;
	//debug only

	//average ints, could cause significant rounding error... (Should be fine)
	SetFrontLeftWheelVelocity((int)(((count-1)*front_left_wheel_velocity_ + cmd->front_left_wheel_velocity_) / count));
	SetFrontRightWheelVelocity((int)(((count-1)*front_right_wheel_velocity_ + cmd->front_right_wheel_velocity_) / count));
	SetBackLeftWheelVelocity((int)(((count-1)*front_left_wheel_velocity_ + cmd->front_left_wheel_velocity_) / count));
	SetBackRightWheelVelocity((int)(((count-1)*front_right_wheel_velocity_ + cmd->front_right_wheel_velocity_) / count));

	SetBucketPitchControl((int)(((count-1)*bucket_pitch_control_ + cmd->bucket_pitch_control_) / count));
	SetBucketMineControl((int)(((count-1)*bucket_mine_control_ + cmd->bucket_mine_control_) / count));

	//or flags
	SetFlags(cmd->flags_);

	//change next operation mode
	next_operation_mode_ = cmd->next_operation_mode_;
}

//copy function
void CommandHolder::CopyValues(CommandHolder* cmd)
{
	SetFrontLeftWheelVelocity(cmd->front_left_wheel_velocity_);
	SetFrontRightWheelVelocity(cmd->front_right_wheel_velocity_);
	SetBackLeftWheelVelocity(cmd->back_left_wheel_velocity_);
	SetBackRightWheelVelocity(cmd->back_right_wheel_velocity_);

	SetBucketPitchControl(cmd->bucket_pitch_control_);
	SetBucketMineControl(cmd->bucket_mine_control_);

	ClearFlags(ALL);
	SetFlags(cmd->flags_);

	next_operation_mode_ = cmd->next_operation_mode_;
}

//set specified flags
void CommandHolder::SetFlags(bitmask flags)
{
	flags_ |= ~flags;

	if(flags & SWITCH_OPERATION_MODE)
		emit SwitchingOperationMode( next_operation_mode_);

	if(flags & TAKE_PIC)
		emit TakingPicture(true);

	if(flags & POLL_LADAR)
		emit PollingLadar(true);
}

//clear specified flags
void CommandHolder::ClearFlags(bitmask flags)
{
	flags_ &= ~flags;

	if(flags & TAKE_PIC)
		emit TakingPicture(false);

	if(flags & POLL_LADAR)
		emit PollingLadar(false);
}

//clear to defaults
void CommandHolder::Clear()
{
	front_left_wheel_velocity_ = 0;
	back_left_wheel_velocity_ = 0;
	front_right_wheel_velocity_ = 0;
	back_right_wheel_velocity_ = 0;

	bucket_pitch_control_ = 0;
	bucket_mine_control_ = 0;

	flags_ = 0;	
	
	next_operation_mode_ = AUTONOMOUS;
}

//send command function
std::string CommandHolder::SendCommand()
{
	if((flags_ & TAKE_PIC) == TAKE_PIC)
	{
		flags_ &= ~TAKE_PIC;		//Reset flag
		// poll a new pic
	}

	if((flags_ & POLL_LADAR) == POLL_LADAR)
	{
		flags_ &= ~POLL_LADAR;		//Reset Flag
		// poll ladar data
	}

	if((flags_ & SWITCH_OPERATION_MODE) == SWITCH_OPERATION_MODE)
	{
		flags_ &= ~SWITCH_OPERATION_MODE;	//Reset Flag
		//Switch to new operation mode
	}

	// don't know the format for this yet...
	return std::string("IDK");
}

//output debug info
std::string CommandHolder::DebugOutput()
{
	std::stringstream output;
	output << 
		"front Left wheel velocity: "	<< front_left_wheel_velocity_	<<
		"front right wheel velocity: "	<< front_right_wheel_velocity_	<<
		"back Left wheel velocity: "	<< back_left_wheel_velocity_	<<
		"back right wheel velocity: "	<< back_right_wheel_velocity_	<<

		"\nBucket drum pitch delta: "	<< bucket_pitch_control_		<<
		"\nBucket drum mine delta: "	<< bucket_mine_control_			<<

		"\nTake a picture: "			<< ((flags_ & TAKE_PIC) == TAKE_PIC)								<< 
		"\nPoll LADAR Data: "			<< ((flags_ & POLL_LADAR) == POLL_LADAR)							<<

		"\nSwitching Modes: "			<< ((flags_ & SWITCH_OPERATION_MODE) == SWITCH_OPERATION_MODE)		<<

		"\nNext mode: "					<< next_operation_mode_			<<
		"\n\n";

	return output.str();
}

void CommandHolder::TakePicture()
{
	flags_ |= TAKE_PIC;
}

void CommandHolder::PollLadarData()
{
	flags_ |= POLL_LADAR;
}

/*
	Set Functions
*/
void CommandHolder::SetBucketMineControl(float rate)
{
	rate = ScaleFloat(rate);
	SetBucketMineControl(ScaleInt(rate, kBucketMaxValue));
}

void CommandHolder::SetBucketMineControl(int rate)
{
	rate = ScaleInt(rate, kBucketMaxValue);
	if(rate != bucket_mine_control_)
	{
		bucket_mine_control_ = rate;
		emit BucketMineChanged(rate);
	}
}

void CommandHolder::SetBucketPitchControl(float rate)
{
	rate = ScaleFloat(rate);
	SetBucketPitchControl(ScaleInt(rate, kBucketMaxValue));
}

void CommandHolder::SetBucketPitchControl(int rate)
{
	rate = ScaleInt(rate, kBucketMaxValue);
	if(rate != bucket_pitch_control_)
	{
		bucket_pitch_control_ = rate;
		emit BucketPitchChanged(rate);
	}
}

void CommandHolder::SetLeftWheelsVelocity(float velocity)
{
	velocity = ScaleFloat(velocity);
	SetLeftWheelsVelocity(ScaleInt(velocity, kWheelsMaxValue));
}

void CommandHolder::SetLeftWheelsVelocity(int velocity)
{
	velocity = ScaleInt(velocity, kWheelsMaxValue);
	if(velocity != front_left_wheel_velocity_ || velocity != back_left_wheel_velocity_)
	{
		front_left_wheel_velocity_ = velocity;
		back_left_wheel_velocity_ = velocity;
		emit FrontLeftWheelChanged(velocity);
		emit BackLeftWheelChanged(velocity);
	}
}

void CommandHolder::SetRightWheelsVelocity(float velocity)
{
	velocity = ScaleFloat(velocity);
	SetRightWheelsVelocity(ScaleInt(velocity, kWheelsMaxValue));
}

void CommandHolder::SetRightWheelsVelocity(int velocity)
{
	velocity = ScaleInt(velocity, kWheelsMaxValue);
	if(velocity != front_right_wheel_velocity_ || velocity != back_right_wheel_velocity_)
	{
		front_right_wheel_velocity_ = velocity;
		back_right_wheel_velocity_ = velocity;
		emit FrontRightWheelChanged(velocity);
		emit BackRightWheelChanged(velocity);
	}
}

// back left wheel
void CommandHolder::SetBackLeftWheelVelocity(float velocity)
{
	velocity = ScaleFloat(velocity);
	SetBackLeftWheelVelocity(ScaleInt(velocity, kWheelsMaxValue));
}

void CommandHolder::SetBackLeftWheelVelocity(int velocity)
{
	velocity = ScaleInt(velocity, kWheelsMaxValue);
	if(velocity != back_left_wheel_velocity_)
	{
		back_left_wheel_velocity_ = velocity;
		emit BackLeftWheelChanged(velocity);
	}
}

// back right wheel
void CommandHolder::SetBackRightWheelVelocity(float velocity)
{
	velocity = ScaleFloat(velocity);
	SetBackRightWheelVelocity(ScaleInt(velocity, kWheelsMaxValue));
}

void CommandHolder::SetBackRightWheelVelocity(int velocity)
{
	velocity = ScaleInt(velocity, kWheelsMaxValue);
	if(velocity != back_right_wheel_velocity_)
	{
		back_right_wheel_velocity_ = velocity;
		emit BackRightWheelChanged(velocity);
	}
}

// Front left wheel
void CommandHolder::SetFrontLeftWheelVelocity(float velocity)
{
	velocity = ScaleFloat(velocity);
	SetFrontLeftWheelVelocity(ScaleInt(velocity, kWheelsMaxValue));
}

void CommandHolder::SetFrontLeftWheelVelocity(int velocity)
{
	velocity = ScaleInt(velocity, kWheelsMaxValue);
	if(velocity != front_left_wheel_velocity_)
	{
		front_left_wheel_velocity_ = velocity;
		emit FrontLeftWheelChanged(velocity);
	}
}

// Front right wheel
void CommandHolder::SetFrontRightWheelVelocity(float velocity)
{
	velocity = ScaleFloat(velocity);
	SetFrontRightWheelVelocity(ScaleInt(velocity, kWheelsMaxValue));
}

void CommandHolder::SetFrontRightWheelVelocity(int velocity)
{
	velocity = ScaleInt(velocity, kWheelsMaxValue);
	if(velocity != front_right_wheel_velocity_)
	{
		front_right_wheel_velocity_ = velocity;
		emit FrontRightWheelChanged(velocity);
	}
}

void CommandHolder::SetOperationMode(OperationMode nextMode)
{
	if(nextMode != next_operation_mode_)
	{
		flags_ |= SWITCH_OPERATION_MODE;
		next_operation_mode_ = nextMode;

		emit SwitchingOperationMode(nextMode);
	}
}