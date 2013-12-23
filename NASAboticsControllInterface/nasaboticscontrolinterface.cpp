#include "nasaboticscontrolinterface.h"

NASAboticsControlInterface::NASAboticsControlInterface(QWidget *parent) :
	QWidget(parent), input_device_(NULL), input_device_thread_(),
	send_frequency_(0.10f), send_cmd_timer_(NULL),
	running_average_count_(1), stop_input_thread_(false)
{

	current_cmd_ = new CommandHolder();

	//set up timing variables
	send_cmd_timer_ = new QTimer(this);
	send_cmd_timer_->start((int)(1000.0f / send_frequency_));

	//setup GUI
	ui.setupUi(this);
}

NASAboticsControlInterface::~NASAboticsControlInterface()
{
	StopInputThread();

	if(input_device_ != NULL)
		delete input_device_;

	if(current_cmd_ != NULL)
		delete current_cmd_;
}

/*
	This function polls input from the attached controller
*/
void NASAboticsControlInterface::PollInput()
{
	while(input_device_ != NULL && !stop_input_thread_)
	{
		CommandHolder* newCmds = input_device_->GetInput();		//Get input from the attached controller

		if(running_average_count_ != 0)
		{
			current_cmd_->RunningAverage(newCmds, running_average_count_);		//average all commands since last send
		}
		else
		{
			current_cmd_->CopyValues(newCmds);		//copy commands in
		}

		delete newCmds;

		++running_average_count_;
	}
}

//attach a new input device from an enum
void NASAboticsControlInterface::AttachNewController(int new_device)
{
	InputDevice* input_device;
	switch(new_device)
	{
	case INPUT_TYPE_XBOX_1:
		input_device = new XBoxController();
		break;

	default:
		input_device = NULL;
		break;
	}

	if(input_device == NULL)
	{
		StopInputThread();
		if(input_device_ != NULL)
		{
			delete input_device_;
			input_device_ = NULL;
		}
	}
	else
	{
		AttachNewController(input_device);
	}
}

//attach a new input device
void NASAboticsControlInterface::AttachNewController(InputDevice* input_device)
{
	//stop the current input thread
	StopInputThread();

	//delete old controller if not the same
	if(input_device_ != NULL)
	{
		delete input_device_;
		input_device_ = NULL;
	}

	//add new input device
	input_device_ = input_device;

	//set up input device thread
	//calls the function PollInput passing the implicit this to the funciton
	input_device_thread_ = boost::thread(&NASAboticsControlInterface::PollInput, this);
}

//Write out the commands to be sent to communicator
void NASAboticsControlInterface::WriteCommands()
{
	running_average_count_ = 0;								//reset running average

	//std::cout << current_cmd.DebugOutput();					//output debug info
	std::string output = current_cmd_->SendCommand();			//Set output = to the formated command

	//send the formated command out on the network
}

void NASAboticsControlInterface::SetSendRate(double frequency)
{
	send_frequency_ = frequency;
	if(frequency != 0)
		send_cmd_timer_->start((int)(1000.0f/send_frequency_));
	else
		send_cmd_timer_->stop();
}

//connect the current comand with the widgits
void NASAboticsControlInterface::ConnectWidgets()
{
	//attach timer to write commands
	QObject::connect(send_cmd_timer_, SIGNAL(timeout()),
					this, SLOT(WriteCommands()));

	//atatch spinbox to send frequency
	QObject::connect(ui.send_frequency_spinbox_, SIGNAL(valueChanged(double)), 
			this, SLOT(SetSendRate(double)));

	//attach controller dropdown to attach controller function
	QObject::connect(ui.controller_select_, SIGNAL(currentIndexChanged(int)),
					this, SLOT(AttachNewController(int)));

	//Back left wheel to slider
	QObject::connect(ui.back_left_wheel_slider_, SIGNAL(valueChanged(int)),
					current_cmd_, SLOT(SetBackLeftWheelVelocity(int)));

	QObject::connect(current_cmd_, SIGNAL(BackLeftWheelChanged(int)),
		ui.back_left_wheel_slider_, SLOT(setValue(int)));
	
	//Back right wheel to slider
	QObject::connect(ui.back_right_wheel_slider_, SIGNAL(valueChanged(int)),
					current_cmd_, SLOT(SetBackRightWheelVelocity(int)));

	QObject::connect(current_cmd_, SIGNAL(BackRightWheelChanged(int)),
		ui.back_right_wheel_slider_, SLOT(setValue(int)));
	
	//Front left wheel to slider
	QObject::connect(ui.front_left_wheel_slider_, SIGNAL(valueChanged(int)),
					current_cmd_, SLOT(SetFrontLeftWheelVelocity(int)));

	QObject::connect(current_cmd_, SIGNAL(FrontLeftWheelChanged(int)),
		ui.front_left_wheel_slider_, SLOT(setValue(int)));
	
	//front right wheel to slider
	QObject::connect(ui.front_right_wheel_slider_, SIGNAL(valueChanged(int)),
					current_cmd_, SLOT(SetFrontRightWheelVelocity(int)));

	QObject::connect(current_cmd_, SIGNAL(FrontRightWheelChanged(int)),
		ui.front_right_wheel_slider_, SLOT(setValue(int)));

	//bucket_pitch_control to slider
	QObject::connect(ui.bucket_pitch_control_slider_, SIGNAL(valueChanged(int)),
		current_cmd_, SLOT(SetBucketPitchControl(int)));

	QObject::connect(current_cmd_, SIGNAL(BucketPitchChanged(int)),
		ui.bucket_pitch_control_slider_, SLOT(setValue(int)));

	//bucket_mine_control to slider
	QObject::connect(ui.bucket_mine_control_slider_, SIGNAL(valueChanged(int)),
		current_cmd_, SLOT(SetBucketMineControl(int)));

	QObject::connect(current_cmd_, SIGNAL(BucketMineChanged(int)),
		ui.bucket_mine_control_slider_, SLOT(setValue(int)));

	//poll ladar
	QObject::connect(ui.poll_ladar_button_, SIGNAL(clicked()),
		current_cmd_, SLOT(PollLadarData()));

	QObject::connect(ui.take_picture_button_, SIGNAL(clicked()),
		current_cmd_, SLOT(TakePicture()));
}

//used to stop the input thread!
void NASAboticsControlInterface::StopInputThread()
{
	stop_input_thread_ = true;
	input_device_thread_.join();
	stop_input_thread_ = false;
}