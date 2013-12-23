#ifndef NASABOTICSCONTROLINTERFACE_H
#define NASABOTICSCONTROLINTERFACE_H

//UI includes
#include <QtWidgets/QWidget>
#include "ui_nasaboticscontrolinterface.h"

//Functionality includes
#include "CommandHolder.h"
#include "InputDevice.h"
#include <boost\thread.hpp>
#include "InputDeviceEnum.h"
#include "XBoxController.h"
#include <qtimer.h>

class NASAboticsControlInterface : public QWidget
{
	Q_OBJECT			//Macro (really long macro!) required by Qt

/*
	Private Variables
*/
private:
	InputDevice* input_device_;			//Manages the input device

	//timer data
	QTimer* send_cmd_timer_;			//timer for sending data
	float send_frequency_;				//Length of time between polls


	//command stuff
	CommandHolder* current_cmd_;					//holds the current command until it is writen out
	unsigned int running_average_count_;			//keeps count for adding up all of the cmds
												//between sends
	//GUI stuff
	Ui::NASAboticsControlInterfaceClass ui;		//GUI stuff (honestly IDK what this is yet)

public:
	NASAboticsControlInterface(QWidget *parent = 0);
	~NASAboticsControlInterface();	

/*
	Functionality Functions
*/
public:
	//Connect the widgets with the current_cmd and variables
	void ConnectWidgets();

	//Polls the current input device for new input
	void PollInput();

	//Switches the controller
	void AttachNewController(InputDevice* input_device);

	//A thread to manage the input device
private:
	boost::thread input_device_thread_;
	bool stop_input_thread_;				//if true it will stop the input thread
	void StopInputThread();					//function to stop the input thread

public slots:
	void SetSendRate(double frequency);			//sets the frequency of sending cmds to the NASAbot
	void AttachNewController(int new_device);	//Takes an enum value and attaches the appropriate controller

	//Write command
	void WriteCommands();
};

#endif // NASABOTICSCONTROLINTERFACE_H
