#ifndef NASABOTICSCONTROLINTERFACE_H
#define NASABOTICSCONTROLINTERFACE_H

//UI includes
#include <QtWidgets/QWidget>
#include "ui_nasaboticscontrolinterface.h"

//Functionality includes
#include "CommandHolder.h"
#include "InputDevice.h"
#include <boost/thread.hpp>
#include "InputDeviceEnum.h"
#include "XBoxController.h"
#include <qtimer.h>
#include <qmessagebox.h>
#include <QtNetwork/qtcpsocket.h>
#include <QtNetwork/qhostaddress.h>
#include <sstream>
#include <string>

//SDL
#include<SDL/SDL.h>
#include<SDL/SDL_joystick.h>

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

    //Sockets
    //Launchpad socket
    QHostAddress kXBeeAddress;                  //192.168.1.112 (C0.A8.01.70)
    qint16 kXBeePort;                           //0x2000
    QTcpSocket* launchpad_sock_;				//a socket to connect to the X-bee attached to the launchpad

    //A thread to manage the input device
    boost::thread input_device_thread_;
    bool stop_input_thread_;				//if true it will stop the input thread
    void StopInputThread();					//function to stop the input thread

    //GUI stuff
    Ui::NASAboticsControlInterfaceClass ui;		//GUI stuff


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


public slots:
    void SetSendRate(double frequency);			//sets the frequency of sending cmds to the NASAbot
    void AttachNewController(int new_device);	//Takes an enum value and attaches the appropriate controller
    void LaunchpadBtn();						//When connect to launchpad is pressed this will ether connect or disconect launchpad
    void OdroidBtn();							//When launchpad connect btn is pressed this will ether connect or disconect Odroid

    //Write command
    void WriteCommands();

    //socket slots
    void ConnectLaunchpad();                                    //Changes settings when it connects to Launchpad
    void DisconnectLaunchpad();                                 //chane settings when disconnect from launcpad
    void SocketError(QAbstractSocket::SocketError error);		//Envoked when a socket has a communication error

};

#endif // NASABOTICSCONTROLINTERFACE_H
