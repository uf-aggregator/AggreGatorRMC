#include "NASAboticsControlInterface.h"

NASAboticsControlInterface::NASAboticsControlInterface(QWidget *parent) :
    QWidget(parent),
    input_device_(NULL),
    send_cmd_timer_(NULL),
    send_frequency_(0.10f),
    current_cmd_(NULL),
    running_average_count_(1),
    kXBeeAddress(0xC0A80170),
    kXBeePort(0x2000),
    launchpad_sock_(NULL),
    input_device_thread_(),
    stop_input_thread_(false)
{
    current_cmd_ = new CommandHolder();

    //set up timing variables
    send_cmd_timer_ = new QTimer(this);
    send_cmd_timer_->start((int)(1000.0f / send_frequency_));

    //set up Sockets
    launchpad_sock_ = new QTcpSocket(this);

    //setup GUI
    ui.setupUi(this);

    /*Setup SDL
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
    {
        QMessageBox::warning(this, "SDL_ERROR", SDL_GetError());
    }
    */
}

NASAboticsControlInterface::~NASAboticsControlInterface()
{

    //StopInputThread();

   if(input_device_ != NULL)
        delete input_device_;

    if(current_cmd_ != NULL)
        delete current_cmd_;

    if(launchpad_sock_ != NULL)
    {
        launchpad_sock_->disconnectFromHost();
        delete launchpad_sock_;
    }

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

    /*SDL_Joystick* joystick;
    if(SDL_NumJoysticks() > 0)
    {
        SDL_JoystickEventState(SDL_ENABLE);
        joystick = SDL_JoystickOpen(0);
    }*/

    InputDevice* input_device;
    switch(new_device)
    {
    case INPUT_TYPE_XBOX_1:
        input_device = new XBoxController();
        if(!input_device->isConnected())
        {
            input_device = NULL;
            ui.controller_select_->setCurrentIndex(0);
        }
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

    /*START DEBUG
    std::string numJoysticks = "There are " +
            static_cast<std::ostringstream*>(&(std::ostringstream() << static_cast<int>(SDL_NumJoysticks())))->str() +    //convert int to string
            " joysticks.\n";

    for(int iii = 0; iii < SDL_NumJoysticks(); ++iii)
        numJoysticks += SDL_JoystickName(iii);

    SDL_Joystick* joystick;
    if(SDL_NumJoysticks() > 0)
    {
        SDL_JoystickEventState(SDL_ENABLE);
        joystick = SDL_JoystickOpen(0);
    }

    QMessageBox::information(this, "Debug", numJoysticks.c_str());

    //END DEBUG*/
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

    running_average_count_ = 0;									//reset running average

    std::string debug_output = current_cmd_->DebugOutput();					//output debug info

    QMessageBox::information(this, "Debug", debug_output.c_str());

    std::string output = /*current_cmd_->SendCommand()*/current_cmd_->DebugOutput();			//Set output = to the formated command

    //send the formated command out on the network
    if(launchpad_sock_->state() == QAbstractSocket::ConnectedState)
    {
        launchpad_sock_->write(output.c_str());
    }
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

    //connect/disconect to Launchpad
    QObject::connect(ui.connect_to_launchpad_btn_, SIGNAL(clicked()),
        this, SLOT(LaunchpadBtn()));

    //Connected to launchpad
    QObject::connect(launchpad_sock_, SIGNAL(connected()),
        this, SLOT(ConnectLaunchpad()));

    //disconnect launchpad
    QObject::connect(launchpad_sock_, SIGNAL(disconnected()),
        this, SLOT(DisconnectLaunchpad()));

    //Connect/Disconect to O-droid

    //Socket error
    QObject::connect(launchpad_sock_, SIGNAL(error(QAbstractSocket::SocketError)),
        this, SLOT(SocketError(QAbstractSocket::SocketError)));
}

//used to stop the input thread!
void NASAboticsControlInterface::StopInputThread()
{
    stop_input_thread_ = true;
    input_device_thread_.join();
    stop_input_thread_ = false;
}

//Launchpad button pressed
void NASAboticsControlInterface::LaunchpadBtn()
{
    switch(launchpad_sock_->state())
    {
    case QAbstractSocket::ConnectedState:
        launchpad_sock_->disconnectFromHost();
        break;
    case QAbstractSocket::UnconnectedState:
        launchpad_sock_->connectToHost(kXBeeAddress, kXBeePort);
        break;
    default:
        break;
    }
}

//launchpad connect
void NASAboticsControlInterface::ConnectLaunchpad()
{
    ui.connect_to_launchpad_btn_->setText("Disconnect launchpad");
    QMessageBox::information(this, "Launchpad connection state", "Connected to Launchpad");

}


void NASAboticsControlInterface::DisconnectLaunchpad()
{
    ui.connect_to_launchpad_btn_->setText("Connect launchpad");
    QMessageBox::information(this, "Launchpad connection state", "Disconnected from Launchpad");

}

//Connect to the O-Droid
void NASAboticsControlInterface::OdroidBtn()
{

}

//Socket Communication error
void NASAboticsControlInterface::SocketError(QAbstractSocket::SocketError error)
{
    std::string error_num = "Error number " +
            static_cast<std::ostringstream*>(&(std::ostringstream() << static_cast<int>(error)))->str() +    //convert int to string
            ": ";

    //append appropriate message
    switch(error)
    {
    case QAbstractSocket::ConnectionRefusedError:
        error_num.append("Connection refused or connection time out");
        break;
    case QAbstractSocket::HostNotFoundError:
        error_num.append("Host not found");
        break;
    default:
        error_num.append("See http://qt-project.org/doc/qt-4.8/qabstractsocket.html#SocketError-enum for details");

    }

    //display warning
    QMessageBox::warning(this, "Connection Error", error_num.c_str());

}
