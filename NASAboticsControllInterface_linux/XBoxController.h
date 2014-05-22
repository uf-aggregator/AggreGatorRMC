/*
    Implements an X Box controller as a input device
*/
#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#include "InputDevice.h"
//#include <Windows.h>      //required for Xinput
//#include <Xinput.h>		//library for working with Xbox controllers
#include <cmath>            //for math... duh?

//Linux commands
#include <linux/input.h>

//SDL
#include<SDL/SDL.h>
#include<SDL/SDL_joystick.h>


//#pragma comment(lib, "XInput.lib")

class XBoxController: public InputDevice
{

private:
    /*
     *      Private Enumurators
     */
    enum SDL_360_INPUT_MAP
    {
        LEFT_STICK_LR,
        LEFT_STICK_UD,
        LEFT_TRIGGER,
        RIGHT_STICK_LR,
        RIGHT_STICK_UD,
        RIGHT_TRIGGER
    };

    /*
        Private variables
    */
private:
    bool mine_;				//mine or dump
    bool lift_bucket_;		//lift or lower bucket drum

    bool right_shoulder_pressed_;		//remebers if right sholder button is pressed
    bool left_shoulder_pressed_;			//remembers if left sholder button is pressed

    static const long kMaxThumstickValue = 32767;
    static const int kMaxTriggerValue = 255;
    static const int kDeadzone = 4800;				//determined by testing one controller (varies based on controller)

    //SDL joystick
    SDL_Joystick* joy_;
    /*
        Constructors and Destructors
    */
public:
    XBoxController();

    ~XBoxController();

    /*
        Function to obtain input from the xBox controller
    */
public:
    virtual CommandHolder* GetInput();
};

#endif
