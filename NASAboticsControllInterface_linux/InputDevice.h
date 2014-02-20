#ifndef INPUTDEVICE_H
#define INPUTDEVICE_H

//Controller base class

/*
    This class is the base for all controllers
*/

#include "CommandHolder.h"

class InputDevice
{

public:

    /*
        Virtual Functions
    */
    virtual CommandHolder* GetInput() = 0;

    virtual bool isConnected()
    {
        return is_connected_;
    }

protected:
    //If connected
    bool is_connected_;

    /*
        Constructors & Destructors
    */
    InputDevice() : is_connected_(false) {}

};

#endif // INPUTDEVICE_H
