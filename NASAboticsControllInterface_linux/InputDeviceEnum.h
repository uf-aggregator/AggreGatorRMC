#ifndef INPUTDEVICEENUM_H
#define INPUTDEVICEENUM_H

#include <qstring.h>

enum InputDeviceIndex
{
    INPUT_TYPE_DESKTOP		=			0,
    INPUT_TYPE_XBOX_1,

    NUM_CONTROLLER_TYPES
};

/*QString InputDeviceIndexToString(InputDeviceIndex index)
{
    QString out;
    switch(index)
    {
    case INPUT_TYPE_DESKTOP:
        out = "None";
        break;
    case INPUT_TYPE_XBOX_1:
        out = "XBox (type 1)";
        break;
    default:
        out = "???";
    }

    return out;
}
*/
#endif
