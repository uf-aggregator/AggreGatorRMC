#include "NASAboticsControlInterface.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    NASAboticsControlInterface GUI;
    GUI.ConnectWidgets();
    GUI.show();
    return app.exec();
}
