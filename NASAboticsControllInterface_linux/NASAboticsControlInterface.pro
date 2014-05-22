#-------------------------------------------------
#
# Project created by QtCreator 2014-01-14T21:04:53
#
#-------------------------------------------------

CONFIG += qt

QT       += core gui \
            network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = NASAboticsControlInterface
TEMPLATE = app


SOURCES += main.cpp\
    CommandHolder.cpp \
    XBoxController.cpp \
    NASAboticsControlInterface.cpp
HEADERS  += \
    InputDevice.h \
    CommandHolder.h \
    InputDeviceEnum.h \
    OperationModeEnum.h \
    XBoxController.h \
    NASAboticsControlInterface.h

FORMS    += nasaboticscontrolinterface.ui

LIBS += -lSDL

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/lib/release/ -lboost_thread
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/lib/debug/ -lboost_thread
else:unix: LIBS += -L$$PWD/../../../../usr/lib/ -lboost_thread

INCLUDEPATH += $$PWD/../../../../usr/include
DEPENDPATH += $$PWD/../../../../usr/include
