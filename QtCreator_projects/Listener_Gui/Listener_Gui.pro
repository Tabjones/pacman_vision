#-------------------------------------------------
#
# Project created by QtCreator 2016-01-04T13:42:42
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = dummy_qtcreator_main
TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += ../../include\
               ../../src/listener/gui\
                /opt/ros/indigo/include\
               ../../../../devel/include #For this to work, pacman vision has to be built with catkin at least once.

SOURCES += ../../src/listener/gui/dummy_qtcreator_main.cpp\
           ../../src/listener/gui/listener_gui.cpp\
           ../../src/common/box.cpp

HEADERS  += ../../src/listener/gui/listener_gui.h\

FORMS    += ../../src/listener/gui/listener_gui.ui\

RESOURCES += ../../resources/resources.qrc
