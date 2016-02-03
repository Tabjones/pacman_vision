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
               ../../src/recognition/gui\
                /opt/ros/indigo/include\
               ../../../../devel/include #For this to work, pacman vision has to be built with catkin at least once.

SOURCES += ../../src/recognition/gui/dummy_qtcreator_main.cpp\
           ../../src/recognition/gui/estimator_gui.cpp\
           ../../src/recognition/gui/tracker_gui.cpp\
           ../../src/common/box.cpp

HEADERS  += ../../src/recognition/gui/estimator_gui.h\
            ../../src/recognition/gui/tracker_gui.h

FORMS    += ../../src/recognition/gui/estimator_gui.ui\
            ../../src/recognition/gui/tracker_gui.ui

RESOURCES += ../../resources/resources.qrc
