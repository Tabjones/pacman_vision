#-------------------------------------------------
#
# Project created by QtCreator 2015-12-09T16:43:52
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = gui
TEMPLATE = app

INCLUDEPATH += $$PWD/../../node/include/

QMAKE_CXXFLAGS += -std=c++11

SOURCES +=\
        mainwindow.cpp \
        dummy_test_main.cpp \
        $$PWD/../../node/src/box.cpp

HEADERS  += mainwindow.h module_config.h box.h

FORMS    += mainwindow.ui

RESOURCES += \
    ../resources/resources.qrc
