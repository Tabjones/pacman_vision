#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <pacman_vision/module_config.h>
#include <pacman_vision/basic_node.h>
#include <pacman_vision/sensor_processor.h>
#include <pacman_vision/estimator.h>
#include <mainwindow.h>
#include <QApplication>
#include <QTimer>

class Application: public QApplication
{
    Q_OBJECT

    public:
    explicit Application(int & argc, char **argv);
    ~Application();

    void showGui();
    void startUpdating(int msecs);
private slots:
    void update();
private:
    QTimer *timer;
    MainWindow *gui;
    Storage::Ptr storage;
    BasicNode::Ptr node;
    SensorProcessor::Ptr sensor;
    Estimator::Ptr estimator;

    void init();
};
#endif
