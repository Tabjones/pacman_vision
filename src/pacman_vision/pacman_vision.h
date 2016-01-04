#ifndef _PACMAN_VISION_H_
#define _PACMAN_VISION_H_

#include <QTimer>
#include <QObject>
#include <QApplication>

#include <memory>

namespace pacv
{
class Storage;
class BasicNode;
class SensorProcessor;
class Estimator;
class Tracker;
class Listener;
class InHandModeler;
}

class BasicNodeGui;

class PacmanVision: public QObject
{
    Q_OBJECT
    //this file must contain only declarations (cause of Q_OBJECT macro)
    public:
    explicit PacmanVision();
    ~PacmanVision();

    /** Start everything !
     * @return true on success, false otherwise
     */
    bool init(int argc, char** argv);

private slots:
    /** Check if ros::ok() is false, if it is, just quit. */
    void checkROS();
private:
    void startChecker();

    QTimer *check_timer;
    std::shared_ptr<pacv::BasicNode> basic_node;
    std::shared_ptr<pacv::Storage> storage;
    std::shared_ptr<pacv::SensorProcessor> sensor;
    std::shared_ptr<BasicNodeGui> basic_gui;
    // std::shared_ptr<Estimator> estimator;
    //todo other modules
};
#endif
