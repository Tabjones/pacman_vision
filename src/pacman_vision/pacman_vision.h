#ifndef _PACMAN_VISION_H_
#define _PACMAN_VISION_H_

#include <QTimer>
#include <QObject>
#include <QApplication>

#include <memory>

//forward declare used classes to ease MOC work and
//to not pollute the header with ifdefs
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
class EstimatorGui;
class TrackerGui;

class PacmanVision: public QObject
{
    Q_OBJECT
    //this file should contain only declarations (cause of Q_OBJECT macro)
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
    ///when box changes
    void onBoxChanged();
    ///when sensor changes
    void onSensorChanged();
    ///when spawnkill estimator is presed
    void onSpawnKillEstimator();
    ///when pose estimation is clicked
    void onPoseEstimation();
private:
    void startChecker();

    QTimer *check_timer;
    std::shared_ptr<pacv::BasicNode> basic_node;
    std::shared_ptr<pacv::Storage> storage;
    std::shared_ptr<pacv::SensorProcessor> sensor;
    std::shared_ptr<pacv::Estimator> estimator;
    std::shared_ptr<BasicNodeGui> basic_gui;
    std::shared_ptr<EstimatorGui> estimator_gui;
    //todo other modules
};
#endif
