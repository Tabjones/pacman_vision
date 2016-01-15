#ifndef _PACMAN_VISION_H_
#define _PACMAN_VISION_H_

//include communication services
#include <pacman_vision_comm/estimate.h>
#include <pacman_vision_comm/stop_track.h>
#include <pacman_vision_comm/track_object.h>
#include <pacman_vision_comm/get_scene.h>
#include <pacman_vision_comm/grasp_verification.h>
#include <pacman_vision_comm/get_cloud_in_hand.h>
#include <pacman_vision_comm/start_modeler.h>
#include <pacman_vision_comm/stop_modeler.h>
//Qt
#include <QTimer>
#include <QObject>
#include <QApplication>

#include <memory>

//forward declare used classes to ease MOC work and
//to not pollute the header with ifdefs
namespace pacv
{
class Servicer;
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
    ///when spawnkill tracker is presed
    void onSpawnKillTracker();
    ///On update estimated objects
    void onObjsRefresh();
    ///when pose estimation is clicked
    void onPoseEstimation();
    ///after service is finished
    void postPoseEstimation();
    ///when save cloud is clicked
    void onSaveCloud(std::string *fname);
    ///after service is finished
    void postSaveCloud();
    ///when start tracker is clicked
    void onTrackObject(std::string *obj);
    ///after service is finished
    void postTrackObject();
    ///when stop tracker is clicked
    void onStopTrack();
    ///after service is finished
    void postStopTrack();
private:
    void initConnections();
    QTimer *service_timer;
    QTimer *check_timer;
    //services
    pacman_vision_comm::estimate srv_estimate;
    pacman_vision_comm::get_scene srv_get_scene;
    pacman_vision_comm::track_object srv_track_obj;
    pacman_vision_comm::stop_track srv_stop_track;
    //modules and their gui
    std::shared_ptr<pacv::Servicer> service_caller;
    std::shared_ptr<pacv::BasicNode> basic_node;
    std::shared_ptr<pacv::Storage> storage;
    std::shared_ptr<pacv::SensorProcessor> sensor;
    std::shared_ptr<pacv::Estimator> estimator;
    std::shared_ptr<pacv::Tracker> tracker;
    std::shared_ptr<BasicNodeGui> basic_gui;
    std::shared_ptr<EstimatorGui> estimator_gui;
    std::shared_ptr<TrackerGui> tracker_gui;
    //todo other modules
};
#endif
