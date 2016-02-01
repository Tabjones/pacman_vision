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
class ListenerGui;
class InHandModelerGui;

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
    //when enable/disable button is pressed
    void onEnableDisable(bool enable);
    //when master reset is pressed
    void onReset();
    ///Shutdown cleanup
    void onShutdown();
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
    //when spawnkill listener is pressed
    void onSpawnKillListener();
    //when getCloudInHand is clicked
    void onSaveInHand(bool right, std::string *obj, std::string *hand);
    //after service is finished
    void postSaveInHand();
    //save configuration
    void onSaveConf(std::string* n);
    //load configuration
    void onLoadConf(std::string* n);
private:
    void initConnections();
    QTimer *service_timer;
    QTimer *check_timer;
    //services
    pacman_vision_comm::estimate srv_estimate;
    pacman_vision_comm::get_scene srv_get_scene;
    pacman_vision_comm::track_object srv_track_obj;
    pacman_vision_comm::stop_track srv_stop_track;
    pacman_vision_comm::get_cloud_in_hand srv_get_in_hand;
    //modules and their gui
    std::shared_ptr<pacv::Servicer> service_caller;
    std::shared_ptr<pacv::BasicNode> basic_node;
    std::shared_ptr<pacv::Storage> storage;
    std::shared_ptr<pacv::SensorProcessor> sensor;
    std::shared_ptr<pacv::Estimator> estimator;
    std::shared_ptr<pacv::Tracker> tracker;
    std::shared_ptr<pacv::Listener> listener;
    std::shared_ptr<pacv::InHandModeler> modeler;
    std::shared_ptr<BasicNodeGui> basic_gui;
    std::shared_ptr<EstimatorGui> estimator_gui;
    std::shared_ptr<TrackerGui> tracker_gui;
    std::shared_ptr<ListenerGui> listener_gui;
    std::shared_ptr<InHandModelerGui> modeler_gui;
};
#endif
