#include <pacv_config.h>
#include <pacman_vision.h>
#include <servicer.hpp>
#include <basic_node/basic_node.h>
#include <basic_node/sensor_processor.h>
#include <common/storage.h>
#include <common/common_std.h>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <basic_node_gui.h>
#include <QPushButton>

//Add recognition support
#ifdef PACV_RECOGNITION_SUPPORT
#include <recognition/estimator.h>
#include <recognition/tracker.h>
#include <estimator_gui.h>
#include <tracker_gui.h>
#endif

#include <ros/ros.h>

PacmanVision::PacmanVision():
    QObject()
{}

PacmanVision::~PacmanVision()
{
    delete check_timer;
    delete service_timer;
    sensor->kill();
    basic_node->kill();
    //...
    ros::shutdown();
}

void
PacmanVision::initConnections()
{
    check_timer = new QTimer(this);
    connect (check_timer, SIGNAL( timeout() ), this, SLOT( checkROS() ));
    check_timer->start(200); //check every 200ms
    connect (&(*basic_gui), SIGNAL( boxChanged()), this, SLOT( onBoxChanged() ));
    connect (&(*basic_gui), SIGNAL( sensorChanged()), this, SLOT( onSensorChanged() ));
    connect (&(*basic_gui), SIGNAL( saveCloud(std::string*) ), this, SLOT( onSaveCloud(std::string*) ));
#ifdef PACV_RECOGNITION_SUPPORT
    connect (estimator_gui->getRunButt(), SIGNAL( clicked()), this, SLOT( onSpawnKillEstimator() ));
    connect (estimator_gui->getEstButt(), SIGNAL( clicked()), this, SLOT( onPoseEstimation() ));
    connect (tracker_gui->getRunButt(), SIGNAL( clicked()), this, SLOT( onSpawnKillTracker() ));
#endif
}

void PacmanVision::onBoxChanged()
{
    basic_node->update_markers();
}

void PacmanVision::onSensorChanged()
{
    sensor->update();
}

void PacmanVision::onSpawnKillEstimator()
{
#ifdef PACV_RECOGNITION_SUPPORT
    if (estimator->isRunning()){
        estimator->kill();
        estimator_gui->setRunning(false);
        ROS_INFO("[PaCMan Vision]\tKilled Estimator Module.");
        return;
    }
    else{
        estimator->spawn();
        estimator_gui->setRunning(true);
        ROS_INFO("[PaCMan Vision]\tSpawned Estimator Module.");
        return;
    }
#endif
}

void PacmanVision::onSpawnKillTracker()
{
#ifdef PACV_RECOGNITION_SUPPORT
    if (tracker->isRunning()){
        tracker->kill();
        tracker_gui->setRunning(false);
        ROS_INFO("[PaCMan Vision]\tKilled Tracker Module.");
        return;
    }
    else{
        tracker->spawn();
        tracker_gui->setRunning(true);
        ROS_INFO("[PaCMan Vision]\tSpawned Tracker Module.");
        return;
    }
#endif
}

bool
PacmanVision::init(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "pacman_vision");
        ROS_INFO("[PaCMan Vision]\tInitializing...");
        storage = std::make_shared<pacv::Storage>();
        basic_node = std::make_shared<pacv::BasicNode>("pacman_vision", storage);
        basic_node->setRate(40.0); //40Hz
        basic_node->spawn();
        sensor = std::make_shared<pacv::SensorProcessor>(basic_node->getNodeHandle(), "sensor", storage);
        sensor->setRate(40.0); //40hz
        sensor->spawn();
        basic_gui = std::make_shared<BasicNodeGui>(basic_node->getConfig(), sensor->getConfig());
        //allocate the async service caller
        service_caller = std::make_shared<pacv::Servicer>(basic_node->getNodeHandle(), "service_caller");
        service_timer = new QTimer(this);
#ifdef PACV_RECOGNITION_SUPPORT
        ROS_INFO("[PaCMan Vision]\tAdding Estimator Module");
        estimator = std::make_shared<pacv::Estimator>(basic_node->getNodeHandle(), "estimator", storage);
        estimator->setRate(5.0); //5Hz is enough
        estimator_gui = std::make_shared<EstimatorGui>(estimator->getConfig());
        basic_gui->addTab(estimator_gui->getWidget(), "Estimator Module");
        ROS_INFO("[PaCMan Vision]\tAdding Tracker Module");
        tracker = std::make_shared<pacv::Tracker>(basic_node->getNodeHandle(), "tracker", storage);
        tracker->setRate(40.0); //40Hz is enough
        tracker->setBasicNodeConfig(basic_node->getConfig());
        tracker_gui = std::make_shared<TrackerGui>(tracker->getConfig());
        basic_gui->addTab(tracker_gui->getWidget(), "Tracker Module");
#endif
        //...

        initConnections();
        ROS_INFO("[PaCMan Vision]\tShowing Gui(s)...");
        basic_gui->show();
    }
    catch(...)
    {
        ROS_ERROR("[PaCMan Vision]\tError while initializing...");
        return false;
    }
    return true;
}

void
PacmanVision::checkROS()
{
    if (!ros::ok()){
        check_timer->stop();
        QApplication::closeAllWindows();
    }
}

void
PacmanVision::onSaveCloud(std::string *fname)
{
    std::string srv_name(basic_node->getNamespace());
    srv_name += "/get_scene_processed";
    srv_get_scene.request.save = *fname;
    service_caller->spawn(srv_get_scene, srv_name);
    connect (service_timer, SIGNAL( timeout() ), this, SLOT( postSaveCloud() ));
    service_timer->start(200);
}
void
PacmanVision::postSaveCloud()
{
    if (service_caller->postCallClean()){
        //service was called and it is finished
        service_timer->disconnect(SIGNAL( timeout() ));
        service_timer->stop();
        //srv_estimate has the response, but we dont care about it
        //reactivate the gui button
        basic_gui->getSaveButt()->setDisabled(false);
    }
}

void
PacmanVision::onPoseEstimation()
{
#ifdef PACV_RECOGNITION_SUPPORT
    std::string srv_name(estimator->getNamespace());
    srv_name += "/estimate";
    service_caller->spawn(srv_estimate, srv_name);
    connect (service_timer, SIGNAL( timeout() ), this, SLOT( postPoseEstimation() ));
    service_timer->start(500);
#endif
}
void
PacmanVision::postPoseEstimation()
{
#ifdef PACV_RECOGNITION_SUPPORT
    if (service_caller->postCallClean()){
        //service was called and it is finished
        service_timer->disconnect(SIGNAL( timeout() ));
        service_timer->stop();
        //srv_estimate has the response, but we dont care about it
        //reactivate the gui button
        estimator_gui->getEstButt()->setDisabled(false);
        estimator_gui->getEstButt()->setText("Pose Estimation");
    }
#endif
}
