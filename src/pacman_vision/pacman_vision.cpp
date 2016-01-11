#include <pacv_config.h>
#include <pacman_vision.h>
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
// #include <recognition/tracker.h>
#include <estimator_gui.h>
// #include <tracker_gui.h>
#include <pacman_vision_comm/estimate.h>
#endif

#include <ros/ros.h>

PacmanVision::PacmanVision():
    QObject()
{}

PacmanVision::~PacmanVision()
{
    delete check_timer;
    sensor->kill();
    basic_node->kill();
    //...
    ros::shutdown();
}

void
PacmanVision::startChecker()
{
    check_timer = new QTimer(this);
    connect (check_timer, SIGNAL( timeout() ), this, SLOT( checkROS() ));
    check_timer->start(200); //check every 200ms
    connect (&(*basic_gui), SIGNAL( boxChanged()), this, SLOT( onBoxChanged() ));
    connect (&(*basic_gui), SIGNAL( sensorChanged()), this, SLOT( onSensorChanged() ));
#ifdef PACV_RECOGNITION_SUPPORT
    connect (estimator_gui->getRunButt(), SIGNAL( clicked()), this, SLOT( onSpawnKillEstimator() ));
    connect (estimator_gui->getEstButt(), SIGNAL( clicked()), this, SLOT( onPoseEstimation() ));
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
        sensor = std::make_shared<pacv::SensorProcessor>(basic_node->getNodeHandle(), "sensor", storage);
        sensor->setRate(40.0); //40hz
        sensor->spawn();
        //spawn basic node for last since it needs SensorProcessor
        basic_node->spawn();
        basic_gui = std::make_shared<BasicNodeGui>(basic_node->getConfig(), sensor->getConfig());
#ifdef PACV_RECOGNITION_SUPPORT
        ROS_INFO("[PaCMan Vision]\tAdding Estimator Module");
        estimator = std::make_shared<pacv::Estimator>(basic_node->getNodeHandle(), "estimator", storage);
        estimator->setRate(5.0); //5Hz is enough
        estimator_gui = std::make_shared<EstimatorGui>(estimator->getConfig());
        basic_gui->addTab(estimator_gui->getWidget(), "Estimator Module");
#endif
        //...

        startChecker();
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
PacmanVision::onPoseEstimation()
{
#ifdef PACV_RECOGNITION_SUPPORT
    pacman_vision_comm::estimate::Request req;
    pacman_vision_comm::estimate::Response res;
    std::string name(estimator->getNamespace());
    name += "/estimate";
    ros::service::call(name, req, res);
    estimator_gui->getEstButt()->setDisabled(false);
#endif
}
