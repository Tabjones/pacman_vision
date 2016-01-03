#include <pacv_config.h>
#include <pacman_vision.h>
#include <basic_node/basic_node.h>
#include <basic_node/sensor_processor.h>
#include <common/storage.h>
#include <common/common_std.h>
#include <common/common_pcl.h>
#include <common/common_ros.h>
#include <ros/ros.h>

PacmanVision::PacmanVision():
    QObject()
{}

PacmanVision::~PacmanVision()
{
    delete check_timer;
}

void
PacmanVision::startChecker()
{
    check_timer = new QTimer(this);
    connect (check_timer, SIGNAL( timeout() ), this, SLOT( checkROS() ));
    check_timer->start(200); //check every 200ms
}

bool
PacmanVision::init(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "pacman_vision");
        ROS_INFO("PaCMan Vision Initializing...");
        storage = std::make_shared<pacv::Storage>();
        basic_node = std::make_shared<pacv::BasicNode>("pacman_vision", storage);
        basic_node->setRate(40.0); //40Hz
        basic_node->spawn();
        sensor = std::make_shared<pacv::SensorProcessor>(basic_node->getNodeHandle(), "sensor", storage);
        sensor->setRate(40.0); //40hz
        sensor->spawn();
        //todo
    }
    catch(...)
    {
        ROS_ERROR("Error while initializing...");
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
//
// void
// Application::update()
// {
//     if (!node->isRunning())
//         qApp->quit();
//     if (gui->masterReset())
//     {
//         node->updateIfNeeded(BasicNode::ConfigPtr(), true);
//         sensor->updateIfNeeded(SensorProcessor::ConfigPtr(), true);
//         estimator->updateIfNeeded(Estimator::ConfigPtr(), true);
//         gui->configure(node->getConfig());
//         gui->configure(sensor->getConfig());
//         gui->configure(estimator->getConfig());
//         return;
//     }
//     if(gui->isDisabled()){
//         if(!node->isDisabled())
//             node->disable();
//         if(!sensor->isDisabled())
//             sensor->disable();
//         if(!estimator->isDisabled())
//             estimator->disable();
//         return;
//     }
//     if(node->isDisabled())
//         node->enable();
//     if(sensor->isDisabled())
//         sensor->enable();
//     if(estimator->isDisabled())
//         estimator->enable();
//     node->updateIfNeeded(gui->getBaseConfig());
//     sensor->updateIfNeeded(gui->getSensorConfig());
//     estimator->updateIfNeeded(gui->getEstimatorConfig());
//     if (estimator->isRunning() && !gui->getEstimatorConfig()->spawn)
//         estimator->kill();
//     else if (!estimator->isRunning() && gui->getEstimatorConfig()->spawn)
//         estimator->spawn();
// }
