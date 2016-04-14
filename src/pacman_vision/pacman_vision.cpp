// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <cstdlib>

//Add recognition support
#ifdef PACV_RECOGNITION_SUPPORT
#include <recognition/estimator.h>
#include <recognition/tracker.h>
#include <estimator_gui.h>
#include <tracker_gui.h>
#endif

//add listener support
#ifdef PACV_LISTENER_SUPPORT
#include <listener/listener.h>
#include <listener_gui.h>
#endif

//add modeler support
#ifdef PACV_MODELER_SUPPORT
#include <modeler/modeler.h>
// #include <modeler_gui.h>
#endif

#include <ros/ros.h>

PacmanVision::PacmanVision():
    QObject()
{}

PacmanVision::~PacmanVision()
{
    delete check_timer;
    delete service_timer;
}

void
PacmanVision::onShutdown()
{
    check_timer->stop();
    service_timer->stop();
    sensor->kill();
#ifdef PACV_RECOGNITION_SUPPORT
    if (estimator->isRunning())
        estimator->kill();
    if (tracker->isRunning())
        tracker->kill();
#endif
#ifdef PACV_LISTENER_SUPPORT
    if (listener->isRunning())
        listener->kill();
#endif
    basic_node->kill();
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
    connect (&(*basic_gui), SIGNAL( enableDisable(bool) ), this, SLOT( onEnableDisable(bool) ));
    connect (&(*basic_gui), SIGNAL( reset() ), this, SLOT( onReset() ));
    connect (&(*basic_gui), SIGNAL( saveConf(std::string*) ), this, SLOT(onSaveConf(std::string*) ));
    connect (&(*basic_gui), SIGNAL( loadConf(std::string*) ), this, SLOT(onLoadConf(std::string*) ));
    connect (&(*basic_gui), SIGNAL( colorChange(double, double, double) ), this, SLOT(onColorChange(double,double,double) ));
#ifdef PACV_RECOGNITION_SUPPORT
    connect (estimator_gui->getRunButt(), SIGNAL( clicked()), this, SLOT( onSpawnKillEstimator() ));
    connect (estimator_gui->getEstButt(), SIGNAL( clicked()), this, SLOT( onPoseEstimation() ));
    connect (tracker_gui->getRunButt(), SIGNAL( clicked()), this, SLOT( onSpawnKillTracker() ));
    connect (tracker_gui->getRefreshButt(), SIGNAL( clicked() ), this, SLOT ( onObjsRefresh() ));
    connect (&(*tracker_gui), SIGNAL( trackObject(std::string*) ), this, SLOT ( onTrackObject(std::string*) ));
    connect (tracker_gui->getStopButt(), SIGNAL( clicked() ), this, SLOT ( onStopTrack() ));
#endif
#ifdef PACV_LISTENER_SUPPORT
    connect (listener_gui->getRunButt(), SIGNAL( clicked()), this, SLOT( onSpawnKillListener() ));
    connect (&(*listener_gui), SIGNAL( saveInHand(bool, std::string*, std::string*) ), this, SLOT( onSaveInHand(bool, std::string*, std::string*) ));
#endif
}

void
PacmanVision::onSaveConf(std::string* n)
{
    basic_node->updateRosparams();
    sensor->updateRosparams();
#ifdef PACV_LISTENER_SUPPORT
    if (listener->isRunning())
        listener->updateRosparams();
#endif
#ifdef PACV_RECOGNITION_SUPPORT
    if (estimator->isRunning())
        estimator->updateRosparams();
    if (tracker->isRunning())
        tracker->updateRosparams();
#endif
    std::string command("rosparam dump ");
    command += *n;
    command += " /pacman_vision";
    std::system(command.c_str());
}

void
PacmanVision::onLoadConf(std::string* n)
{
    std::string command("rosparam load ");
    command += *n;
    command += " /pacman_vision";
    std::system(command.c_str());
    onReset();
}

void
PacmanVision::onEnableDisable(bool enable)
{
    sensor->setDisabled(!enable);
    basic_node->setDisabled(!enable);
    sensor->update();
#ifdef PACV_RECOGNITION_SUPPORT
    estimator->setDisabled(!enable);
    tracker->setDisabled(!enable);
    if (enable){
        estimator_gui->enable(true);
        tracker_gui->enable(true);
    }
    else{
        estimator_gui->disable(true);
        tracker_gui->disable(true);
    }
#endif
#ifdef PACV_LISTENER_SUPPORT
    listener->setDisabled(!enable);
    if (enable)
        listener_gui->enable(true);
    else
        listener_gui->disable(true);
#endif
}

void
PacmanVision::onReset()
{
    bool run;
    std::string father_ns = basic_node->getNamespace();
#ifdef PACV_LISTENER_SUPPORT
    if(listener->isRunning()){
        listener->kill();
        listener_gui->disable();
    }
    ros::param::get( (father_ns + "/" + listener->getRelativeName() + "/spawn").c_str(), run);
    if (run)
        onSpawnKillListener();
#endif
#ifdef PACV_RECOGNITION_SUPPORT
    if(estimator->isRunning()){
        estimator->kill();
        estimator_gui->disable();
    }
    ros::param::get( (father_ns + "/" + estimator->getRelativeName() + "/spawn").c_str(), run);
    if(run)
        onSpawnKillEstimator();
    if(tracker->isRunning()){
        tracker->kill();
        tracker_gui->disable();
    }
    ros::param::get( (father_ns + "/" + tracker->getRelativeName() + "/spawn").c_str(), run);
    if (run)
        onSpawnKillTracker();
#endif
    basic_node->setConfigFromRosparams();
    sensor->setConfigFromRosparams();
    basic_gui->init();
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
        estimator_gui->disable();
        ROS_INFO("[PaCMan Vision]\tKilled Estimator Module.");
        tracker_gui->getObjList()->clear();
        tracker_gui->getTrackButt()->setDisabled(true);
        return;
    }
    else{
        estimator->spawn();
        estimator_gui->enable();
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
        tracker_gui->disable();
        ROS_INFO("[PaCMan Vision]\tKilled Tracker Module.");
        return;
    }
    else{
        tracker->spawn();
        tracker_gui->enable();
        ROS_INFO("[PaCMan Vision]\tSpawned Tracker Module.");
        onObjsRefresh();
        return;
    }
#endif
}

void PacmanVision::onSpawnKillListener()
{
#ifdef PACV_LISTENER_SUPPORT
    if (listener->isRunning()){
        listener->kill();
        listener_gui->disable();
        ROS_INFO("[PaCMan Vision]\tKilled Listener Module.");
        return;
    }
    else{
        listener->spawn();
        listener_gui->enable();
        ROS_INFO("[PaCMan Vision]\tSpawned Listener Module.");
        return;
    }
#endif
}

void PacmanVision::onSpawnKillModeler()
{
#ifdef PACV_MODELER_SUPPORT
    if (modeler->isRunning()){
        modeler->kill();
        // modeler_gui->disable();
        ROS_INFO("[PaCMan Vision]\tKilled Modeler Module.");
        return;
    }
    else{
        modeler->spawn();
        // modeler_gui->enable();
        ROS_INFO("[PaCMan Vision]\tSpawned Modeler Module.");
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
        bool run;
        estimator->getConfig()->get("spawn", run);
        if (run && !estimator->isRunning())
            onSpawnKillEstimator();
        ROS_INFO("[PaCMan Vision]\tAdding Tracker Module");
        tracker = std::make_shared<pacv::Tracker>(basic_node->getNodeHandle(), "tracker", storage);
        tracker->setRate(40.0); //40Hz is enough
        tracker->setBasicNodeConfig(basic_node->getConfig());
        tracker_gui = std::make_shared<TrackerGui>(tracker->getConfig());
        basic_gui->addTab(tracker_gui->getWidget(), "Tracker Module");
        tracker->getConfig()->get("spawn", run);
        if (run && !tracker->isRunning())
            onSpawnKillTracker();
#endif
#ifdef PACV_LISTENER_SUPPORT
        ROS_INFO("[PaCMan Vision]\tAdding Listener Module");
        listener = std::make_shared<pacv::Listener>(basic_node->getNodeHandle(), "listener", storage);
        listener->setRate(30.0); //30Hz is enough
        listener_gui = std::make_shared<ListenerGui>(listener->getConfig());
        basic_node->setListenerConfig(listener->getConfig());
        basic_gui->addTab(listener_gui->getWidget(), "Listener Module");
        bool val;
        listener->getConfig()->get("spawn", val);
        if (val && !listener->isRunning())
            onSpawnKillListener();
#endif
#ifdef PACV_MODELER_SUPPORT
        ROS_INFO("[PaCMan Vision]\tAdding Modeler Module");
        modeler = std::make_shared<pacv::Modeler>(basic_node->getNodeHandle(), "in_hand_modeler", storage);
        modeler->setRate(40.0); //30Hz is enough
        // modeler_gui = std::make_shared<ListenerGui>(listener->getConfig());
        // basic_gui->addTab(listener_gui->getWidget(), "Listener Module");
        bool spawn;
        modeler->getConfig()->get("spawn", spawn);
        if (spawn && !modeler->isRunning())
            onSpawnKillModeler();

#endif
        initConnections();
        ROS_INFO("[PaCMan Vision]\tShowing Gui...");
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
        ROS_INFO("[PaCMan Vision]\tShutting Down...");
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
        //reactivate the gui button
        basic_gui->getSaveButt()->setDisabled(false);
    }
}

void
PacmanVision::onColorChange(double r, double g, double b)
{
    basic_node->setFilterColor(r,g,b);
}

void
PacmanVision::onObjsRefresh()
{
#ifdef PACV_RECOGNITION_SUPPORT
    tracker_gui->getObjList()->clear();
    std::list<std::string> objs;
    tracker->getObjList(objs);
    for (const auto& x: objs)
        tracker_gui->getObjList()->addItem(x.c_str());
    tracker_gui->getTrackButt()->setDisabled(true);
#endif
}

void
PacmanVision::onPoseEstimation()
{
#ifdef PACV_RECOGNITION_SUPPORT
    tracker_gui->getObjList()->clear();
    if (tracker->isRunning())
        tracker_gui->getTrackButt()->setDisabled(true);
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
        if (tracker->isRunning()){
            std::list<std::string> objs;
            tracker->getObjList(objs);
            for (const auto& x: objs)
                tracker_gui->getObjList()->addItem(x.c_str());
        }
    }
#endif
}

void
PacmanVision::onTrackObject(std::string *obj)
{
#ifdef PACV_RECOGNITION_SUPPORT
    std::string srv_name(tracker->getNamespace());
    srv_name += "/track_object";
    srv_track_obj.request.name = *obj;
    service_caller->spawn(srv_track_obj, srv_name);
    connect (service_timer, SIGNAL( timeout() ), this, SLOT( postTrackObject() ));
    service_timer->start(300);
#endif
}
void
PacmanVision::postTrackObject()
{
#ifdef PACV_RECOGNITION_SUPPORT
    if (service_caller->postCallClean()){
        //service was called and it is finished
        service_timer->disconnect(SIGNAL( timeout() ));
        service_timer->stop();
        tracker_gui->getTrackButt()->setDisabled(false);
    }
#endif
}

void
PacmanVision::onStopTrack()
{
#ifdef PACV_RECOGNITION_SUPPORT
    std::string srv_name(tracker->getNamespace());
    srv_name += "/stop_track";
    service_caller->spawn(srv_stop_track, srv_name);
    connect (service_timer, SIGNAL( timeout() ), this, SLOT( postStopTrack() ));
    service_timer->start(300);
#endif
}
void
PacmanVision::postStopTrack()
{
#ifdef PACV_RECOGNITION_SUPPORT
    if (service_caller->postCallClean()){
        //service was called and it is finished
        service_timer->disconnect(SIGNAL( timeout() ));
        service_timer->stop();
    }
#endif
}

void
PacmanVision::onSaveInHand(bool right, std::string *obj, std::string *hand)
{
#ifdef PACV_LISTENER_SUPPORT
    std::string srv_name(listener->getNamespace());
    srv_name += "/get_cloud_in_hand";
    srv_get_in_hand.request.right = right;
    srv_get_in_hand.request.save_obj = *obj;
    srv_get_in_hand.request.save_hand = *hand;
    service_caller->spawn(srv_get_in_hand, srv_name);
    connect (service_timer, SIGNAL( timeout() ), this, SLOT( postSaveInHand() ));
    service_timer->start(200);
#endif
}
void
PacmanVision::postSaveInHand()
{
#ifdef PACV_LISTENER_SUPPORT
    if (service_caller->postCallClean()){
        //service was called and it is finished
        service_timer->disconnect(SIGNAL( timeout() ));
        service_timer->stop();
        //srv_estimate has the response, but we dont care about it
        //reactivate the gui button
        listener_gui->getInHandButt()->setDisabled(false);
    }
#endif
}
