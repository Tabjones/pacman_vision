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
#include <recognition/estimator.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <pel/pe_progressive_bisection.h>

namespace pacv
{
//Constructor
Estimator::Estimator(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor)
    :Module<Estimator>(n,ns,stor)
{
    config=std::make_shared<EstimatorConfig>();
    db_path = (ros::package::getPath("pacman_vision") + "/database" );
    pe = std::make_shared<pel::interface::PEProgressiveBisection>();
    if (!boost::filesystem::exists(db_path) || !boost::filesystem::is_directory(db_path))
        ROS_WARN("[Estimator::%s] Database for pose estimation does not exists!! Plese put one in /database folder, before trying to perform a pose estimation.",__func__);
    //tmp set params to dump into default
    // nh.setParam("object_calibration", false);
    // nh.setParam("iterations", 5);
    // nh.setParam("neighbors", 20);
    // nh.setParam("cluster_tol", 0.05);
    ////////////////////////////////////////////////////////
    bool run;
    ros::param::get("/pacman_vision/estimator/running", run);
    config->set("running", run);
}

void
Estimator::updateRosparams()
{
    for (const auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (!config->get(key, val))
            ROS_WARN("[Estimator::%s]\tFailed to get key:%s from Config",__func__,key.c_str());
        else
            nh->setParam(key, val);
    }
}

void
Estimator::init()
{
    if(!nh){
        ROS_ERROR("[Estimator::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    srv_estimate = nh->advertiseService("estimate", &Estimator::cb_estimate, this);
    std::string mark_topic(getFatherNamespace()+"/markers");
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>(mark_topic, 1);
    scene=boost::make_shared<PXC>();
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[Estimator::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[Estimator::%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
    pe->setParam("verbosity",1);
    pe->setRMSEThreshold(0.003);
    config->get("iterations", iter);
    pe->setStepIterations(iter);
    config->get("neighbors", neigh);
    pe->setParam("lists_size",neigh);
    pe->setParam("downsamp",0);
    pe->loadAndSetDatabase(db_path);
    config->get("always_success", all_success);
    pe->setConsiderSuccessOnListSizeOne(all_success);
    config->get("rmse_thresh", rmse_thresh);
    pe->setRMSEThreshold(rmse_thresh);
    tracked_idx = transition = -1;
}

void Estimator::deInit()
{
    //this frees memory when module is killed
    marks.reset();
    estimations.reset();
    clusters.reset();
    names.reset();
    scene.reset();
}

EstimatorConfig::Ptr
Estimator::getConfig() const
{
    return config;
}
int
Estimator::extract_clusters()
{
    storage->readSceneProcessed(scene);
    if (scene->empty()){
        ROS_WARN("[Estimator::%s]\tProcessed scene is empty, cannot continue...",__func__);
        return -1;
    }
    double tol;
    config->get("cluster_tol", tol);
    ROS_INFO("[Estimator::%s]\tExtracting object clusters with cluster tolerance of %g",__func__, tol);
    //objects
    pcl::ExtractIndices<PX> extract;
    pcl::EuclideanClusterExtraction<PX> ec;
    pcl::search::KdTree<PX>::Ptr tree (new pcl::search::KdTree<PX>);
    std::vector<pcl::PointIndices> cluster_indices;
    //cluster extraction
    tree->setInputCloud(scene);
    ec.setInputCloud(scene);
    ec.setSearchMethod(tree);
    ec.setClusterTolerance(tol);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(scene->points.size());
    ec.extract(cluster_indices);
    int size = (int)cluster_indices.size();
    clusters = std::make_shared<std::vector<PXC> >();
    clusters->resize(size);
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin(); it != cluster_indices.end(); ++it, ++j)
    {
        PXC::Ptr object (new PXC);
        extract.setInputCloud(scene);
        extract.setIndices(boost::make_shared<pcl::PointIndices>(*it));
        extract.setNegative(false);
        extract.filter(clusters->at(j));
    }
    ROS_INFO("[Estimator::%s]\tFound %d clusters of possible objects.",__func__,size);
    return size;
}

bool
Estimator::cb_estimate(pacman_vision_comm::estimate::Request& req, pacman_vision_comm::estimate::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Estimator::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (tracked_idx != -1){
        //the tracker is tracking, no new pose estimations :(
        ROS_ERROR("[Estimator::%s]\tThis service is suspended while the Tracker is tracking an object.",__func__);
        return false;
    }
    if (estimate()){
        geometry_msgs::Pose pose;
        for (size_t i=0; i<estimations->size(); ++i)
        {
            //assemble service response
            fromEigen(estimations->at(i), pose);
            pacman_vision_comm::pe pose_est;
            pose_est.pose = pose;
            pose_est.name = names->at(i).first;
            pose_est.id = names->at(i).second;
            pose_est.parent_frame = scene->header.frame_id;
            res.estimated.poses.push_back(pose_est);
        }
        ROS_INFO("[Estimator::%s]\tPose Estimation complete!", __func__);
        return true;
    }
    else{
        ROS_WARN("[Estimator::%s]\tPose Estimation failed!", __func__);
        return false;
    }
}

bool
Estimator::estimate()
{
    int size = extract_clusters();
    if (size < 1){
        ROS_ERROR("[Estimator::%s]\tNo object clusters found in scene, aborting pose estimation...",__func__);
        return false;
    }
    bool calib, succ;
    int it,k;
    double thresh;
    config->get("iterations", it);
    if (it!=iter){
        iter = it;
        pe->setStepIterations(iter);
    }
    config->get("neighbors", k);
    if (k!= neigh){
        neigh = k;
        pe->setParam("lists_size", neigh);
    }
    config->get("always_success", succ);
    if (succ != all_success){
        all_success = succ;
        pe->setConsiderSuccessOnListSizeOne(all_success);
    }
    config->get("rmse_thresh", thresh);
    if (thresh != rmse_thresh){
        rmse_thresh = thresh;
        pe->setRMSEThreshold(rmse_thresh);
    }
    config->get("object_calibration", calib);
    names= std::make_shared<std::vector<std::pair<std::string, std::string>>>();
    estimations = std::make_shared<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    for (int i=0; i<size; ++i)
    {
        pe->setTarget(clusters->at(i).makeShared(), "object");
        pel::Candidate pest;
        pe->estimate(pest);
        std::pair<std::string, std::string> name;
        if (pest.getName().empty()){
            //failed move on the next cluster
            continue;
        }
        std::vector<std::string> vst;
        std::string n = pest.getName();
        boost::split(vst, n, boost::is_any_of("_"), boost::token_compress_on);
        if (calib)
            name.first = "object";
        else
            name.first = vst.at(0);
        estimations->push_back(pest.getTransformation());
        name.second = vst.at(0);
        names->push_back(name);
        ROS_INFO("[Estimator::%s]\tFound %s.",__func__,name.first.c_str());
    }
    if (names->empty()){
        //pose estimation failed no cluster could be recognized
        names.reset();
        estimations.reset();
        marks.reset();
        return false;
    }
    //first check if we have more copy of the same object in names
    for (int i=0; i<names->size(); ++i)
    {
        int count(1);
        std::string name_original = names->at(i).first;
        if (i>0){
            for (int j=0; j<i; ++j)
            {
                if (names->at(i).first.compare(names->at(j).first) == 0){
                    //i-th name is equal to j-th name
                    names->at(i).first = name_original + "_" + std::to_string(++count);
                }
            }
        }
    }
    //Save estimations in Storage
    // this->storage->write_obj_clusters(this->clusters);
    storage->writeObjNames(names);
    storage->writeObjTransforms(estimations);
    //elaborate new markers
    create_markers();
    return true;
}

void
Estimator::create_markers()
{
    if(!estimations){
        //We dont have estimations, check if storage has some. This could happen
        //after estimater  gets killed and  tracker is started without  new pose
        //estimation called.
        if (storage->readObjTransforms(estimations))
            storage->readObjNames(names);
        else
            return;
    }
    std::string ref_frame;
    storage->readSensorFrame(ref_frame);
    // if(marks){
    //     //remove old markers
    //     for (auto &x: marks->markers){
    //         x.action = 2;
    //         x.header.stamp = ros::Time();
    //         pub_markers.publish(x);
    //     }
    // }
    marks = std::make_shared<visualization_msgs::MarkerArray>();
    for (size_t i=0; i<estimations->size(); ++i) //if size is zero dont do anything
    {
        if (i == tracked_idx)
            //skip tracked object marker, Tracker will pusblish it
            continue;
        geometry_msgs::Pose pose;
        fromEigen(estimations->at(i), pose);
        visualization_msgs::Marker marker;
        create_object_marker(pose, names->at(i), marker);
        marker.header.frame_id = ref_frame.c_str();
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marks->markers.push_back(marker);
    }
}

void
Estimator::publish_markers()
{
    bool val;
    config->get("publish_markers", val);
    if (!val)
        return;
    if (marks && pub_markers.getNumSubscribers()>0){
        pub_markers.publish(*marks);
    }
}

void
Estimator::publish_tf()
{
    bool val;
    config->get("broadcast_tf", val);
    if (!val || !estimations || !names)
        return;
    std::string frame;
    storage->readSensorFrame(frame);
    for (size_t i=0; i<estimations->size(); ++i)
    {
        if (i == tracked_idx)
            //skip tracked tf, Tracker will do it
            continue;
        tf::Transform tran;
        fromEigen(estimations->at(i), tran);
        tf_brc.sendTransform(tf::StampedTransform(tran, ros::Time::now(), frame.c_str(), names->at(i).first.c_str()));
    }
}

///Module behaviour
void
Estimator::spinOnce()
{
    //monitor when and if the Tracker starts tracking
    storage->readTrackedIndex(tracked_idx);
    //Tracker transition from stopped to started
    if (transition == -1 && tracked_idx != -1){
        create_markers();
        transition = tracked_idx;
    }
    //Tracker transition from started to stopped
    else if (transition != -1 && tracked_idx == -1){
        storage->readObjTransforms(estimations);
        storage->readObjNames(names);
        create_markers();
        transition = tracked_idx;
    }
    //publish markers if user requested
    publish_markers();
    //and tf if user requested
    publish_tf();
}
}
