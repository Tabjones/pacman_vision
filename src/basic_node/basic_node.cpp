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
#include <basic_node/basic_node.h>

#ifdef PACV_LISTENER_SUPPORT
#include <listener/vito_geometry.h>
#endif

namespace pacv
{
BasicNode::BasicNode(const std::string ns, const Storage::Ptr stor):
        Module<BasicNode>(ns,stor), was_color_filtering(false)
{
    scene_processed = boost::make_shared<PTC>();
    config = std::make_shared<BasicConfig>();
    box_transform.setIdentity();
    //tmp set param to dump into default
    // nh.setParam("cropping", false);
    // nh.setParam("downsampling", false);
    // nh.setParam("segmenting", false);
    // nh.setParam("keep_organized", false);
    // nh.setParam("publish_limits", false);
    // nh.setParam("limit_xmax", 0.5);
    // nh.setParam("limit_xmin", -0.5);
    // nh.setParam("limit_ymax", 0.5);
    // nh.setParam("limit_ymin", -0.5);
    // nh.setParam("limit_zmax", 1.5);
    // nh.setParam("limit_zmin", 0.3);
    // nh.setParam("downsampling_leaf_size", 0.01);
    // nh.setParam("plane_tolerance", 0.005);
    /////////////////////////////////////////
}

void
BasicNode::updateRosparams()
{
    for (const auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (key.compare("filter_limits")==0){
            Box lim;
            if (!config->get("filter_limits", lim))
                ROS_WARN("[BasicNode::%s]\tFailed to get key:%s from Config",__func__,key.c_str());
            else{
                nh->setParam("limit_xmax", lim.x2);
                nh->setParam("limit_xmin", lim.x1);
                nh->setParam("limit_ymax", lim.y2);
                nh->setParam("limit_ymin", lim.y1);
                nh->setParam("limit_zmax", lim.z2);
                nh->setParam("limit_zmin", lim.z1);
            }
            continue;
        }
        if (!config->get(key, val))
            ROS_WARN("[BasicNode::%s]\tFailed to get key:%s from Config",__func__,key.c_str());
        else
            nh->setParam(key, val);
    }
}

void
BasicNode::setConfigFromRosparams()
{
    //init node params
    for (const auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (key.compare("filter_limits")==0)
        {
            Box lim(-0.5,-0.5,0.3,0.5,0.5,2);
            if(!nh->getParam("limit_xmax", lim.x2))
                ROS_WARN("[BasicNode::%s]\tKey:limit_xmax not found on parameter server",__func__);
            if(!nh->getParam("limit_xmin", lim.x1))
                ROS_WARN("[BasicNode::%s]\tKey:limit_xmin not found on parameter server",__func__);
            if(!nh->getParam("limit_ymax", lim.y2))
                ROS_WARN("[BasicNode::%s]\tKey:limit_ymax not found on parameter server",__func__);
            if(!nh->getParam("limit_ymin", lim.y1))
                ROS_WARN("[BasicNode::%s]\tKey:limit_ymin not found on parameter server",__func__);
            if(!nh->getParam("limit_zmax", lim.z2))
                ROS_WARN("[BasicNode::%s]\tKey:limit_zmax not found on parameter server",__func__);
            if(!nh->getParam("limit_zmin", lim.z1))
                ROS_WARN("[BasicNode::%s]\tKey:limit_zmin not found on parameter server",__func__);

            if(!config->set(key, lim))
                ROS_WARN("[BasicNode::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
            continue;
        }
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[BasicNode::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[BasicNode::%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
}
void
BasicNode::deInit()
{
    marks.reset();
    //nothing much to do,
    //actually this is to suppress annoying warning from base class
}

void
BasicNode::init()
{
    if(!nh){
        ROS_ERROR("[BasicNode::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    srv_get_scene = nh->advertiseService("get_scene_processed", &BasicNode::cb_get_scene, this);
    pub_scene = nh->advertise<PTC> ("processed_scene", 5);
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>("markers", 1);
    setConfigFromRosparams();
    update_markers(); //one time call
}

BasicConfig::Ptr
BasicNode::getConfig() const
{
    return config;
}

//when service to get scene is called
bool
BasicNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
{
    //This saves in home... possible todo improvement to let user specify location
    if (isDisabled()){
        ROS_WARN("[BasicNode::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (scene_processed){
        sensor_msgs::PointCloud2 msg;
        if (!req.save.empty()){
            pcl::PointCloud<pcl::PointXYZRGBA> cloud;
            pcl::copyPointCloud(*scene_processed, cloud);
            if(pcl::io::savePCDFileBinaryCompressed( req.save.c_str(), cloud) == 0)
                ROS_INFO("[BasicNode::%s]\tScene Processed saved to %s", __func__, req.save.c_str());
            else
                ROS_ERROR("[BasicNode::%s]\tFailed to save scene to %s", __func__, req.save.c_str());
        }
        pcl::toROSMsg(*scene_processed, msg);
        res.scene = msg;
        return true;
    }
    else{
        ROS_WARN("[BasicNode::%s]\tNo Scene Processed to send to Service!", __func__);
        return false;
    }
}

void
BasicNode::remove_outliers(const PTC::ConstPtr source, PTC::Ptr &dest)
{
    if (!dest)
        dest=boost::make_shared<PTC>();
    pcl::StatisticalOutlierRemoval<PT> sor;
    int k;
    double std_mul;
    config->get("outliers_mean_k", k);
    config->get("outliers_std_mul", std_mul);
    sor.setInputCloud(source);
    sor.setMeanK(k);
    sor.setStddevMulThresh(std_mul);
    sor.filter(*dest);
}

void
BasicNode::downsamp_scene(const PTC::ConstPtr source, PTC::Ptr &dest){
    //cannot keep organized cloud after voxelgrid
    if (!dest)
        dest=boost::make_shared<PTC>();
    pcl::VoxelGrid<PT> vg;
    double leaf;
    config->get("downsampling_leaf_size", leaf);
    vg.setLeafSize( leaf, leaf, leaf);
    vg.setDownsampleAllData(true);
    vg.setInputCloud (source);
    vg.filter (*dest);
}
void
BasicNode::segment_scene(const PTC::ConstPtr source, PTC::Ptr &dest)
{
    if (!dest)
        dest=boost::make_shared<PTC>();
    pcl::SACSegmentation<PT> seg;
    pcl::ExtractIndices<PT> extract;
    //coefficients
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    //plane segmentation
    seg.setInputCloud(source);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    double tol;
    config->get("plane_tolerance", tol);
    seg.setDistanceThreshold (tol);
    seg.segment(*inliers, *coefficients);
    //extract what's on top of plane
    extract.setInputCloud(seg.getInputCloud());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*dest);
    //optionally extract  a plane model  for visualization purpose and  create a
    // marker from it
    // Disabled, cause broken!
    /*
     * if (config->publish_plane){
     *     extract.setNegative(false);
     *     PTC::Ptr plane (new PTC);
     *     extract.filter(*plane);
     *     Eigen::Vector4f min,max;
     *     pcl::getMinMax3D(*plane, min, max);
     *     Box limits(min[0],min[1],min[2],max[0],max[1],max[2]);
     *     create_box_marker(mark_plane, limits, true);
     *     //make it red
     *     mark_plane.color.g = 0.0f;
     *     mark_plane.color.b = 0.0f;
     *     //name it
     *     mark_plane.ns="Plane Estimation Model";
     *     mark_plane.id=1;
     *     mark_plane.header.frame_id = dest->header.frame_id;
     * }
     */
}

// void
// BasicNode::extract_principal_color(const PTC::ConstPtr scene)
// {
//     //for now just compute the mean color...
//     //in the future we can create some palette and let the user choose
//     mean_L = mean_a = mean_b = 0.0;
//     for (const auto& pt: scene->points)
//     {
//         double L, a, b;
//         convertPCLColorToCIELAB(pt, L, a, b);
//         mean_L += L;
//         mean_a += a;
//         mean_b += b;
//     }
//     mean_L /= scene->size();
//     mean_a /= scene->size();
//     mean_b /= scene->size();
// }

void
BasicNode::setFilterColor(const double r, const double g, const double b)
{
    convertRGBToCIELAB(r,g,b, ref_L, ref_a, ref_b);
}

void
BasicNode::apply_color_filter(const PTC::ConstPtr source, PTC::Ptr &dest)
{
    if (!dest)
        dest=boost::make_shared<PTC>();
    double thresh;
    config->get("color_dist_thresh", thresh);
    for (std::size_t i=0; i< source->size(); ++i)
    {
        double L,a,b;
        convertPCLColorToCIELAB(source->points[i], L,a,b);
        double dE = deltaE(ref_L, ref_a, ref_b, L,a,b);
        bool invert(false);
        config->get("invert_color_filter", invert);
        if ( dE <= thresh && !invert)
            dest->push_back(source->points[i]);
        else if ( dE > thresh && invert)
            dest->push_back(source->points[i]);
    }
}

void
BasicNode::process_scene()
{
    PTC::Ptr source;
    storage->readScene(source);
    //check if we need to crop scene
    if(!source)
        return;
    if(source->empty())
        return;
    bool crop, downsamp, segment, outliers, color;
    config->get("cropping", crop);
    config->get("downsampling", downsamp);
    config->get("segmenting", segment);
    config->get("outliers_filter", outliers);
    config->get("color_filter", color);
    // if (color && !was_color_filtering){
    //     //we compute a new color model
    //     PTC::Ptr last_processed_scene;
    //     storage->readSceneProcessed(last_processed_scene);
    //     extract_principal_color(last_processed_scene);
    // }
    // was_color_filtering = color;
    PTC::Ptr tmp = boost::make_shared<PTC>();
    PTC::Ptr dest;
    if (crop){
        Box lim;
        bool org;
        config->get("filter_limits", lim);
        config->get("keep_organized", org);
        std::string  ref_frame, box_frame;
        config->get("cropping_ref_frame", box_frame);
        storage->readSensorFrame(ref_frame);
        if (ref_frame.compare(box_frame)!=0)
            crop_a_box(source, dest, lim, false, box_transform.inverse(), org);
        else
            crop_a_box(source, dest, lim, false, Eigen::Matrix4f::Identity(), org);
        if(dest->empty())
            return;
    }
    //check if we need to downsample scene
    if (downsamp){
        if (dest){
            //means we have performed at least one filter before this
            downsamp_scene(dest, tmp);
            dest = tmp;
            tmp = boost::make_shared<PTC>();
        }
        else
            downsamp_scene(source, dest);
        if(dest->empty())
            return;
    }
    //check if we need the plane segmentation
    if (segment){
        if (dest){
            //means we have performed at least one filter before this
            segment_scene(dest, tmp);
            dest = tmp;
            tmp = boost::make_shared<PTC>();
        }
        else
            segment_scene(source, dest);
        if(dest->empty())
            return;
    }
    //check if we need to apply a color filter
    if (color){
        if (dest){
            //means we have performed at least one filter before this
            apply_color_filter(dest, tmp);
            dest = tmp;
            tmp = boost::make_shared<PTC>();
        }
        else
            apply_color_filter(source, dest);
        if(dest->empty())
            return;
    }
    //check if we need to remove outliers
    if (outliers){
        if (dest){
            //means we have performed at least one filter before this
            remove_outliers(dest, tmp);
            dest = tmp;
            tmp = boost::make_shared<PTC>();
        }
        else
            remove_outliers(source, dest);
        if(dest->empty())
            return;
    }
    //Add vito cropping when listener is active
    //crop arms if listener is active and user requested it
#ifdef PACV_LISTENER_SUPPORT
    std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> trans;
    bool val;
    double scale;
    if (!list_config){
        ROS_ERROR("[BasicNode::%s]\tNo Listener Config set, set it with setListenerConfig()",__func__);
        return;
    }
    list_config->get("spawn", val);
    if (val){
        list_config->get("geometry_scale", scale);
        list_config->get("remove_right_arm", val);
        if (val){
            if(storage->readRightArm(trans))
                if (trans->size() == lwr_arm.size() )
                    for(size_t i=0; i<trans->size(); ++i)
                    {
                        if (dest){
                            crop_a_box(dest, tmp, lwr_arm[i]*scale, true, trans->at(i).inverse());
                            dest = tmp;
                            tmp = boost::make_shared<PTC>();
                        }
                        else
                            crop_a_box(source, dest, lwr_arm[i]*scale, true, trans->at(i).inverse());
                    }
        }
        list_config->get("remove_left_arm", val);
        if (val){
            if(storage->readLeftArm(trans))
                if (trans->size() == lwr_arm.size() )
                    for(size_t i=0; i<trans->size(); ++i)
                    {
                        if (dest){
                            crop_a_box(dest, tmp, lwr_arm[i]*scale, true, trans->at(i).inverse());
                            dest = tmp;
                            tmp = boost::make_shared<PTC>();
                        }
                        else
                            crop_a_box(source, dest, lwr_arm[i]*scale, true, trans->at(i).inverse());
                    }
        }
        list_config->get("remove_right_hand", val);
        if (val){
            if(storage->readRightHand(trans))
                if (trans->size() == soft_hand_right.size() )
                    for(size_t i=0; i<trans->size(); ++i)
                    {
                        if (dest){
                            crop_a_box(dest, tmp, soft_hand_right[i]*scale, true, trans->at(i).inverse());
                            dest = tmp;
                            tmp = boost::make_shared<PTC>();
                        }
                        else
                            crop_a_box(source, dest, soft_hand_right[i]*scale, true, trans->at(i).inverse());
                    }
        }
        list_config->get("remove_left_hand", val);
        if (val){
            if(storage->readLeftHand(trans))
                if (trans->size() == soft_hand_left.size() )
                    for(size_t i=0; i<trans->size(); ++i)
                    {
                        if (dest){
                            crop_a_box(dest, tmp, soft_hand_left[i]*scale, true, trans->at(i).inverse());
                            dest = tmp;
                            tmp = boost::make_shared<PTC>();
                        }
                        else
                            crop_a_box(source, dest, soft_hand_left[i]*scale, true, trans->at(i).inverse());
                    }
        }
    }
#endif
    //Save into storage
    if (dest){
        if(!dest->empty()){
            pcl::copyPointCloud(*dest, *scene_processed);
            std::string frame;
            storage->readSensorFrame(frame);
            scene_processed->header.frame_id = frame;
            storage->writeSceneProcessed(scene_processed);
        }
    }
    else{
        pcl::copyPointCloud(*source, *scene_processed);
        std::string frame;
        storage->readSensorFrame(frame);
        scene_processed->header.frame_id = frame;
        storage->writeSceneProcessed(scene_processed);
    }
}
void
BasicNode::publish_scene_processed() const
{
    //republish processed cloud
    if (scene_processed)
        if (!scene_processed->empty() && pub_scene.getNumSubscribers()>0)
            pub_scene.publish(*scene_processed);
}

void
BasicNode::update_markers()
{
    //This is triggered when a config for limits changes
    //only update crop limits, plane always gets recomputed if active
    marks=std::make_shared<visualization_msgs::MarkerArray>();
    Box lim;
    std::string  frame, box_frame;
    storage->readSensorFrame(frame);
    config->get("cropping_ref_frame", box_frame);
    visualization_msgs::Marker mark_lim;
    config->get("filter_limits", lim);
    create_box_marker(lim, mark_lim, false);
    geometry_msgs::Pose pose;
    mark_lim.header.frame_id = frame;
    fromEigen(box_transform, pose);
    mark_lim.pose = pose;
    //make it red
    mark_lim.color.g = 0.0f;
    mark_lim.color.b = 0.0f;
    //name it
    mark_lim.ns="Cropping Limits";
    mark_lim.id=0;
    marks->markers.push_back(mark_lim);
}
void
BasicNode::publish_markers()
{
    bool crop, plim;
    config->get("cropping", crop);
    config->get("publish_limits", plim);
    if (crop && plim && pub_markers.getNumSubscribers()>0 && marks){
        std::string ref_frame;
        storage->readSensorFrame(ref_frame);
        if (ref_frame.empty())
            //fallback to asus
            ref_frame = "/camera_rgb_optical_frame";
        for (auto &m: marks->markers)
            m.header.frame_id = ref_frame;
        pub_markers.publish(*marks);
    }
    /*
     * if (config->publish_plane && pub_markers.getNumSubscribers()>0){
     *     mark_plane.header.stamp = ros::Time();
     *     pub_markers.publish(mark_plane);
     * }
     */
}

void
BasicNode::spinOnce()
{
    std::string  ref_frame, box_frame;
    config->get("cropping_ref_frame", box_frame);
    storage->readSensorFrame(ref_frame);
    if (ref_frame.compare(box_frame) !=0){
        try
        {
            tf::StampedTransform trans;
            tf_listener.waitForTransform(ref_frame, box_frame, ros::Time(0), ros::Duration(2.0));
            tf_listener.lookupTransform(ref_frame, box_frame, ros::Time(0), trans);
            fromTF(trans, box_transform);
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR_THROTTLE(30,"[BasicNode::%s]\tCannot find Cropbox transform %s.",__func__, box_frame.c_str());
        }
    }
    process_scene();
    publish_scene_processed();
    publish_markers();
}

void
BasicNode::spin()
{
    while (isOk() && is_running)
    {
        if (!isDisabled())
            spinOnce();
        ros::spinOnce();
        spin_rate->sleep();
    }
}
}
