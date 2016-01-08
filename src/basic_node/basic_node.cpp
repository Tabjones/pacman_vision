#include <basic_node/basic_node.h>

namespace pacv
{
BasicNode::BasicNode(const std::string ns, const Storage::Ptr stor):
        Module<BasicNode>(ns,stor)
{
    scene_processed = boost::make_shared<PTC>();
    config = std::make_shared<BasicConfig>();
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
BasicNode::deInit()
{
    //nothing to do actually this is suppress annoying warning
}

void
BasicNode::init()
{
    if(!nh){
        ROS_ERROR("[BasicNode::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    srv_get_scene = nh->advertiseService("get_scene_processed", &BasicNode::cb_get_scene, this);
    pub_scene = nh->advertise<PTC> ("scene_processed", 5);
    pub_markers = nh->advertise<visualization_msgs::Marker>("markers", 1);
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (key.compare("filter_limits")==0)
        {
            Box lim(-0.5,-0.5,0.3,0.5,0.5,2);
            if(!nh->getParam("limit_xmax", lim.x2))
                ROS_WARN("[%s]\tKey:limit_xmax not found on parameter server",__func__);
            if(!nh->getParam("limit_xmin", lim.x1))
                ROS_WARN("[%s]\tKey:limit_xmin not found on parameter server",__func__);
            if(!nh->getParam("limit_ymax", lim.y2))
                ROS_WARN("[%s]\tKey:limit_ymax not found on parameter server",__func__);
            if(!nh->getParam("limit_ymin", lim.y1))
                ROS_WARN("[%s]\tKey:limit_ymin not found on parameter server",__func__);
            if(!nh->getParam("limit_zmax", lim.z2))
                ROS_WARN("[%s]\tKey:limit_zmax not found on parameter server",__func__);
            if(!nh->getParam("limit_zmin", lim.z1))
                ROS_WARN("[%s]\tKey:limit_zmin not found on parameter server",__func__);

            if(!config->set(key, lim))
                ROS_WARN("[%s]\tFailed to set key:%s into Config",__func__,key.c_str());
            continue;
        }
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
    update_markers(); //one time call
}

BasicConfig::Ptr
BasicNode::getConfig() const
{
    return config;
}
// void
// BasicNode::updateIfNeeded(const BasicNode::ConfigPtr conf, bool reset)
// {
//     if (reset){
//         init();
//         return;
//     }
//     if (conf){
//         if (config->cropping != conf->cropping){
//             LOCK guard(mtx_config);
//             config->cropping = conf->cropping;
//         }
//         if (config->downsampling != conf->downsampling){
//             LOCK guard(mtx_config);
//             config->downsampling = conf->downsampling;
//         }
//         if (config->segmenting != conf->segmenting){
//             LOCK guard(mtx_config);
//             config->segmenting = conf->segmenting;
//         }
//         if (config->keep_organized != conf->keep_organized){
//             LOCK guard(mtx_config);
//             config->keep_organized = conf->keep_organized;
//         }
//         if (config->publish_limits != conf->publish_limits){
//             LOCK guard(mtx_config);
//             config->publish_limits = conf->publish_limits;
//         }
//         // config->publish_plane = conf->publish_plane;
//         if (config->limits != conf->limits){
//             LOCK guard(mtx_config);
//             config->limits = conf->limits;
//             update_markers();
//         }
//         if (config->downsampling_leaf_size != conf->downsampling_leaf_size){
//             LOCK guard(mtx_config);
//             config->downsampling_leaf_size = conf->downsampling_leaf_size;
//         }
//         if (config->plane_tolerance != conf->plane_tolerance){
//             LOCK guard(mtx_config);
//             config->plane_tolerance = conf->plane_tolerance;
//         }
//     }
// }
//
//when service to get scene is called
bool
BasicNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res)
{
    //This saves in home... possible todo improvement to let user specify location
    if (this->isDisabled()){
        ROS_WARN("[PaCMaN Vision][%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (scene_processed){
        sensor_msgs::PointCloud2 msg;
        if (req.save.compare("false") != 0){
            std::string home = std::getenv("HOME");
            pcl::io::savePCDFile( (home + "/" + req.save + ".pcd").c_str(), *scene_processed);
            ROS_INFO("[PaCMaN Vision][%s]\tProcessed scene saved to %s", __func__, (home + "/" + req.save + ".pcd").c_str());
        }
        pcl::toROSMsg(*scene_processed, msg);
        res.scene = msg;
        ROS_INFO("[PaCMaN Vision][%s]\tSent processed scene to service response.", __func__);
        return true;
    }
    else{
        ROS_WARN("[PaCMaN Vision][%s]\tNo Processed Scene to send to Service!", __func__);
        return false;
    }
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
void
BasicNode::process_scene()
{
    bool crop, downsamp, segment;
    config->get("cropping", crop);
    config->get("downsampling", downsamp);
    config->get("segmenting", segment);
    PTC::Ptr tmp = boost::make_shared<PTC>();
    PTC::Ptr dest;
    PTC::Ptr source;
    storage->read_scene(source);
    //check if we need to crop scene
    if(!source)
        return;
    if(source->empty())
        return;
    if (crop){
        Box lim;
        bool org;
        config->get("filter_limits", lim);
        config->get("keep_organized", org);
        crop_a_box(source, dest, lim, false, Eigen::Matrix4f::Identity(), org);
        if(dest->empty())
            return;
    }
    //check if we need to downsample scene
    if (downsamp){
        if (dest){
            //means we have performed at least one filter before this
            pcl::copyPointCloud(*dest, *tmp);
            downsamp_scene(tmp, dest);
        }
        else
            downsamp_scene(source, dest);
        if(dest->empty())
            return;
    }
    if (segment){
        if (dest){
            //means we have performed at least one filter before this
            pcl::copyPointCloud(*dest, *tmp);
            segment_scene(tmp, dest);
        }
        else
            segment_scene(source, dest);
        if(dest->empty())
            return;
    }
    //Add vito cropping when listener is done
    // //crop arms if listener is active and we set it
    // if ((crop_r_arm || crop_l_arm || crop_r_hand || crop_l_hand) && this->en_listener && this->listener_module){
    //     if (crop_l_arm){
    //         if (dest)
    //             pcl::copyPointCloud(*dest, *source);
    //         crop_arm(source, dest, false);
    //         if(dest->empty())
    //             return;
    //     }
    //     if (crop_r_arm){
    //         if (dest)
    //             pcl::copyPointCloud(*dest, *source);
    //         crop_arm(source, dest, true);
    //         if(dest->empty())
    //             return;
    //     }
    //     if (crop_r_hand){
    //         if(dest)
    //             pcl::copyPointCloud(*dest, *source);
    //         if (detailed_hand_crop){
    //             for (size_t i=0; i<21; ++i)
    //                 listener_module->listen_and_crop_detailed_hand_piece(true, i, source);
    //             pcl::copyPointCloud(*source, *dest);
    //             if(dest->empty())
    //                 return;
    //         }
    //         else{
    //             crop_hand(source, dest, true);
    //             if(dest->empty())
    //                 return;
    //         }
    //     }
    //     if (crop_l_hand){
    //         if (dest)
    //             pcl::copyPointCloud(*dest, *source);
    //         if (detailed_hand_crop){
    //             for (size_t i=0; i<21; ++i)
    //                 listener_module->listen_and_crop_detailed_hand_piece(false, i, source);
    //             pcl::copyPointCloud(*source, *dest);
    //             if(dest->empty())
    //                 return;
    //         }
    //         else{
    //             crop_hand(source, dest, false);
    //             if(dest->empty())
    //                 return;
    //         }
    //     }
    // }

    //Save into storage
    if (dest){
        if(!dest->empty()){
            pcl::copyPointCloud(*dest, *scene_processed);
            storage->write_scene_processed(this->scene_processed);
        }
    }
    else{
        pcl::copyPointCloud(*source, *scene_processed);
        storage->write_scene_processed(this->scene_processed);
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
    mark_lim=std::make_shared<visualization_msgs::Marker>();
    Box lim;
    config->get("filter_limits", lim);
    create_box_marker(lim, *mark_lim, false);
    //make it red
    mark_lim->color.g = 0.0f;
    mark_lim->color.b = 0.0f;
    //name it
    mark_lim->ns="Cropping Limits";
    mark_lim->id=0;
    if (scene_processed->header.frame_id.empty())
        //no scene processed yet... guess the frame id.
        mark_lim->header.frame_id = "/camera_rgb_optical_frame";
    else
        mark_lim->header.frame_id = scene_processed->header.frame_id;
}
void
BasicNode::publish_markers()
{
    bool crop, plim;
    config->get("cropping", crop);
    config->get("publish_limits", plim);
    if (crop && plim && pub_markers.getNumSubscribers()>0 && mark_lim){
        mark_lim->header.stamp = ros::Time();
        pub_markers.publish(*mark_lim);
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
    process_scene();
    publish_scene_processed();
    publish_markers();
    ros::spinOnce();
}

void
BasicNode::spin()
{
    while (isOk() && is_running)
    {
        if (this->isDisabled())
            ros::spinOnce();
        else
            spinOnce();
        spin_rate->sleep();
    }
}
}
