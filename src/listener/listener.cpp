#include <listener/listener.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <common/common_ros.h>
#include <common/common_pcl.h>
#include <sensor_msgs/PointCloud2.h>

namespace pacv
{

Listener::Listener(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor)
    :Module<Listener>(n,ns,stor)
{
    config=std::make_shared<ListenerConfig>();
}

void
Listener::updateRosparams()
{
    for (const auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (!config->get(key, val))
            ROS_WARN("[Listener::%s]\tFailed to get key:%s from Config",__func__,key.c_str());
        else
            nh->setParam(key, val);
    }
}

void
Listener::init()
{
    if(!nh){
        ROS_ERROR("[Listener::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    srv_get_in_hand = nh->advertiseService("get_cloud_in_hand", &Listener::cb_get_in_hand, this);
    std::string mark_topic(getFatherNamespace()+"/markers");
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>(mark_topic, 1);
    right_arm = std::make_shared<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    left_arm = std::make_shared<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    right_hand = std::make_shared<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    left_hand = std::make_shared<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    right_arm->resize(arm_names.size());
    left_arm->resize(arm_names.size());
    left_hand->resize(hand_names.size());
    right_hand->resize(hand_names.size());
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[Listener::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[Listener::%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
}

void
Listener::deInit()
{
    marks.reset();
    right_arm.reset();
    left_arm.reset();
    right_hand.reset();
    left_hand.reset();
}

ListenerConfig::Ptr
Listener::getConfig() const
{
    return config;
}

void
Listener::listen(std::string which, std::string component)
{
    if (which.compare("left") !=0 && which.compare("right")!=0){
        ROS_ERROR_THROTTLE(30,"[Listener::%s]\tPlease only specify left or right",__func__);
        return;
    }
    if (component.compare("arm") !=0 && component.compare("hand")!=0){
        ROS_ERROR_THROTTLE(30,"[Listener::%s]\tPlease only specify arm or hand as component",__func__);
        return;
    }
    tf::StampedTransform trans;
    std::string ref_frame;
    storage->readSensorFrame(ref_frame);
    if (component.compare("arm") == 0){
        try
        {
            for (size_t i=0; i<arm_names.size(); ++i)
            {
                tf_listener.waitForTransform(ref_frame, (which+arm_names[i]), ros::Time(0), ros::Duration(2.0));
                tf_listener.lookupTransform(ref_frame, (which+arm_names[i]), ros::Time(0), trans);
                if (which.compare("right")==0)
                    fromTF(trans, right_arm->at(i));
                else
                    fromTF(trans, left_arm->at(i));
            }
            //update storage
            if (which.compare("right")==0)
                storage->writeRightArm(right_arm);
            else
                storage->writeLeftArm(left_arm);
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR_THROTTLE(30,"[Listener::%s]\tCannot find Vito Arm Transforms. Make sure Vito is running",__func__);
        }
    }
    else{
        try
        {
            for (size_t i=0; i<hand_names.size(); ++i)
            {
                tf_listener.waitForTransform(ref_frame, (which+hand_names[i]), ros::Time(0), ros::Duration(2.0));
                tf_listener.lookupTransform(ref_frame, (which+hand_names[i]), ros::Time(0), trans);
                if (which.compare("right")==0)
                    fromTF(trans, right_hand->at(i));
                else
                    fromTF(trans, left_hand->at(i));
            }
            if (which.compare("right")==0)
                storage->writeRightHand(right_hand);
            else
                storage->writeLeftHand(left_hand);
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            ROS_ERROR_THROTTLE(30,"[Listener::%s]\tCannot find Vito Hand Transforms. Make sure Vito is running",__func__);
        }
    }
}

bool
Listener::cb_get_in_hand(pacman_vision_comm::get_cloud_in_hand::Request& req, pacman_vision_comm::get_cloud_in_hand::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Listener::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (req.right && !right_hand){
        ROS_ERROR("[Listener::%s]\tRequested right hand transforms are not present at the moment.",__func__);
        return false;
    }
    if (!req.right && !left_hand){
        ROS_ERROR("[Listener::%s]\tRequested left hand transforms are not present at the moment.",__func__);
        return false;
    }
    PTC::Ptr obj = boost::make_shared<PTC>(); //gets progressively overwritten
    PTC::Ptr cloud_original = boost::make_shared<PTC>();
    PTC::Ptr hand = boost::make_shared<PTC>();
    PTC::Ptr piece = boost::make_shared<PTC>();
    PTC::Ptr dest = boost::make_shared<PTC>();
    this->storage->readSceneProcessed(obj);
    pcl::copyPointCloud(*obj, *cloud_original);
    //listen right or left hand based on req.right
    if (req.right)
    {
        for (size_t i=0; i<right_hand->size(); ++i)
        {
            crop_a_box(obj, dest, soft_hand_right[i], true, right_hand->at(i), false);
            pcl::copyPointCloud(*dest, *obj);
            crop_a_box(cloud_original, piece, soft_hand_right[i], false, right_hand->at(i), false);
            *hand += *piece;
        }
    }
    else{
        for (size_t i=0; i<left_hand->size(); ++i)
        {
            crop_a_box(obj, dest, soft_hand_left[i], true, left_hand->at(i), false);
            pcl::copyPointCloud(*dest, *obj);
            crop_a_box(cloud_original, piece, soft_hand_left[i], false, right_hand->at(i), false);
            *hand += *piece;
        }
    }
    if (!req.save_obj.empty()){
            pcl::PointCloud<pcl::PointXYZRGBA> obj_rgba;
            pcl::copyPointCloud(*obj, obj_rgba);
            obj_rgba.is_dense = true;
            pcl::PCDWriter writer;
            if ( writer.writeASCII(req.save_obj.c_str(), obj_rgba,16 ) ==0 )
                ROS_INFO("[Listener::%s]\tIn hand object cloud saved to %s", __func__, req.save_obj.c_str());
            else{
                ROS_ERROR("[Listener::%s]\tFailed to save in hand object cloud to %s", __func__, req.save_obj.c_str());
                return false;
            }
    }
    if (!req.save_hand.empty()){
            pcl::PointCloud<pcl::PointXYZRGBA> hand_rgba;
            pcl::copyPointCloud(*hand, hand_rgba);
            hand_rgba.is_dense = true;
            pcl::PCDWriter writer;
            pcl::copyPointCloud(*hand, hand_rgba);
            if ( writer.writeASCII(req.save_hand.c_str(), hand_rgba,16 ) == 0 )
                ROS_INFO("[Listener::%s]\tHand cloud saved to %s", __func__, req.save_hand.c_str());
            else{
                ROS_ERROR ("[Listener::%s]\tFailed to save hand cloud to %s",__func__,req.save_hand.c_str());
                return false;
            }
    }
    sensor_msgs::PointCloud2 msg, msg2;
    pcl::toROSMsg(*obj, msg);
    pcl::toROSMsg(*hand, msg2);
    res.obj = msg;
    res.hand = msg2;
    return true;
}

void
Listener::spinOnce()
{
    bool val;
    config->get("listen_right_arm", val);
    if (val)
        listen("right", "arm");
    config->get("listen_left_arm", val);
    if (val)
        listen("left", "arm");
    config->get("listen_right_hand", val);
    if (val)
        listen("right", "hand");
    config->get("listen_left_hand", val);
    if (val)
        listen("left", "hand");
    create_markers();
    publish_markers();
}

void
Listener::create_markers()
{
    bool val;
    config->get("publish_markers", val);
    if(!val)
        return;
    marks = std::make_shared<visualization_msgs::MarkerArray>();
    std::string frame;
    storage->readSensorFrame(frame);
    double scale;
    config->get("geometry_scale", scale);
    config->get("listen_right_arm", val);
    if(val && right_arm){
        for (size_t i=0; i<right_arm->size(); ++i)
        {
            visualization_msgs::Marker mark;
            geometry_msgs::Pose pose;
            create_box_marker(lwr_arm[i]*scale, mark, false);
            mark.header.frame_id = frame;
            mark.header.stamp = ros::Time();
            fromEigen(right_arm->at(i), pose);
            mark.pose = pose;
            mark.ns = "Right Arm";
            mark.id = i;
            mark.color.b=0.0f;
            mark.color.r=0.4f;
            marks->markers.push_back(mark);
        }
    }
    config->get("listen_left_arm", val);
    if(val && left_arm){
        for (size_t i=0; i<left_arm->size(); ++i)
        {
            visualization_msgs::Marker mark;
            geometry_msgs::Pose pose;
            create_box_marker(lwr_arm[i]*scale, mark, false);
            mark.header.frame_id = frame;
            mark.header.stamp = ros::Time();
            fromEigen(left_arm->at(i), pose);
            mark.pose = pose;
            mark.ns = "Left Arm";
            mark.id = i;
            mark.color.b=0.0f;
            mark.color.r=0.4f;
            marks->markers.push_back(mark);
        }
    }
    config->get("listen_right_hand", val);
    if(val && right_hand){
        for (size_t i=0; i<right_hand->size(); ++i)
        {
            visualization_msgs::Marker mark;
            geometry_msgs::Pose pose;
            create_box_marker(soft_hand_right[i]*scale, mark, false);
            mark.header.frame_id = frame;
            mark.header.stamp = ros::Time();
            fromEigen(right_hand->at(i), pose);
            mark.pose = pose;
            mark.ns = "Right Hand";
            mark.id = i;
            mark.color.b=0.0f;
            mark.color.r=0.4f;
            marks->markers.push_back(mark);
        }
    }
    config->get("listen_left_hand", val);
    if(val && left_hand){
        for (size_t i=0; i<left_hand->size(); ++i)
        {
            visualization_msgs::Marker mark;
            geometry_msgs::Pose pose;
            create_box_marker(soft_hand_left[i]*scale, mark, false);
            mark.header.frame_id = frame;
            mark.header.stamp = ros::Time();
            fromEigen(left_hand->at(i), pose);
            mark.pose = pose;
            mark.ns = "Left Hand";
            mark.id = i;
            mark.color.b=0.0f;
            mark.color.r=0.4f;
            marks->markers.push_back(mark);
        }
    }
}

void
Listener::publish_markers()
{
    bool val;
    config->get("publish_markers", val);
    if (!val)
        return;
    if (marks && pub_markers.getNumSubscribers()>0){
        pub_markers.publish(*marks);
    }
}

}
