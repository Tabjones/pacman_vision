#include <common/storage.h>
#include <common/common_ros.h>

namespace pacv
{
//Constructor
Storage::Storage(): index(-1)
{
    left_hand = Eigen::Matrix4f::Zero();
    right_hand = Eigen::Matrix4f::Zero();
}

bool
Storage::readScene(PTC::Ptr &cloud)
{
    cloud=boost::make_shared<PTC>();
    if (scene){
        if(!scene->empty()){
            LOCK guard(mtx_scene);
            pcl::copyPointCloud(*scene, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tScene is empty! Not reading anything",__func__);
    cloud.reset();
    return false;
}

bool
Storage::readScene(PXC::Ptr &cloud)
{
    cloud=boost::make_shared<PXC>();
    if (scene){
        if (!scene->empty()){
            LOCK guard(mtx_scene);
            pcl::copyPointCloud(*scene, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tScene is empty! Not reading anything",__func__);
    cloud.reset();
    return false;
}

bool
Storage::readSceneProcessed(PTC::Ptr &cloud)
{
    cloud=boost::make_shared<PTC>();
    if (scene_processed){
        if(!scene_processed->empty()){
            LOCK guard(mtx_scene_processed);
            pcl::copyPointCloud(*scene_processed, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tProcessed scene is empty! Not reading anything",__func__);
    cloud.reset();
    return false;
}

bool
Storage::readSceneProcessed(PXC::Ptr &cloud)
{
    cloud=boost::make_shared<PXC>();
    if(scene_processed){
        if(!scene_processed->empty()){
            LOCK guard(mtx_scene_processed);
            pcl::copyPointCloud(*scene_processed, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tProcessed scene is empty! Not reading anything",__func__);
    cloud.reset();
    return false;
}

bool
Storage::writeScene(PTC::Ptr cloud)
{
    if (cloud){
        if (!cloud->empty()){
            LOCK guard(mtx_scene);
            if (!scene)
                scene=boost::make_shared<PTC>();
            pcl::copyPointCloud(*cloud, *scene);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write an empty cloud. Skipping...",__func__);
    return false;
}

bool
Storage::writeSceneProcessed(PTC::Ptr cloud)
{
    if (cloud){
        if (!cloud->empty()){
            LOCK guard(mtx_scene_processed);
            if (!scene_processed)
                scene_processed=boost::make_shared<PTC>();
            pcl::copyPointCloud(*cloud, *scene_processed);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write an empty cloud. Skipping...",__func__);
    return false;
}

// bool
// Storage::read_obj_clusters (std::shared_ptr<std::vector<PXC>> &objs)
// {
//     LOCK guard(mtx_clusters);
//     if (!objs)
//         objs.reset(new std::vector<PXC>);
//     else
//         objs->clear();
//     if (this->clusters.empty()){
//         ROS_WARN_THROTTLE(30,"[Storage][%s] Clusters from Storage are empty! Not reading anything",__func__);
//         return false;
//     }
//     std::copy(clusters.begin(), clusters.end(), std::back_inserter(*objs));
//     return true;
// }
//
// bool
// Storage::write_obj_clusters (std::shared_ptr<std::vector<PXC>> &objs)
// {
//     if (objs){
//         if (!objs->empty()){
//             LOCK guard(mtx_clusters);
//             this->clusters.clear();
//             std::copy(objs->begin(), objs->begin(), std::back_inserter(clusters));
//             return true;
//         }
//     }
//     ROS_WARN_THROTTLE(30,"[Storage][%s] Passed clusters are empty! Not writing anything",__func__);
//     return false;
// }

bool
Storage::readObjTransforms (std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans)
{
    trans=std::make_shared<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    LOCK guard(mtx_estimations);
    if (estimations.empty()){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tEstimated objects transformations are empty! Not reading anything",__func__);
        trans.reset();
        return false;
    }
    std::copy(estimations.begin(), estimations.end(), std::back_inserter(*trans));
    return true;
}

bool
Storage::writeObjTransforms (std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> trans)
{
    if (trans){
        if (!trans->empty()){
            LOCK guard(mtx_estimations);
            estimations.clear();
            std::copy(trans->begin(), trans->end(), std::back_inserter(estimations));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty estimated transformations! Skipping...",__func__);
    return false;
}

bool
Storage::readObjNames (std::shared_ptr<std::vector<std::pair<std::string,std::string>>> &n)
{
    n=std::make_shared<std::vector<std::pair<std::string, std::string>>>();
    LOCK guard(mtx_names);
    if (names.empty()){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tEstimated objects names are empty! Not reading anything",__func__);
        n.reset();
        return false;
    }
    std::copy(names.begin(), names.end(), std::back_inserter(*n));
    return true;
}

bool
Storage::writeObjNames (std::shared_ptr<std::vector<std::pair<std::string,std::string>>> n)
{
    if (n){
        if (!n->empty()){
            LOCK guard(mtx_names);
            names.clear();
            std::copy(n->begin(), n->end(), back_inserter(names));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty estimated names! Skipping...", __func__);
    return false;
}

bool
Storage::searchObjName(std::string n, int &idx)
{
    idx = -1;
    LOCK guard(mtx_names);
    for (size_t i=0; i<names.size(); ++i){
        if (names[i].first.compare(n) == 0){
            idx = i;
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tObject %s not found among estimated objects...", __func__, n.c_str());
    return false;
}

bool
Storage::readObjTransformByIndex(int idx, std::shared_ptr<Eigen::Matrix4f> &trans)
{
    trans=std::make_shared<Eigen::Matrix4f>();
    LOCK guard(mtx_estimations);
    for (size_t i=0; i<estimations.size(); ++i)
    {
        if ( i == (size_t)idx){
            *trans = estimations[i];
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tIndex %d does not correspond to a particular estimated transform...", __func__, idx);
    trans.reset();
    return false;
}

bool
Storage::writeObjTransformByIndex(int idx, std::shared_ptr<Eigen::Matrix4f> trans)
{
    if (!trans){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty transform! Skipping...",__func__);
        return false;
    }
    LOCK guard(mtx_estimations);
    if (idx >=0 && idx < (estimations.size())){
        estimations.at(idx) = *trans;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tIndex %d is invalid, or no estimations are yet present",__func__, idx);
    return false;
}

bool
Storage::readLeftArm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm)
{
    arm=std::make_shared<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    LOCK guard(mtx_left_arm);
    if (left_arm.empty()){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tVito Left Arm transforms are empty! Not reading anything",__func__);
        arm.reset();
        return false;
    }
    std::copy(left_arm.begin(), left_arm.end(), std::back_inserter(*arm));
    return true;
}

bool
Storage::writeLeftArm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> arm)
{
    if (arm){
        if (!arm->empty()){
            LOCK guard(mtx_left_arm);
            left_arm.clear();
            std::copy(arm->begin(), arm->end(), std::back_inserter(left_arm));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty Vito Left Arm transforms! Skipping...",__func__);
    return false;
}

bool
Storage::readRightArm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm)
{
    arm=std::make_shared<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>>();
    LOCK guard(mtx_right_arm);
    if (right_arm.empty()){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tVito Right Arm transforms are empty! Not reading anything",__func__);
        arm.reset();
        return false;
    }
    std::copy(right_arm.begin(), right_arm.end(),  std::back_inserter(*arm));
    return true;
}

bool
Storage::writeRightArm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> arm)
{
    if (arm){
        if (!arm->empty()){
            LOCK guard(mtx_right_arm);
            right_arm.clear();
            std::copy(arm->begin(), arm->end(),  std::back_inserter(right_arm));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty Vito Right Arm transforms! Skipping...",__func__);
    return false;
}

bool
Storage::readLeftHand(std::shared_ptr<Eigen::Matrix4f> &hand)
{
    hand=std::make_shared<Eigen::Matrix4f>();
    LOCK guard(mtx_left_hand);
    if(left_hand.isZero(1e-2)){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tVito Left Soft Hand transforms is empty! Not reading anything",__func__);
        hand.reset();
        return false;
    }
    *hand = left_hand;
    return true;
}

bool
Storage::writeLeftHand(std::shared_ptr<Eigen::Matrix4f> hand)
{
    if (hand){
        LOCK guard(mtx_left_hand);
        left_hand = *hand;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s]Tried to write empty Left Hand transformation! Skipping...",__func__);
    return false;
}

bool
Storage::readRightHand(std::shared_ptr<Eigen::Matrix4f> &hand)
{
    hand=std::make_shared<Eigen::Matrix4f>();
    LOCK guard(mtx_right_hand);
    if(right_hand.isZero(1e-2)){
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tVito Right Soft Hand transforms is empty! Not reading anything",__func__);
        hand.reset();
        return false;
    }
    *hand = right_hand;
    return true;
}

bool
Storage::writeRightHand(std::shared_ptr<Eigen::Matrix4f> hand)
{
    if (hand){
        LOCK guard(mtx_right_hand);
        right_hand = *hand;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty Right Hand transformation! Skipping...",__func__);
    return false;
}

// void
// Storage::read_table(std::shared_ptr<Eigen::Matrix4f> &t)
// {
//     LOCK guard(mtx_table);
//     if (!t)
//         t.reset(new Eigen::Matrix4f);
//     *t = this->table;
//     return;
// }
//
// bool
// Storage::write_table(std::shared_ptr<Eigen::Matrix4f> &t)
// {
//     if (t){
//         LOCK guard(mtx_table);
//         this->table = *t;
//         return true;
//     }
//     ROS_WARN_THROTTLE(30,"[Storage][%s] Passed Table transformation is empty! Not writing anything in Storage",__func__);
//     return false;
// }

void
Storage::readTrackedIndex(int &idx)
{
    LOCK guard(mtx_index);
    idx = index;
    return;
}

void
Storage::writeTrackedIndex(int idx)
{
    LOCK guard(mtx_index);
    index = idx;
    return;
}

// void
// Storage::read_tracked_box(std::shared_ptr<Box> &b)
// {
//     if(!b)
//         b.reset(new Box);
//     LOCK guard(mtx_bbox);
//     b->x1 = bbox.x1;
//     b->x2 = bbox.x2;
//     b->y1 = bbox.y1;
//     b->y2 = bbox.y2;
//     b->z1 = bbox.z1;
//     b->z2 = bbox.z2;
//     return;
// }
//
// bool
// Storage::write_tracked_box(std::shared_ptr<Box> &b)
// {
//     if (b){
//         LOCK guard(mtx_bbox);
//         bbox.x1= b->x1;
//         bbox.x2= b->x2;
//         bbox.y1= b->y1;
//         bbox.y2= b->y2;
//         bbox.z1= b->z1;
//         bbox.z2= b->z2;
//         return true;
//     }
//     ROS_WARN_THROTTLE(30,"[Storage][%s] Passed BoundingBox limits are empty! Not writing anything in Storage",__func__);
//     return false;
// }
//
// bool
// Storage::read_supervoxels_clusters(std::shared_ptr<std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>> &clus)
// {
//     if (!clus)
//         clus.reset(new std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>);
//     else
//         clus->clear();
//     if (!super_clusters.empty()){
//         LOCK guard(mtx_super_clusters);
//         *clus = super_clusters;
//         return true;
//     }
//     ROS_WARN_THROTTLE(30,"[Storage][%s] Supervoxel clusters from Storage are empty! Not reading anything",__func__);
//     return false;
// }
//
// bool
// Storage::write_supervoxels_clusters(std::shared_ptr<std::map<uint32_t,pcl::Supervoxel<PT>::Ptr>> &clus)
// {
//     if (clus){
//         if (!clus->empty()){
//             LOCK guard(this->mtx_super_clusters);
//             this->super_clusters.clear();
//             this->super_clusters = *clus;
//             return true;
//         }
//     }
//     ROS_WARN_THROTTLE(30,"[Storage][%s] Passed clusters are empty! Not writing anything",__func__);
//     return false;
// }

bool
Storage::readSensorFrame (std::string& frame)
{
    LOCK guard(mtx_sensor_ref_frame);
    frame = sensor_ref_frame;
    return true;
}

bool
Storage::writeSensorFrame (std::string frame)
{
    if (!frame.empty()){
        LOCK guard(mtx_sensor_ref_frame);
        sensor_ref_frame = frame;
        return true;
    }
    else{
        ROS_WARN_THROTTLE(30,"[Storage::%s]\tTried to write empty sensor reference frame! Skipping...",__func__);
        return false;
    }
}
}
