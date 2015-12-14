#include <pacman_vision/storage.h>

//Constructor
Storage::Storage()
{
    this->index = -1;
}

//TODO review pointer references in function arguments
bool
Storage::read_scene(PTC::Ptr &cloud)
{
    cloud.reset(new PTC);
    if (scene){
        if(!scene->empty()){
            LOCK guard(mtx_scene);
            pcl::copyPointCloud(*scene, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Scene from Storage is empty! Not reading anything",__func__);
    return false;
}

bool
Storage::read_scene(PXC::Ptr &cloud)
{
    cloud.reset(new PXC);
    if (scene){
        if (!scene->empty()){
            LOCK guard(mtx_scene);
            pcl::copyPointCloud(*scene, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Scene from Storage is empty! Not reading anything",__func__);
    return false;
}

bool
Storage::read_scene_processed(PTC::Ptr &cloud)
{
    cloud.reset(new PTC);
    if (scene_processed){
        if(!scene_processed->empty()){
            LOCK guard(mtx_scene_processed);
            pcl::copyPointCloud(*scene_processed, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Scene Processed from Storage is empty! Not reading anything",__func__);
    return false;
}

bool
Storage::read_scene_processed(PXC::Ptr &cloud)
{
    cloud.reset(new PXC);
    if(scene_processed){
        if(!scene_processed->empty()){
            LOCK guard(mtx_scene_processed);
            pcl::copyPointCloud(*scene_processed, *cloud);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Scene Processed from Storage is empty! Not reading anything",__func__);
    return false;
}

bool
Storage::write_scene(PTC::Ptr &cloud)
{
    if (cloud){
        if (!cloud->empty()){
            LOCK guard(mtx_scene);
            if (!scene)
                scene.reset(new PTC);
            pcl::copyPointCloud(*cloud, *scene);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed cloud is empty! Not writing anything in Storage",__func__);
    return false;
}

bool
Storage::write_scene_processed(PTC::Ptr &cloud)
{
    if (cloud){
        if (!cloud->empty()){
            LOCK guard(mtx_scene_processed);
            if (!scene_processed)
                scene_processed.reset(new PTC);
            pcl::copyPointCloud(*cloud, *scene_processed);
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed cloud is empty! Not writing anything in Storage",__func__);
    return false;
}

bool
Storage::read_obj_clusters (std::shared_ptr<std::vector<PXC>> &objs)
{
    LOCK guard(mtx_clusters);
    if (!objs)
        objs.reset(new std::vector<PXC>);
    else
        objs->clear();
    if (this->clusters.empty()){
        ROS_WARN_THROTTLE(30,"[Storage][%s] Clusters from Storage are empty! Not reading anything",__func__);
        return false;
    }
    std::copy(clusters.begin(), clusters.end(), std::back_inserter(*objs));
    return true;
}

bool
Storage::write_obj_clusters (std::shared_ptr<std::vector<PXC>> &objs)
{
    if (objs){
        if (!objs->empty()){
            LOCK guard(mtx_clusters);
            this->clusters.clear();
            std::copy(objs->begin(), objs->begin(), std::back_inserter(clusters));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed clusters are empty! Not writing anything",__func__);
    return false;
}

bool
Storage::read_obj_transforms (std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans)
{
    LOCK guard(mtx_estimations);
    if (!trans)
        trans.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);
    else
        trans->clear();
    if (this->estimations.empty())
        return false;
    std::copy(estimations.begin(), estimations.end(), std::back_inserter(*trans));
    return true;
}

bool
Storage::write_obj_transforms (std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans)
{
    if (trans){
        if (!trans->empty()){
            LOCK guard(mtx_estimations);
            this->estimations.clear();
            std::copy(trans->begin(), trans->end(), std::back_inserter(estimations));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed transformations are empty! Not writing anything",__func__);
    return false;
}

bool
Storage::read_obj_names (std::shared_ptr<std::vector<std::pair<std::string,std::string>>> &n)
{
    LOCK guard(mtx_names);
    if (!n)
        n.reset(new std::vector<std::pair<std::string, std::string>>);
    else
        n->clear();
    if (this->names.empty()){
        ROS_WARN_THROTTLE(30,"[Storage][%s] Names of objects from Storage are empty! Not reading anything",__func__);
        return false;
    }
    std::copy(names.begin(), names.end(), std::back_inserter(*n));
    return true;
}

bool
Storage::write_obj_names (std::shared_ptr<std::vector<std::pair<std::string,std::string>>> &n)
{
    if (n){
        if (!n->empty()){
            LOCK guard(mtx_names);
            this->names.clear();
            std::copy(n->begin(), n->end(), back_inserter(names));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed names are empty! Not writing anything", __func__);
    return false;
}

bool
Storage::search_obj_name(std::string n, int &idx)
{
    LOCK guard(mtx_names);
    idx = -1;
    for (int i=0; i<names.size(); ++i){
        if (names[i].first.compare(n) == 0){
            idx = i;
            return true;
        }
    }
    return false;
}

bool
Storage::read_obj_transform_by_index(int idx, std::shared_ptr<Eigen::Matrix4f> &trans)
{
    if (!trans)
        trans.reset(new Eigen::Matrix4f);
    LOCK guard(mtx_estimations);
    for (int i=0; i<estimations.size(); ++i)
    {
        if ( i == idx){
            *trans = estimations[i];
            return true;
        }
    }
    return false;
}

bool
Storage::write_obj_transform_by_index(int idx, std::shared_ptr<Eigen::Matrix4f> &trans)
{
    if (!trans){
        ROS_WARN_THROTTLE(30,"[Storage][%s] Passed transform is empty! Not writing anything",__func__);
        return false;
    }
    LOCK guard(mtx_estimations);
    if (idx >=0 && idx < (estimations.size())){
        estimations.at(idx) = *trans;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed index is invalid, or estimations are not available",__func__);
    return false;
}

bool
Storage::read_left_arm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm)
{
    LOCK guard(mtx_left_arm);
    if (!arm)
        arm.reset(new std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>);
    else
        arm->clear();
    if (this->left_arm.empty()){
        ROS_WARN_THROTTLE(30,"[Storage][%s] Vito Left Arm transforms from Storage are empty! Not reading anything",__func__);
        return false;
    }
    std::copy(left_arm.begin(), left_arm.end(), std::back_inserter(*arm));
    return true;
}

bool
Storage::write_left_arm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm)
{
    if (arm){
        if (!arm->empty()){
            LOCK guard(mtx_left_arm);
            left_arm.clear();
            std::copy(arm->begin(), arm->end(), std::back_inserter(left_arm));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed Arm transforms are empty! Not writing anything in Storage",__func__);
    return false;
}

bool
Storage::read_right_arm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm)
{
    LOCK guard(mtx_right_arm);
    if (!arm)
        arm.reset(new std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>);
    else
        arm->clear();
    if (this->right_arm.empty()){
        ROS_WARN_THROTTLE(30,"[Storage][%s] Vito Right Arm transforms from Storage are empty! Not reading anything",__func__);
        return false;
    }
    std::copy(right_arm.begin(), right_arm.end(),  std::back_inserter(*arm));
    return true;
}

bool
Storage::write_right_arm(std::shared_ptr<std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm)
{
    if (arm){
        if (!arm->empty()){
            LOCK guard(mtx_right_arm);
            right_arm.clear();
            std::copy(arm->begin(), arm->end(),  std::back_inserter(right_arm));
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed Arm transforms are empty! Not writing anything in Storage",__func__);
    return false;
}

void
Storage::read_left_hand(std::shared_ptr<Eigen::Matrix4f> &hand)
{
    LOCK guard(mtx_left_hand);
    if (!hand)
        hand.reset(new Eigen::Matrix4f);
    *hand = this->left_hand;
    return;
}

bool
Storage::write_left_hand(std::shared_ptr<Eigen::Matrix4f> &hand)
{
    if (hand){
        LOCK guard(mtx_left_hand);
        this->left_hand = *hand;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed Hand transformation is empty! Not writing anything in Storage",__func__);
    return false;
}

void
Storage::read_right_hand(std::shared_ptr<Eigen::Matrix4f> &hand)
{
    LOCK guard(mtx_right_hand);
    if (!hand)
        hand.reset(new Eigen::Matrix4f);
    *hand = this->right_hand;
    return;
}

bool
Storage::write_right_hand(std::shared_ptr<Eigen::Matrix4f> &hand)
{
    if (hand){
        LOCK guard(mtx_right_hand);
        this->right_hand = *hand;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed Hand transformation is empty! Not writing anything in Storage",__func__);
    return false;
}

void
Storage::read_table(std::shared_ptr<Eigen::Matrix4f> &t)
{
    LOCK guard(mtx_table);
    if (!t)
        t.reset(new Eigen::Matrix4f);
    *t = this->table;
    return;
}

bool
Storage::write_table(std::shared_ptr<Eigen::Matrix4f> &t)
{
    if (t){
        LOCK guard(mtx_table);
        this->table = *t;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed Table transformation is empty! Not writing anything in Storage",__func__);
    return false;
}

void
Storage::read_tracked_index(int &idx)
{
    LOCK guard(mtx_index);
    idx = this->index;
    return;
}

void
Storage::write_tracked_index(int idx)
{
    LOCK guard(mtx_index);
    this->index = idx;
    return;
}

void
Storage::read_tracked_box(std::shared_ptr<Box> &b)
{
    if(!b)
        b.reset(new Box);
    LOCK guard(mtx_bbox);
    b->x1 = bbox.x1;
    b->x2 = bbox.x2;
    b->y1 = bbox.y1;
    b->y2 = bbox.y2;
    b->z1 = bbox.z1;
    b->z2 = bbox.z2;
    return;
}

bool
Storage::write_tracked_box(std::shared_ptr<Box> &b)
{
    if (b){
        LOCK guard(mtx_bbox);
        bbox.x1= b->x1;
        bbox.x2= b->x2;
        bbox.y1= b->y1;
        bbox.y2= b->y2;
        bbox.z1= b->z1;
        bbox.z2= b->z2;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed BoundingBox limits are empty! Not writing anything in Storage",__func__);
    return false;
}

bool
Storage::read_supervoxels_clusters(std::shared_ptr<std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>> &clus)
{
    if (!clus)
        clus.reset(new std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>);
    else
        clus->clear();
    if (!super_clusters.empty()){
        LOCK guard(mtx_super_clusters);
        *clus = super_clusters;
        return true;
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Supervoxel clusters from Storage are empty! Not reading anything",__func__);
    return false;
}

bool
Storage::write_supervoxels_clusters(std::shared_ptr<std::map<uint32_t,pcl::Supervoxel<PT>::Ptr>> &clus)
{
    if (clus){
        if (!clus->empty()){
            LOCK guard(this->mtx_super_clusters);
            this->super_clusters.clear();
            this->super_clusters = *clus;
            return true;
        }
    }
    ROS_WARN_THROTTLE(30,"[Storage][%s] Passed clusters are empty! Not writing anything",__func__);
    return false;
}

bool
Storage::read_sensor_ref_frame (std::string& frame)
{
    LOCK guard(mtx_sensor_ref_frame);
    frame = this->sensor_ref_frame;
    return true;
}

bool
Storage::write_sensor_ref_frame (std::string& frame)
{
    if (!frame.empty()){
        LOCK guard(mtx_sensor_ref_frame);
        this->sensor_ref_frame = frame;
        return true;
    }
    else{
        return false;
    }
}
