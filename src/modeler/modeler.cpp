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
#include <modeler/modeler.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace pacv
{

Modeler::Modeler(const ros::NodeHandle n, const std::string ns, const Storage::Ptr stor)
    :Module<Modeler>(n,ns,stor), acquiring(false), processing(false)
{
    config=std::make_shared<ModelerConfig>();
    bool run;
    ros::param::get("/pacman_vision/in_hand_modeler/spawn", run);
    config->set("spawn", run);
}

void
Modeler::updateRosparams()
{
    for (const auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if (!config->get(key, val))
            ROS_WARN("[Modeler::%s]\tFailed to get key:%s from Config",__func__,key.c_str());
        else
            nh->setParam(key, val);
    }
}

void
Modeler::init()
{
    if(!nh){
        ROS_ERROR("[Modeler::%s]\tNode Handle not initialized, Module must call spawn() first!",__func__);
        return;
    }
    srv_start = nh->advertiseService("start", &Modeler::cb_start, this);
    srv_stop = nh->advertiseService("stop_recording", &Modeler::cb_stop, this);
    std::string mark_topic(getFatherNamespace()+"/markers");
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>(mark_topic, 1);
    pub_model = nh->advertise<PTC>("model", 1);
    //add subscriber TODO
    acquisition_q.clear();
    T_km.setIdentity();
    T_mk.setIdentity();
    model_ds = boost::make_shared<PTC>();
    model_c = boost::make_shared<PTC>();
    teDQ = boost::make_shared<pcl::registration::TransformationEstimationDualQuaternion<PT,PT,float>>();
    //init node params
    for (auto key: config->valid_keys)
    {
        XmlRpc::XmlRpcValue val;
        if(nh->getParam(key, val))
        {
            if(!config->set(key, val))
                ROS_WARN("[Modeler::%s]\tFailed to set key:%s into Config",__func__,key.c_str());
        }
        else
            ROS_WARN("[Modeler::%s]\tKey:%s not found on parameter server",__func__,key.c_str());
    }
    // testDeltaE();  //tmp test
}

void
Modeler::deInit()
{
    model_c.reset();
    model_ds.reset();
    teDQ.reset();
    acquisition_q.clear();
    first_frame.reset();
}

void
Modeler::computeColorDistribution(const PTC &frame)
{
    model_mean_dE = 0.0;
    mean_L = mean_a = mean_b = 0.0;
    std::vector<double> color_dists;
    //calculate mean color of the frame
    for (const auto& pt: frame.points)
    {
        double L, a, b;
        convertPCLColorToCIELAB(pt, L, a, b);
        mean_L += L;
        mean_a += a;
        mean_b += b;
    }
    mean_L /= frame.size();
    mean_a /= frame.size();
    mean_b /= frame.size();
    //calculate mean deltaE of points from mean color
    for (const auto& pt: frame.points)
    {
        double L, a, b;
        convertPCLColorToCIELAB(pt, L,a,b);
        color_dists.push_back(deltaE(mean_L,mean_a,mean_b, L,a,b));
        model_mean_dE += color_dists.back();
    }
    model_mean_dE /= color_dists.size();
    model_stddev_dE = 0.0;
    //calculate standard deviation of distance distribution
    for (const auto& d: color_dists)
        model_stddev_dE += std::pow(model_mean_dE - d, 2);
    model_stddev_dE /= color_dists.size();
    model_stddev_dE = std::sqrt(model_stddev_dE);
    // std::cout<<"mean L "<<mean_L<<" mean a "<<mean_a<<" mean_b "<<mean_b<<std::endl;
    // std::cout<<"mean deltaE "<<model_mean_dE<<" std Dev deltaE "<<model_stddev_dE<<std::endl;
}
bool
Modeler::colorMetricInclusion(const PT &pt)
{
    config->get("color_std_dev_multiplier", stddev_mul);
    double dEM, dEm;
    dEM =  model_mean_dE + model_stddev_dE*stddev_mul;
    dEm =  model_mean_dE - model_stddev_dE*stddev_mul;
    double L,a,b;
    convertPCLColorToCIELAB(pt, L,a,b);
    double dE = deltaE(mean_L, mean_a, mean_b, L, a, b);
    if (dE > dEM || dE < dEm )
        return false;
    else
        return true;
}

bool
Modeler::cb_start(pacman_vision_comm::start_modeler::Request& req, pacman_vision_comm::start_modeler::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Modeler::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (acquiring){
        //Modeler is already acquiring
        ROS_ERROR("[Modeler::%s]\tModeler is already acquiring!",__func__);
        return false;
    }
    ROS_INFO("[Modeler][%s]\tRecord a motion then call stop_modeler_recording service when satisfied",__func__);
    acquiring = true;
    return true;
}

bool
Modeler::cb_stop(pacman_vision_comm::stop_modeler_recording::Request &req, pacman_vision_comm::stop_modeler_recording::Response &res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Modeler::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (!acquiring){
        //Modeler is already stopped
        ROS_ERROR("[Modeler::%s]\tModeler is already stopped!",__func__);
        return false;
    }
    acquiring = false;
    return true;
}

void
Modeler::spinOnce()
{
    if (acquiring){
        if (!first_frame){
            storage->readSceneProcessed(first_frame);
        }
        else{
            PTC::Ptr frame;
            storage->readSceneProcessed(frame);
            if (!frame->empty()){
                LOCK guard(mtx_acq);
                acquisition_q.push_back(*frame);
            }
        }
    }
    if (acquiring && !processing){
        processing = true;
        proc_t = std::thread(&Modeler::processQueue, this);
    }
    if (!acquiring && !processing && proc_t.joinable())
        proc_t.join();
    publishModel();
}

bool
Modeler::checkFramesSimilarity(PTC::Ptr current, PTC::Ptr next, float factor)
{
    pcl::octree::OctreePointCloudChangeDetector<PT> octcd(0.01);
    octcd.setInputCloud(current);
    octcd.addPointsFromInputCloud();
    octcd.switchBuffers();
    octcd.setInputCloud(next);
    octcd.addPointsFromInputCloud();
    std::vector<int> changes;
    octcd.getPointIndicesFromNewVoxels(changes);
    if (changes.size() <= next->size() * factor)
        //less than factor% points are changed, frames are similar
        return true;
    else
        //more than factor% points are changed, frames are different
        return false;
}

void
Modeler::processQueue()
{
    while (!first_frame)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ROS_INFO("[Modeler::%s]\tStarted",__func__);
    std::size_t acq_size(1);
    T_remean.setZero();
    T_sm.setZero();
    pcl::StatisticalOutlierRemoval<PT> sor;
    PTC::Ptr current = boost::make_shared<PTC>();
    sor.setInputCloud(first_frame);
    sor.setMeanK(80);
    sor.setStddevMulThresh(2.5);
    sor.filter(*current);
    bool color_f;
    config->get("use_color_filtering", color_f);
    if (color_f)
        computeColorDistribution(*current);

    // pcl::visualization::PCLVisualizer viz;
    // viz.addPointCloud<PT>(current);
    // viz.addCoordinateSystem(0.1);
    // viz.spinOnce(500);

    while (acquiring || acq_size>0)
    {
        if (this->isDisabled()){
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            continue;
        }
        PTC::Ptr front_ptr;
        mtx_acq.lock();
        acq_size = acquisition_q.size();
        mtx_acq.unlock();
        ROS_INFO_THROTTLE(30,"[Modeler::%s]\tStill %d frames left to process",__func__,(int)acq_size);
        if (acq_size > 0 && !front_ptr){
            front_ptr = boost::make_shared<PTC>();
            LOCK guard(mtx_acq);
            pcl::copyPointCloud(acquisition_q.front(), *front_ptr);
            acquisition_q.pop_front();
            acq_size = acquisition_q.size();
        }
        if (!front_ptr){
            //Wait for more frames to be acquired
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            LOCK guard(mtx_acq);
            acq_size = acquisition_q.size();
            continue;
        }
        config->get("use_color_filtering", color_f);
        if (color_f){
            pcl::IndicesPtr kept_points = boost::make_shared<std::vector<int>>();
            for (std::size_t i=0; i< front_ptr->points.size(); ++i)
            {
                if (colorMetricInclusion(front_ptr->points[i]))
                    kept_points->push_back(i);
            }
            pcl::ExtractIndices<PT> eif;
            PTC filtered;
            eif.setInputCloud(front_ptr);
            eif.setIndices(kept_points);
            eif.filter(filtered);
            // sor.setInputCloud(filtered.makeShared());
            // sor.setMeanK(80);
            // sor.setStddevMulThresh(2.5);
            // sor.filter(*front_ptr);
            pcl::copyPointCloud(filtered, *front_ptr);
        }
        //remove similar frames
        if (checkFramesSimilarity(current, front_ptr))
            //old and  new frames are  almost equal in point  differences
            //we can remove the new frame and wait for more informative one.
            continue;
        //Align front frame over current frame
        pcl::VoxelGrid<PT> vg;
        Eigen::Vector4f centroid;
        if (T_remean.isZero()){
            pcl::compute3DCentroid(*current, centroid);
            pcl::demeanPointCloud(*current, centroid, *model_c);
            T_remean.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
            T_remean.row(3) = Eigen::Vector4f::Zero();
            T_remean.col(3) = centroid;
            T_remean(3,3) = 1;
            vg.setInputCloud(model_c);
            double leaf;
            config->get("model_ds_leaf", leaf);
            vg.setLeafSize(leaf,leaf,leaf);
            PTC ds;
            // vg.filter(ds);
            // sor.setInputCloud(ds.makeShared());
            // sor.setMeanK(50);
            // sor.setStddevMulThresh(2.0);
            LOCK guard(mtx_model);
            // sor.filter(*model_ds);
            vg.filter(*model_ds);
        }
        PTC::Ptr aligned;
        Eigen::Matrix4f T = alignFrames(front_ptr, current, aligned);
        //compose the model
        // PTC::Ptr refined;
        // refineFrames(aligned, refined);
        if (T_sm.isZero())
            T_sm = T*T_remean;

        // viz.updatePointCloud<PT>(aligned);
        // viz.spinOnce(500);

        PTC::Ptr model_in_scene;
        PTC::Ptr model= boost::make_shared<PTC>();
        mtx_model.lock();
        pcl::copyPointCloud(*model_ds, *model);
        mtx_model.unlock();
        T_sm = alignFrames(aligned, model, model_in_scene, T_sm, 0.02);
        *model_in_scene += *aligned;

        // viz.updatePointCloud<PT>(model_in_scene.makeShared());
        // viz.spinOnce(2000);

        pcl::compute3DCentroid(*model_in_scene, centroid);
        pcl::demeanPointCloud(*model_in_scene, centroid, *model_c);
        T_remean.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        T_remean.row(3) = Eigen::Vector4f::Zero();
        T_remean.col(3) = centroid;
        T_remean(3,3) = 1;
        // *model_c += *aligned;
        vg.setInputCloud(model_c);
        double leaf;
        config->get("model_ds_leaf", leaf);
        vg.setLeafSize(leaf,leaf,leaf);
        PTC ds;
        vg.filter(ds);
        // LOCK guard(mtx_model);
        // vg.filter(*model_ds);
        sor.setInputCloud(ds.makeShared());
        sor.setMeanK(80);
        sor.setStddevMulThresh(3.5);
        // current = refined;
        LOCK guard(mtx_model);
        sor.filter(*model_ds);
        current = front_ptr;
    }//EndWhile
    //last frame
    // if (current){
    //     if (!current->empty()){
    //         PTC::Ptr refined;
    //         refineFrames(current, refined);
    //         *model_c += *refined;
    //         pcl::VoxelGrid<PT> vg;
    //         vg.setInputCloud(model_c);
    //         double leaf;
    //         config->get("model_ds_leaf", leaf);
    //         vg.setLeafSize(leaf,leaf,leaf);
    //         PTC ds;
    //         vg.filter(ds);
    //         sor.setInputCloud(ds.makeShared());
    //         sor.setMeanK(50);
    //         sor.setStddevMulThresh(3.0);
    //         LOCK guard (mtx_model);
    //         sor.filter(*model_ds);
    //     }
    // }
    ROS_INFO("[Modeler::%s]\tStopped",__func__);
    processing = false;
    first_frame.reset();

    //debug visualization
    // pcl::visualization::PCLVisualizer viz;
    // viz.addPointCloud<PT>(processing_q.front().makeShared());
    // viz.spinOnce(500);
    // for (size_t i=1; i<processing_q.size(); ++i)
    // {
    //     viz.updatePointCloud<PT>(processing_q[i].makeShared());
    //     viz.spinOnce(500);
    // }
    // viz.close();
    ///////////////////////////
}

Eigen::Matrix4f
Modeler::alignFrames(PTC::Ptr target, PTC::Ptr source, PTC::Ptr &aligned, const Eigen::Matrix4f &guess, const float dist)
{
    icp.setInputTarget(target);
    icp.setInputSource(source);
    icp.setEuclideanFitnessEpsilon(1e-7);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(dist);
    icp.setUseReciprocalCorrespondences(true);
    icp.setTransformationEstimation(teDQ);
    // icp.setCorrespondenceRandomness(10);
    // icp.setMaximumOptimizerIterations(50);
    // icp.setRotationEpsilon(1e-5);
    aligned = boost::make_shared<PTC>();
    icp.align(*aligned, guess);
    // pcl::transformPointCloud(*source, *aligned, icp.getFinalTransformation());
    return icp.getFinalTransformation();
}

Eigen::Matrix4f
Modeler::refineFrames(PTC::Ptr frame, PTC::Ptr &refined, const Eigen::Matrix4f &guess,  const float dist)
{
    icp.setInputTarget(frame);
    icp.setInputSource(model_ds);
    icp.setEuclideanFitnessEpsilon(5e-5);
    icp.setMaximumIterations(10);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(dist);
    icp.setUseReciprocalCorrespondences(true);
    icp.setTransformationEstimation(teDQ);
    refined = boost::make_shared<PTC>();
    icp.align(*refined, guess*T_remean);
    pcl::transformPointCloud(*model_ds, *refined, icp.getFinalTransformation());
    return icp.getFinalTransformation();
}

void
Modeler::publishModel()
{
    std::string ref_frame;
    storage->readSensorFrame(ref_frame);
    LOCK guard(mtx_model);
    if (model_ds)
        if (!model_ds->empty() && pub_model.getNumSubscribers()>0){
            model_ds->header.frame_id=ref_frame;
            pub_model.publish(model_ds);
        }
}
} //namespace

// bool
// InHandModeler::computeModelTransform(PT pt, float nx, float ny, float nz)
// {
//     try
//     {
//         Eigen::Vector3f new_z(nx,ny,nz); //Z axis is the plane normal
//         new_z.normalize();
//         //Find a vector as close as possible to x-kinect but orthonormal to new_z
//         Eigen::Vector3f kin_x(Eigen::Vector3f::UnitX());
//         Eigen::Vector3f new_x;
//         new_x = kin_x - (new_z*(new_z.dot(kin_x)));
//         new_x.normalize();
//         //New_y is new_z cross new_x;
//         Eigen::Vector3f new_y;
//         new_y = new_z.cross(new_x);
//         new_y.normalize();
//
//         #<{(|
//          * Rotation matrix of new model reference system with respect to
//          * kinect ref. frame. Translation that express model ref system centre
//          * with respect to kinect (the clicked point).
//          * Then Compose the 4x4 rototraslation
//          |)}>#
//         T_km << new_x[0], new_y[0], new_z[0], pt.x,
//                 new_x[1], new_y[1], new_z[1], pt.y,
//                 new_x[2], new_y[2], new_z[2], pt.z,
//                 0,        0,        0,        1;
//         T_mk = T_km.inverse();
//         has_transform = true;
//     }
//     catch (...)
//     {
//         return (false);
//     }
//     return (true);
// }
// void InHandModeler::cb_clicked (const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//     #<{(|
//      * if(!ignore_clicked_point && !do_acquisition){
//      *     float nx,ny,nz;
//      *     PT pt;
//      *     //Get clicked point
//      *     pt.x = msg->point.x;
//      *     pt.y = msg->point.y;
//      *     pt.z = msg->point.z;
//      *     std::string sensor;
//      *     PC::Ptr scene;
//      *     this->storage->read_scene_processed(scene);
//      *     this->storage->read_sensor_ref_frame(sensor);
//      *     if (msg->header.frame_id.compare(sensor) != 0){
//      *         //Put the point read from rviz into kinect reference frame (if needed)
//      *         tf_listener.waitForTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
//      *         tf::StampedTransform t_msg;
//      *         tf_listener.lookupTransform(sensor.c_str(), msg->header.frame_id.c_str(), ros::Time(0), t_msg);
//      *         Eigen::Matrix4f T;
//      *         geometry_msgs::Pose pose;
//      *         fromTF(t_msg, T, pose);
//      *         PT pt_t;
//      *         pt_t = pcl::transformPoint<PT>(pt, Eigen::Affine3f(T));
//      *         //now the point is in kinect ref. frame.
//      *         pt.x = pt_t.x;
//      *         pt.y = pt_t.y;
//      *         pt.z = pt_t.z;
//      *     }
//      *     //compute a normal around the point neighborhood (2cm)
//      *     pcl::search::KdTree<PT> kdtree;
//      *     std::vector<int> idx(scene->points.size());
//      *     std::vector<float> dist(scene->points.size());
//      *     kdtree.setInputCloud(scene);
//      *     kdtree.radiusSearch(pt, 0.02, idx, dist);
//      *     pcl::NormalEstimationOMP<PT, pcl::Normal> ne_point;
//      *     ne_point.setInputCloud(scene);
//      *     ne_point.useSensorOriginAsViewPoint();
//      *     float curv;
//      *     ne_point.computePointNormal (*scene, idx, nx,ny,nz, curv);
//      *     //Compute model transform
//      *     if (!computeModelTransform(pt,nx,ny,nz)){
//      *         ROS_WARN("[InHandModeler][%s]\tFailed to compute Model Transform, please click again!", __func__);
//      *         return;
//      *     }
//      *     //save this scene into the sequence, so that we can use it as a start.
//      *     if (!cloud_sequence.empty())
//      *         cloud_sequence.clear();
//      *     cloud_sequence.push_back(scene);
//      * }
//      * else{
//      *     ROS_INFO("[InHandModeler][%s]Ignoring clicked point as requested or because already started processing...", __func__);
//      * }
//      |)}>#
// }
//
// void
// InHandModeler::alignSequence()
// {
//     {
//         LOCK guard(mtx_seq);
//         align_it = cloud_sequence.begin();
//         //initialize model with first frame.
//         //Assume first frame show really few hand points, at best none.
//         LOCK guard_m(mtx_model);
//         model_f.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
//         model_n.reset(new NC);
//         model.reset(new PC);
//         vg.setLeafSize(leaf, leaf, leaf);
//         vg.setInputCloud(cloud_sequence.front().makeShared());
//         vg.filter(*model);
//     }
//     nh_alignment = ros::NodeHandle(nh, "frame_alignment");
//     queue_alignment.reset(new ros::CallbackQueue);
//     nh_alignment.setCallbackQueue(&(*this->queue_alignment));
//     pcl::visualization::PCLVisualizer v("registration"); //todo temp visualization
//     PC::Ptr tmp(new PC);
//     v.addPointCloud(tmp, "source");
//     //end tmp visualization
//     ROS_INFO("[InHandModeler][%s]\tStarting Sequence Alignment",__func__);
//     while (do_alignment)
//     {
//         //Next element, previous is always front (or begin)
//         std::list<PC>::iterator end;
//         {
//             LOCK guard(mtx_seq);
//             align_it = cloud_sequence.begin();
//             end = cloud_sequence.end();
//         }
//         ++align_it;
//         if (!done_frame_fusion && align_it == fuse_it){
//             //We have to wait for frame fusion
//             boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//             continue;
//         }
//         if (done_frame_fusion && align_it == end){
//             ROS_INFO("[InHandModeler][%s]\tFinished Sequence Alignment",__func__);
//             done_alignment = true;
//             //Leave do_alignment = true so thread wont start again
//             //It will get reset by a new start anyway
//             break;
//         }
//         //tmp visualization color points that have features in red
//         uint8_t r=255, g=0, b=0;
//         uint32_t rgb = ( (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b );
//         for (size_t i=0; i<source_p_idx.size(); ++i)
//             source->points.at(source_p_idx[i]).rgb = *reinterpret_cast<float*>(&rgb);
//         v.updatePointCloud(source, "source");
//         r=0;
//         b=255;
//         Eigen::Vector3f t(0.3,0,0);
//         Eigen::Quaternionf R(1,0,0,0);
//         pcl::transformPointCloud(*target, *tmp,t, R);
//         rgb = ( (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b );
//         for (size_t i=0; i<tmp->size(); ++i)
//             tmp->points[i].rgb = *reinterpret_cast<float*>(&rgb);
//         v.addPointCloud(tmp,"target");
//         v.spinOnce(1000,true);
//         //end tmp
//         {
//             LOCK guard(mtx_seq);
//             cloud_sequence.pop_front();
//         }
//         //Find correspondences between source and target
//         SearchT tree (true, CreatorT(new IndexT(4)));
//         tree.setPointRepresentation (RepT(new pcl::DefaultFeatureRepresentation<pcl::FPFHSignature33>));
//         tree.setChecks(256);
//         tree.setInputCloud(target_f);
//         //Search source features over target features
//         //If source features are n, these will be n*k_nn matrices
//         std::vector<std::vector<int>> k_idx;
//         std::vector<std::vector<float>> k_dist;
//         tree.nearestKSearch (*source_f, source_p_idx, 1, k_idx, k_dist);
//         boost::shared_ptr<pcl::Correspondences> corr_s_over_t_pre (new pcl::Correspondences);
//         //define a distance threshold
//         float dist_thresh = 300.0f;
//         for(size_t i=0; i < k_idx.size(); ++i)
//         {
//             for(size_t k=0; k < k_idx[i].size(); ++k)
//             {
//                 if (k_dist[i][k] < dist_thresh){
//                     PT p1 (source->points[source_p_idx[i]]);
//                     PT p2 (target->points[k_idx[i][k]]);
//                     Eigen::Vector3f diff (p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
//                     float eu_dist = diff.squaredNorm();
//                     //Add a correspondence only if distance is below threshold
//                     pcl::Correspondence cor(source_p_idx[i], k_idx[i][k], eu_dist);
//                     corr_s_over_t_pre->push_back(cor);
//                 }
//             }
//         }
//         //reject too far points
//         pcl::Correspondences corr_s_over_t;
//         cr.setMaximumDistance(0.01);
//         cr.getRemainingCorrespondences(*corr_s_over_t_pre, corr_s_over_t);
//         if(corr_s_over_t.size() < 3){
//             ROS_ERROR("[InHandModeler][%s]\tToo few correspondences found... abort",__func__);
//             break;
//             // TODO: Add a  better error handling, right now  it just terminates
//             // thread! (Wed 11 Nov 2015 02:58:44 PM CET -- tabjones)
//         }
//         //tmp visualization
//         std::string name("corr");
//         v.addCorrespondences<pcl::PointXYZRGB>(source, tmp, corr_s_over_t, name);
//         v.spinOnce(1000,true);
//         v.removeShape(name);
//         v.removePointCloud("target");
//         //end tmp
//         //Estimate the rigid transformation of source -> target
//         Eigen::Matrix4f frame_trans;
//         teDQ->estimateRigidTransformation(*source, *target, corr_s_over_t, frame_trans);
//         PC::Ptr source_aligned (new PC);
//         pcl::transformPointCloud(*source, *source_aligned, frame_trans);
//         //tmp vis
//         r =0;
//         g =255;
//         b =0;
//         rgb = ( (uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b );
//         for (size_t i=0; i<source_aligned->size(); ++i)
//             source_aligned->points[i].rgb = *reinterpret_cast<float*>(&rgb);
//         v.addPointCloud(source_aligned, "source_align");
//         v.removePointCloud("source");
//         v.addPointCloud(target, "target");
//         v.spinOnce(3000,true);
//         v.addPointCloud(source, "source");
//         v.removePointCloud("target");
//         v.removePointCloud("source_align");
//         //end tmp
// //         alignment.setmaxcorrespondencedistance(8.0f*leaf);
// //         alignment.setinputsource(source);
// //         // alignment.setindices(same);
// //         alignment.setsourcefeatures(source_f);
// //         alignment.setinputtarget(target);
// //         alignment.settargetfeatures(target_f);
// //         pc::ptr source_aligned(new pc);
// //         pcl::scopetime t("alignment");
// //         alignment.align(*source_aligned);
// //         if (alignment.hasconverged()){
// //             //save result into sequence to be used as next target
// //             {
// //                 eigen::matrix4f t = alignment.getfinaltransformation();
// //                 lock guard(mtx_sequence);
// //                 pcl::transformpointcloud(*cloud_sequence.front(),*source_aligned, t);
// //                 cloud_sequence.front() = source_aligned;
// //             }
// //             //update model
// //             pc::ptr tmp (new pc);
// //             {
// //                 lock guard(mtx_model);
// //                 *model += *source_aligned;
// //                 if(model->points.size() > 1e4){
// //                     vg.setinputcloud(model);
// //                     vg.setleafsize(0.001, 0.001, 0.001);
// //                     vg.filter(*tmp);
// //                     pcl::copypointcloud(*tmp, *model);
// //                 }
// //                 vg.setinputcloud(model);
// //                 vg.setleafsize(model_ls, model_ls, model_ls);
// //                 vg.filter(*tmp);
// //                 pcl::transformpointcloud(*tmp, *model_ds, t_mk);
// //             }
// //         }
// //         //todo add octomap hand removal
// //         else{
// //             ros_error("[inhandmodeler][%s]alignment failed!",__func__);
// //             //todo add error handling and termination
// //         }
// //         {
// //             lock guard(mtx_sequence);
// //             if(cloud_sequence.size() < 2)
// //             {
// //                 ros_info("[inhandmodeler][%s]finished alignment!", __func__);
// //                 break;
// //             }
// //         }
//         queue_alignment->callAvailable(ros::WallDuration(0));
//         boost::this_thread::sleep(boost::posix_time::milliseconds(10));
//     }//EndWhile
//     nh_alignment.shutdown();
//     return;
// }
//
// void
// InHandModeler::fuseSimilarFrames()
// {
//     {
//         LOCK guard(mtx_seq);
//         fuse_it = cloud_sequence.begin();
//     }
//     nh_fusion = ros::NodeHandle(nh, "frame_fusion");
//     queue_fusion.reset(new ros::CallbackQueue);
//     nh_fusion.setCallbackQueue(&(*this->queue_fusion));
// }
//
// void
// InHandModeler::spin_once()
// {
//     if (has_transform){
//         tf::Transform t_km;
//         geometry_msgs::Pose pose;
//         fromEigen(T_km, pose, t_km);
//         std::string sensor_ref_frame;
//         this->storage->read_sensor_ref_frame(sensor_ref_frame);
//         tf_broadcaster.sendTransform(tf::StampedTransform(t_km, ros::Time::now(), sensor_ref_frame.c_str(), "in_hand_model_frame"));
//     }
//     if (frames>5 && !do_frame_fusion){
//         //start fusion of two frames if necessary
//         do_frame_fusion = true;
//         fusion_driver = boost::thread(&InHandModeler::fuseSimilarFrames, this);
//     }
//     if (not_fused>2 && !do_alignment){
//         //time to start the alignment
//         do_alignment = true;
//         alignment_driver = boost::thread(&InHandModeler::alignSequence, this);
//     }
//     if (model_ds)
//         if(!model_ds->empty() && pub_model.getNumSubscribers()>0)
//             pub_model.publish(*model_ds);
//     //process this module callbacks
//     this->queue_ptr->callAvailable(ros::WallDuration(0));
//     LOCK guard(mtx_seq);
//     if (cloud_sequence.size()>1)
//         ROS_INFO_DELAYED_THROTTLE(10,"Frames left to process: %d", (int)cloud_sequence.size()-1);
// }
// }
