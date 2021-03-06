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
    :Module<Modeler>(n,ns,stor), acquiring(false), processing(false), viz_spin(false),
    fitness(0.0), bad_align(false)
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
    srv_reset = nh->advertiseService("reset_model", &Modeler::cb_reset_model, this);
    srv_save = nh->advertiseService("save_model", &Modeler::cb_save_model, this);
    std::string mark_topic(getFatherNamespace()+"/markers");
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>(mark_topic, 1);
    pub_model = nh->advertise<PNTC>("model", 1);
    //add subscriber TODO
    acquisition_q.clear();
    T_km.setIdentity();
    T_mk.setIdentity();
    model_ds = boost::make_shared<PNTC>();
    model_c = boost::make_shared<PNTC>();
    T_ms.setZero();
    teDQ = boost::make_shared<pcl::registration::TransformationEstimationDualQuaternion<PNT,PNT,float>>();
    cebp = boost::make_shared<pcl::registration::CorrespondenceEstimationBackProjection<PNT,PNT,PNT>>();
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
    T_ms.setZero();
}

bool
Modeler::cb_reset_model(pacman_vision_comm::reset_model::Request& req, pacman_vision_comm::reset_model::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Modeler::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if(acquiring){
        ROS_WARN("[Modeler::%s]\tModeler is still acquiring, call stop_recording first!",__func__);
        return false;
    }
    if (!acquiring && processing && proc_t.joinable()){
        ROS_INFO("[Modeler::%s]\tStopping processing ...",__func__);
        mtx_acq.lock();
        acquisition_q.clear();
        mtx_acq.unlock();
        proc_t.join();
    }
    T_ms.setZero();
    model_c = boost::make_shared<PNTC>();
    model_ds = boost::make_shared<PNTC>();
    return true;
}

bool
Modeler::cb_save_model(pacman_vision_comm::save_model::Request& req, pacman_vision_comm::save_model::Response& res)
{
    if (isDisabled()){
        //Module was temporary disabled, notify the sad user, then exit
        ROS_ERROR("[Modeler::%s]\tNode is globally disabled, this service is suspended!",__func__);
        return false;
    }
    if (model_ds && !model_ds->empty()){
        if (!req.model_path.empty()){
            pcl::PCDWriter writer;
            if ( writer.writeASCII(req.model_path.c_str(), *model_ds ) ==0 )
                ROS_INFO("[Modeler::%s]\tModel saved to %s", __func__, req.model_path.c_str());
            else{
                ROS_ERROR("[Modeler::%s]\tFailed to save model to %s", __func__, req.model_path.c_str());
                return false;
            }
        }
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*model_ds, msg);
        res.model = msg;
        return true;
    }
    else{
        ROS_ERROR("[Modeler::%s]\tModeler does not have a model to save!",__func__);
        return false;
    }
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
        viz_spin=true;
        viz_t = std::thread(&Modeler::spinVisualizer, this);
    }
    if (!acquiring && !processing && proc_t.joinable()){
        proc_t.join();
        viz_spin = false;
        viz_t.join();
    }
    publishModel();
}

bool
Modeler::checkFrameSimilarity(PTC::Ptr frame, PTC::Ptr next, float factor)
{
    pcl::octree::OctreePointCloudChangeDetector<PT> octcd(0.005);  //TODO need to be function of scene leaf size
    octcd.setInputCloud(frame);
    octcd.addPointsFromInputCloud();
    octcd.switchBuffers();
    octcd.setInputCloud(next);
    octcd.addPointsFromInputCloud();
    std::vector<int> changes;
    octcd.getPointIndicesFromNewVoxels(changes);
    //changes contains points that were not in previous buffer (i.e. model)
    if (changes.size() <= next->points.size() * factor)
        //less than factor% points are changed, this frame is uninportant
        return true;
    else
        //more than factor% points are changed
        return false;
}
bool
Modeler::initModel(PTC::Ptr frame)
{
    if (!model_c){
        ROS_ERROR("[Modeler::%s]\tModel pointer empty, something went wrong in node initialization, reset model with service and try again",__func__);
        return false;
    }
    if (!frame){
        ROS_ERROR("[Modeler::%s]\tFrame pointer empty",__func__);
        return false;
    }
    PNTC tmp;
    pcl::NormalEstimationOMP<PT, PNT> ne;
    ne.setRadiusSearch(0.01); //TODO make it function of scene leaf size
    ne.useSensorOriginAsViewPoint();
    ne.setInputCloud(frame);
    ne.compute(tmp);
    if (tmp.size() != frame->size()){
        ROS_ERROR("[Modeler::%s]\tModel initialization error",__func__);
        return false;
    }
    for (std::size_t i =0; i<frame->size(); ++i)
    {
        tmp.points.at(i).x = frame->points.at(i).x;
        tmp.points.at(i).y = frame->points.at(i).y;
        tmp.points.at(i).z = frame->points.at(i).z;
        tmp.points.at(i).r = frame->points.at(i).r;
        tmp.points.at(i).g = frame->points.at(i).g;
        tmp.points.at(i).b = frame->points.at(i).b;
    }
    if (model_c->empty())
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(tmp, centroid);
        if (T_ms.isZero()){
            T_ms.setIdentity();
            T_ms.col(3) = -centroid;
            T_ms(3,3) = 1;
        }
        pcl::transformPointCloudWithNormals(tmp, *model_c, T_ms);
    }
    else{
        PNTC::Ptr aligned;
        T_ms = alignFrame(tmp.makeShared(), aligned, T_ms);
        *model_c += *aligned;
    }
    pcl::VoxelGrid<PNT> vg;
    pcl::StatisticalOutlierRemoval<PNT> sor;
    vg.setInputCloud(model_c);
    double leaf;
    config->get("model_ds_leaf", leaf);
    vg.setLeafSize(leaf,leaf,leaf);
    vg.setDownsampleAllData(true);
    PNTC ds;
    vg.filter(ds);
    sor.setInputCloud(ds.makeShared());
    sor.setMeanK(100);
    sor.setStddevMulThresh(10.0);
    LOCK guard(mtx_model);
    sor.filter(*model_ds);
    return true;
}

bool
enforceNormalSimilarity (const PNT& pa, const PNT& pb, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> na = pa.normal;
    Eigen::Map<const Eigen::Vector3f> nb = pb.normal;
    float cos_angle = na.dot(nb);
    float curv_dist = std::fabs(pa.curvature - pb.curvature);
    // if (squared_distance <= 1e-4){
        if (cos_angle >= normal_similarity_thresh && curv_dist <= curv_thresh)
            return true;
    // }
    return false;
}

void
Modeler::processQueue()
{
    while (!first_frame)
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ROS_INFO("[Modeler::%s]\tStarted",__func__);
    if (!initModel(first_frame)){
        processing=false;
        acquiring=false;
        first_frame.reset();
        return;
    }
    std::size_t acq_size(1);
    //alignment loop
    while (acquiring || acq_size>0)
    {
        if (this->isDisabled()){
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            continue;
        }
        PTC::Ptr front_ptr;
        ROS_INFO_DELAYED_THROTTLE(30,"[Modeler::%s]\tStill %d frames left to process",__func__,(int)acq_size);
        {
            LOCK guard(mtx_acq);
            if (!acquisition_q.empty()){
                front_ptr = boost::make_shared<PTC>();
                pcl::copyPointCloud(acquisition_q.front(), *front_ptr);
                acquisition_q.pop_front();
                acq_size = acquisition_q.size();
            }
        }
        if (!front_ptr){
            //Wait for more frames to be acquired
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            LOCK guard(mtx_acq);
            acq_size = acquisition_q.size();
            continue;
        }
        {
            LOCK guard(mtx_acq);
            PTC::Ptr next_ptr = boost::make_shared<PTC>();
            if (!acquisition_q.empty()){
                //remove similar frames
                pcl::copyPointCloud(acquisition_q.front(), *next_ptr);
                if (checkFrameSimilarity(front_ptr, next_ptr, 0.1)){
                    //this frame does not add much to the model
                    //we can skip it and wait for a more informative one.
                    acquisition_q.pop_front();
                    acq_size = acquisition_q.size();
                    continue;
                }
            }
        }
        //Align front frame over the model
        //read normals ang threshold
        double ang_thresh;
        config->get("normals_ang_thresh", ang_thresh);
        config->get("curvature_thresh", curv_thresh);
        normal_similarity_thresh = std::cos(ang_thresh*D2R);
        //compute normals
        pcl::NormalEstimationOMP<PT, PNT> ne;
        ne.setRadiusSearch(0.01);
        ne.useSensorOriginAsViewPoint();
        ne.setInputCloud(front_ptr);
        PNTC::Ptr front_with_normals = boost::make_shared<PNTC>();
        ne.compute(*front_with_normals);
        for (size_t i=0; i<front_ptr->points.size(); ++i)
        {
            front_with_normals->at(i).x = front_ptr->at(i).x;
            front_with_normals->at(i).y = front_ptr->at(i).y;
            front_with_normals->at(i).z = front_ptr->at(i).z;
            front_with_normals->at(i).r = front_ptr->at(i).r;
            front_with_normals->at(i).g = front_ptr->at(i).g;
            front_with_normals->at(i).b = front_ptr->at(i).b;
        }
        PNTC::Ptr aligned;
        //get rough transform estimate to use as guess
        // T_ms = alignFrame(front_with_normals, aligned, T_ms, 0.1);
        //get a model copy
        mtx_model.lock();
        pcl::copyPointCloud(*model_ds, *model_c);
        mtx_model.unlock();

        //clusterize the frame and align each one on model
        // pcl::SupervoxelClustering<PT> svc(0.005, 0.15, true);
        // NTC::Ptr normals = boost::make_shared<NTC>();
        // for (const auto &pt: front_with_normals->points)
        // {
        //     NT n;
        //     n.normal_x = pt.normal_x;
        //     n.normal_y = pt.normal_y;
        //     n.normal_z = pt.normal_z;
        //     normals->push_back(n);
        // }
        // std::map<uint32_t, pcl::Supervoxel<PT>::Ptr> clusters;
        // svc.setInputCloud(front_ptr);
        // svc.setNormalCloud(normals);
        // svc.setColorImportance(0.3);
        // svc.setSpatialImportance(0.5);
        // svc.setNormalImportance(0.5);
        // svc.extract(clusters);
        // svc.refineSupervoxels(2, clusters);
        pcl::ConditionalEuclideanClustering<PNT> cec (false);
        pcl::IndicesClustersPtr clusters = boost::make_shared<pcl::IndicesClusters>();
        // pcl::IndicesClustersPtr too_small_clusters = boost::make_shared<pcl::IndicesClusters>();
        // pcl::IndicesClustersPtr too_big_clusters = boost::make_shared<pcl::IndicesClusters>();
        cec.setInputCloud(front_with_normals);
        cec.setConditionFunction(&enforceNormalSimilarity);
        cec.setClusterTolerance(0.01);
        cec.setMinClusterSize(100);
        cec.setMaxClusterSize(front_with_normals->size());
        cec.segment(*clusters);
        // cec.getRemovedClusters(too_small_clusters, too_big_clusters);
        // PNTC ali;
        // for (auto it=clusters.begin(); it!=clusters.end(); ++it)
        // {
        std::vector<double> fits;
        std::vector<PNTC> aligns;
        Eigen::MatrixX4f trans;
        trans.resize(4*clusters->size(), 4);
        aligns.resize(clusters->size());
        size_t i (0);
        for (const auto &cl: *clusters)
        {
            PNTC::Ptr cluster = boost::make_shared<PNTC>();
            pcl::copyPointCloud(*front_with_normals, cl, *cluster);
            Eigen::Matrix4f T = alignFrame(cluster, aligned, T_ms, 0.05, true);
            trans.block<4,4>(i*4,0)=alignFrame(front_with_normals, aligned, T, 0.007, true);
            fits.push_back(fitness);
            pcl::copyPointCloud(*aligned, aligns[i]);
            ++i;
        }
        i=0;
        double min=500;
        for(size_t k=0; k<fits.size(); ++k)
        {
            if(fits[k]<=min){
                min=fits[k];
                i=k;
            }
        }
        *model_c += aligns[i];
        T_ms = trans.block<4,4>(i*4,0);

        {
            //debug viz
            LOCK guard(mtx_viz);
            s->clear();
            t->clear();
            a->clear();
            ns = boost::make_shared<PNC>();
            nt = boost::make_shared<PNC>();
            for (const auto &pt: aligns[i].points)
            {
                pcl::PointXYZRGBA p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z;
                p.r = 0;
                p.g = 255;
                p.b = 0;
                p.a = 255;
                a->push_back(p);
            }
            for (const auto &pt: front_with_normals->points)
            {
                pcl::PointXYZRGBA p;
                PN pn;
                p.x = pn.x = pt.x;
                p.y = pn.y = pt.y;
                p.z = pn.z = pt.z;
                p.r=255;
                p.g = 0;
                p.b = 0;
                pn.normal_x = pt.normal_x;
                pn.normal_y = pt.normal_y;
                pn.normal_z = pt.normal_z;
                s->push_back(p);
                ns->push_back(pn);
            }
            for (const auto &pt: model_c->points)
            {
                pcl::PointXYZRGBA p;
                PN pn;
                p.x = pn.x = pt.x;
                p.y = pn.y = pt.y;
                p.z = pn.z = pt.z;
                p.r = 0;
                p.g = 0;
                p.b = 255;
                pn.normal_x = pt.normal_x;
                pn.normal_y = pt.normal_y;
                pn.normal_z = pt.normal_z;
                t->push_back(p);
                nt->push_back(pn);
            }
        }
        // if (fitness > std::pow(0.02,2) ){
        //     std::size_t align_tries(0);
        //     while (align_tries<5)
        //     {
        //         Ticp = alignFrames(front_ptr, current, aligned, Ticp, 0.03);
        //         ++align_tries;
        //     }
        //     //bad alignment throw away this frame, try to align on the next
        //     LOCK guard(mtx_acq);
        //     acq_size = acquisition_q.size();
        //     if (bad_align){
        //         //already failed once, give up and get a new pair
        //         current = front_ptr;
        //         bad_align=false;
        //     }
        //     else
        //         bad_align=true;
        //     continue;
        // }
        ////Merge into model
        // *model_c += ali;
        pcl::VoxelGrid<PNT> vg;
        pcl::StatisticalOutlierRemoval<PNT> sor;
        vg.setInputCloud(model_c);
        vg.setDownsampleAllData(true);
        double leaf;
        config->get("model_ds_leaf", leaf);
        vg.setLeafSize(leaf,leaf,leaf);
        PNTC ds;
        vg.filter(ds);
        sor.setInputCloud(ds.makeShared());
        sor.setMeanK(100);
        sor.setStddevMulThresh(10.0);
        LOCK guard(mtx_model);
        sor.filter(*model_ds);
    }//EndWhile
    ROS_INFO("[Modeler::%s]\tStopped",__func__);
    processing = false;
    first_frame.reset();
}

Eigen::Matrix4f
Modeler::alignFrame(PNTC::Ptr source, PNTC::Ptr &aligned, const Eigen::Matrix4f &guess, const float dist, const bool force_no_normals)
{
    PNTC::Ptr model = boost::make_shared<PNTC>();
    {
        LOCK guard (mtx_model);
        pcl::copyPointCloud(*model_ds, *model);
    }

    cebp->setInputSource(source);
    cebp->setSourceNormals(source);
    cebp->setInputTarget(model);
    cebp->setTargetNormals(model);
    Eigen::Matrix4f ret_trans;
    aligned = boost::make_shared<PNTC>();
    bool use_generalized;
    config->get("use_gicp", use_generalized);
    if (use_generalized){
        gicp.setInputTarget(model);
        gicp.setInputSource(source);
        gicp.setEuclideanFitnessEpsilon(1e-7);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-9);
        gicp.setMaxCorrespondenceDistance(dist);
        gicp.setUseReciprocalCorrespondences(false);
        // gicp.setTransformationEstimation(teDQ);
        // gicp.setCorrespondenceEstimation(cebp);
        gicp.setCorrespondenceRandomness(10);
        gicp.setMaximumOptimizerIterations(50);
        // gicp.setRotationEpsilon(1e-5);
        gicp.align(*aligned, guess); //it does not get transformed due to a bug!?
        pcl::transformPointCloudWithNormals(*source, *aligned, gicp.getFinalTransformation());
        LOCK guard(mtx_viz);
        fitness = gicp.getFitnessScore();
        ret_trans = gicp.getFinalTransformation();
    }
    else{
        icp.setInputTarget(model);
        icp.setInputSource(source);
        icp.setEuclideanFitnessEpsilon(1e-7);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-9);
        icp.setMaxCorrespondenceDistance(dist);
        icp.setUseReciprocalCorrespondences(false);
        icp.setTransformationEstimation(teDQ);
        if (!force_no_normals)
            icp.setCorrespondenceEstimation(cebp);
        icp.align(*aligned, guess);
        pcl::transformPointCloudWithNormals(*source, *aligned, icp.getFinalTransformation());
        LOCK guard(mtx_viz);
        fitness = icp.getFitnessScore();
        ret_trans = icp.getFinalTransformation();
    }
    // cebp->determineCorrespondences(c, dist);
    ///////////////////////////
    return ret_trans;
}

// Eigen::Matrix4f
// Modeler::refineFrames(PTC::Ptr frame, PTC::Ptr &refined, const Eigen::Matrix4f &guess,  const float dist)
// {
//     PTC::Ptr model = boost::make_shared<PTC>();
//     mtx_model.lock();
//     pcl::copyPointCloud(*model_ds, *model);
//     mtx_model.unlock();
//
//     icp.setInputTarget(model);
//     icp.setInputSource(frame);
//     icp.setEuclideanFitnessEpsilon(1e-9);
//     icp.setMaximumIterations(10);
//     icp.setTransformationEpsilon(1e-9);
//     icp.setMaxCorrespondenceDistance(dist);
//     icp.setUseReciprocalCorrespondences(false);
//     pcl::registration::TransformationEstimationDualQuaternion<PT,PT,float>::Ptr te =
//         boost::make_shared<pcl::registration::TransformationEstimationDualQuaternion<PT,PT,float>>();
//     icp.setTransformationEstimation(te);
//     refined = boost::make_shared<PTC>();
//     icp.align(*refined, guess);
//     return icp.getFinalTransformation();
// }

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

void
Modeler::spinVisualizer()
{
    //debug visualization
    a=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    t=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    s=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    while (a->empty() || s->empty() || t->empty())
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    pcl::visualization::PCLVisualizer viz;
    mtx_viz.lock();
    viz.addPointCloud(s, "source");
    viz.addPointCloud(t, "target");
    viz.addPointCloud(a, "aligned");
    viz.addPointCloudNormals<PN>(ns, 10, 0.005, "s_n");
    viz.addPointCloudNormals<PN>(nt, 10, 0.005, "t_n");
    // viz.addCorrespondences<pcl::PointXYZRGBA>(s,t, c, "corrs");
    std::string text = "Fitness: " + std::to_string(std::sqrt(fitness));
    viz.addText(text, 30, 30, 1.0, 0.0, 0.8, "fit");
    mtx_viz.unlock();
    while(viz_spin){
        mtx_viz.lock();
        viz.removeShape("s_n");
        viz.removeShape("t_n");
        viz.updatePointCloud(s, "source");
        viz.updatePointCloud(t, "target");
        viz.updatePointCloud(a, "aligned");
        viz.addPointCloudNormals<PN>(ns, 10, 0.005, "s_n");
        viz.addPointCloudNormals<PN>(nt, 10, 0.005, "t_n");
        // viz.updateCorrespondences<pcl::PointXYZRGBA>(s, t, c, 1, "corrs");
        text = "Fitness: " + std::to_string(std::sqrt(fitness));
        viz.updateText(text, 30,30, "fit");
        mtx_viz.unlock();
        viz.spinOnce(300);
    }
    viz.close();
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
