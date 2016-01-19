#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <pacv_config.h>
#include <common/common_std.h>
#include <common/common_pcl.h>
#include <common/box.h>

#include <Eigen/StdVector>
// #include <pcl/segmentation/supervoxel_clustering.h>

namespace pacv
{
class Storage
{
    public:
        typedef std::shared_ptr<Storage> Ptr;
        Storage();
        ~Storage(){}
        //Eigen Alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Read and write scene methods
        bool readScene (PTC::Ptr &cloud);
        bool writeScene (PTC::Ptr cloud);
        bool readSceneProcessed (PTC::Ptr &cloud);
        bool writeSceneProcessed (PTC::Ptr cloud);
        bool readScene (PXC::Ptr &cloud);
        bool readSceneProcessed (PXC::Ptr &cloud);
        //Read write sensor reference frame
        bool readSensorFrame (std::string& frame);
        bool writeSensorFrame (std::string frame);
        //Read and write estimated objects
        bool readObjTransforms (std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans);
        bool writeObjTransforms (std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> trans);
        bool readObjNames (std::shared_ptr<std::vector<std::pair<std::string, std::string>>> &n);
        bool writeObjNames (std::shared_ptr<std::vector<std::pair<std::string, std::string>>> n);
        //Search for a specific object name and return its index
        bool searchObjName (std::string n, int &idx);
        //Read and object transform by its index
        bool readObjTransformByIndex (int idx, std::shared_ptr<Eigen::Matrix4f> &trans);
        //Write and object transform to the specified index
        bool writeObjTransformByIndex (int idx, std::shared_ptr<Eigen::Matrix4f> trans);
        //Read and write arms/hands transforms
        bool readLeftArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool writeLeftArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> arm);
        bool readRightArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool writeRightArm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> arm);
        bool readLeftHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &hand);
        bool writeLeftHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> hand);
        bool readRightHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &hand);
        bool writeRightHand(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> hand);
        //Read/write index of tracked object
        void readTrackedIndex(int &idx);
        void writeTrackedIndex(int idx);
    private:
        //untouched scene from kinect
        PTC::Ptr scene;
        std::mutex mtx_scene;
        //scene after processing
        PTC::Ptr scene_processed;
        std::mutex mtx_scene_processed;
        //sensor reference frame
        std::string sensor_ref_frame;
        std::mutex mtx_sensor_ref_frame;
        //Estimated transform from estimator
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> estimations;
        std::mutex mtx_estimations;
        //naming and id-ing of estimated objects from estimator
        std::vector<std::pair<std::string, std::string>> names; //name,ID
        std::mutex mtx_names;
        /*
         * Tracked object index (referred to vector of estimations),
         * set to -1 if not tracking
         */
        int index;
        std::mutex mtx_index;
        //Vito Left arm transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> left_arm;
        std::mutex mtx_left_arm;
        //Vito Right arm transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> right_arm;
        std::mutex mtx_right_arm;
        //Vito Left hand transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> left_hand;
        std::mutex mtx_left_hand;
        //Vito Right hand transform
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> right_hand;
        std::mutex mtx_right_hand;
};
}
#endif
