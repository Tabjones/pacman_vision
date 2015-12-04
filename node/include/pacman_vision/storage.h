#ifndef _INCL_STORAGE_H_
#define _INCL_STORAGE_H_

#include <pacman_vision/config.h>
//Utility
#include <pacman_vision/common.h>
#include <pcl/segmentation/supervoxel_clustering.h>

class Storage
{
    public:
        Storage();
        ~Storage(){}
        //Eigen Alignment
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //Read and write scene methods
        bool
        read_scene (PTC::Ptr &cloud);
        bool
        write_scene (PTC::Ptr &cloud);
        bool
        read_scene_processed (PTC::Ptr &cloud);
        bool
        write_scene_processed (PTC::Ptr &cloud);
        bool
        read_scene (PXC::Ptr &cloud);
        bool
        read_scene_processed (PXC::Ptr &cloud);
        //Read write sensor reference frame
        bool
        read_sensor_ref_frame (std::string& frame);
        bool
        write_sensor_ref_frame (std::string& frame);
        //Read and write estimated objects
        bool
        read_obj_clusters (std::shared_ptr<std::vector<PXC> > &objs);
        bool
        write_obj_clusters (std::shared_ptr<std::vector<PXC> > &objs);
        bool
        read_obj_transforms (std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans);
        bool
        write_obj_transforms (std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &trans);
        bool
        read_obj_names (std::shared_ptr<std::vector<std::pair<std::string, std::string>>> &n);
        bool
        write_obj_names (std::shared_ptr<std::vector<std::pair<std::string, std::string>>> &n);
        //Search for a specific object name and return its index
        bool
        search_obj_name (std::string n, int &idx);
        //Read and object transform by its index
        bool
        read_obj_transform_by_index (int idx, std::shared_ptr<Eigen::Matrix4f> &trans);
        //Write and object transform to the specified index
        bool
        write_obj_transform_by_index (int idx, std::shared_ptr<Eigen::Matrix4f> &trans);
        //Read and write arms/hands/table transforms
        bool
        read_left_arm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool
        write_left_arm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool
        read_right_arm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        bool
        write_right_arm(std::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> &arm);
        void
        read_left_hand(std::shared_ptr<Eigen::Matrix4f> &hand);
        bool
        write_left_hand(std::shared_ptr<Eigen::Matrix4f> &hand);
        void
        read_right_hand(std::shared_ptr<Eigen::Matrix4f> &hand);
        bool
        write_right_hand(std::shared_ptr<Eigen::Matrix4f> &hand);
        void
        read_table(std::shared_ptr<Eigen::Matrix4f> &t);
        bool
        write_table(std::shared_ptr<Eigen::Matrix4f> &t);
        //Read/write index of tracked object
        void
        read_tracked_index(int &idx);
        void
        write_tracked_index(int idx);
        //Read/Write tracked object boundingbox
        void
        read_tracked_box(std::shared_ptr<Box> &b);
        bool
        write_tracked_box(std::shared_ptr<Box> &b);
        bool
        read_supervoxels_clusters(std::shared_ptr<std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>>  &clus);
        bool
        write_supervoxels_clusters(std::shared_ptr<std::map<uint32_t, pcl::Supervoxel<PT>::Ptr>>  &clus);
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
        /*
         * cluster of objects found on scene
         * TODO currently unused, candidate for removal
         */
        std::vector<PXC> clusters;
        std::mutex mtx_clusters;
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
        //Tracked object boundingbox
        Box bbox;
        std::mutex mtx_bbox;
        //Vito Left arm transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> left_arm;
        std::mutex mtx_left_arm;
        //Vito Right arm transforms
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> right_arm;
        std::mutex mtx_right_arm;
        //Vito Left hand transform
        Eigen::Matrix4f left_hand;
        std::mutex mtx_left_hand;
        //Vito Right hand transform
        Eigen::Matrix4f right_hand;
        std::mutex mtx_right_hand;
        //Vito Table transformation
        Eigen::Matrix4f table;
        std::mutex mtx_table;
        //Supervoxel clusters
        std::map<uint32_t, pcl::Supervoxel<PT>::Ptr> super_clusters;
        std::mutex mtx_super_clusters;
};
#endif
