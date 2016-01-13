#include <common/common_pcl.h>
#include <pcl/console/print.h>

namespace pacv
{
void
crop_a_box(const PTC::ConstPtr source, PTC::Ptr& dest, const Box lim, const bool remove_inside,
        const Eigen::Matrix4f &trans, const bool keep_organized)
{
    if(!source){
        pcl::console::print_warn("[%s]\tUninitialized source cloud pointer recived, not doing anything...",__func__);
        return;
    }
    if(source->empty()){
        pcl::console::print_warn("[%s]\tEmpty source cloud recived, not doing anything...",__func__);
        return;
    }
    if(!dest)
        dest=boost::make_shared<PTC>();
    pcl::CropBox<PT> cb;
    cb.setKeepOrganized(keep_organized);
    cb.setInputCloud(source);
    Eigen::Vector4f min,max;
    min << lim.x1, lim.y1, lim.z1, 1;
    max << lim.x2, lim.y2, lim.z2, 1;
    cb.setMin(min);
    cb.setMax(max);
    //Note this transform is applied to the box, not the cloud
    if(!trans.isIdentity()){
        Eigen::Matrix3f Rot = trans.block<3,3>(0,0); //3x3 block starting at 0,0
        Eigen::Vector3f angles = Rot.eulerAngles(0,1,2);
        Eigen::Vector3f translation( trans(0,3), trans(1,3), trans(2,3));
        cb.setTranslation(translation);
        cb.setRotation(angles);
    }
    cb.setNegative(remove_inside);
    cb.filter (*dest);
}
void
crop_a_box(const PXC::ConstPtr source, PXC::Ptr& dest, const Box lim, const bool remove_inside,
        const Eigen::Matrix4f &trans, const bool keep_organized)
{
    if(!source){
        pcl::console::print_warn("[%s]\tUninitialized source cloud pointer recived, not doing anything...",__func__);
        return;
    }
    if(source->empty()){
        pcl::console::print_warn("[%s]\tEmpty source cloud recived, not doing anything...",__func__);
        return;
    }
    if(!dest)
        dest=boost::make_shared<PXC>();
    pcl::CropBox<PX> cb;
    cb.setKeepOrganized(keep_organized);
    cb.setInputCloud(source);
    Eigen::Vector4f min,max;
    min << lim.x1, lim.y1, lim.z1, 1;
    max << lim.x2, lim.y2, lim.z2, 1;
    cb.setMin(min);
    cb.setMax(max);
    //Note this transform is applied to the box, not the cloud
    if(!trans.isIdentity()){
        Eigen::Matrix3f Rot = trans.block<3,3>(0,0); //3x3 block starting at 0,0
        Eigen::Vector3f angles = Rot.eulerAngles(0,1,2);
        Eigen::Vector3f translation( trans(0,3), trans(1,3), trans(2,3));
        cb.setTranslation(translation);
        cb.setRotation(angles);
    }
    cb.setNegative(remove_inside);
    cb.filter (*dest);
}
}
