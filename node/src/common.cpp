#include <pacman_vision/common.h>

void
fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest)
{
    Eigen::Matrix3f rot;
    rot<<source(0,0), source(0,1), source(0,2),
        source(1,0), source(1,1), source(1,2),
        source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    dest.orientation.x = quad.x();
    dest.orientation.y = quad.y();
    dest.orientation.z = quad.z();
    dest.orientation.w = quad.w();
    dest.position.x = source(0,3);
    dest.position.y = source(1,3);
    dest.position.z = source(2,3);
}
void
fromEigen(const Eigen::Matrix4f &source, tf::Transform &dest)
{
    Eigen::Matrix3f rot;
    rot<<source(0,0), source(0,1), source(0,2),
        source(1,0), source(1,1), source(1,2),
        source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    tf::Quaternion q(quad.x(), quad.y(), quad.z(), quad.w());
    dest.setOrigin(tf::Vector3(source(0,3), source(1,3), source(2,3)));
    dest.setRotation(q);
}
void
fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest)
{
    Eigen::Matrix3f rot;
    rot<<source(0,0), source(0,1), source(0,2),
        source(1,0), source(1,1), source(1,2),
        source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    dest.orientation.x = quad.x();
    dest.orientation.y = quad.y();
    dest.orientation.z = quad.z();
    dest.orientation.w = quad.w();
    dest.position.x = source(0,3);
    dest.position.y = source(1,3);
    dest.position.z = source(2,3);
    tf::Quaternion q(quad.x(), quad.y(), quad.z(), quad.w());
    tf_dest.setOrigin(tf::Vector3(source(0,3), source(1,3), source(2,3)));
    tf_dest.setRotation(q);
}

void
fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest)
{
    Eigen::Quaternionf q(source.orientation.w, source.orientation.x, source.orientation.y, source.orientation.z);
    q.normalize();
    Eigen::Vector3f t(source.position.x, source.position.y, source.position.z);
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
    tf::Quaternion qt(q.x(), q.y(), q.z(), q.w());
    tf_dest.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    tf_dest.setRotation(qt);
}
void
fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest)
{
    Eigen::Quaternionf q(source.orientation.w, source.orientation.x, source.orientation.y, source.orientation.z);
    q.normalize();
    Eigen::Vector3f t(source.position.x, source.position.y, source.position.z);
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
}
void
fromPose(const geometry_msgs::Pose &source, tf::Transform &dest)
{
    tf::Quaternion q(source.orientation.x, source.orientation.y, source.orientation.z, source.orientation.w);
    dest.setOrigin(tf::Vector3(source.position.x, source.position.y, source.position.z));
    dest.setRotation(q);
}

void
fromTF(const tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest)
{
    Eigen::Quaternionf q(source.getRotation().getW(),source.getRotation().getX(), source.getRotation().getY(),source.getRotation().getZ());
    q.normalize();
    Eigen::Vector3f t(source.getOrigin().x(), source.getOrigin().y(), source.getOrigin().z());
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
    pose_dest.orientation.x = q.x();
    pose_dest.orientation.y = q.y();
    pose_dest.orientation.z = q.z();
    pose_dest.orientation.w = q.w();
    pose_dest.position.x = t(0);
    pose_dest.position.y = t(1);
    pose_dest.position.z = t(2);
}
void
fromTF(const tf::Transform &source, Eigen::Matrix4f &dest)
{
    Eigen::Quaternionf q(source.getRotation().getW(),source.getRotation().getX(), source.getRotation().getY(),source.getRotation().getZ());
    q.normalize();
    Eigen::Vector3f t(source.getOrigin().x(), source.getOrigin().y(), source.getOrigin().z());
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
}
void
fromTF(const tf::Transform &source, geometry_msgs::Pose &dest)
{
    dest.orientation.x = source.getRotation().getX();
    dest.orientation.y = source.getRotation().getY();
    dest.orientation.z = source.getRotation().getZ();
    dest.orientation.w = source.getRotation().getW();
    dest.position.x = source.getOrigin().x();
    dest.position.y = source.getOrigin().y();
    dest.position.z = source.getOrigin().z();
}

//Box implementations
Box&
Box::operator= (const Box& other)
{
    x1=other.x1;
    x2=other.x2;
    y1=other.y1;
    y2=other.y2;
    z1=other.z1;
    z2=other.z2;
    return *this;
}
Box&
Box::operator= (Box&& other)
{
    x1= std::move(other.x1);
    x2= std::move(other.x2);
    y1= std::move(other.y1);
    y2= std::move(other.y2);
    z1= std::move(other.z1);
    z2= std::move(other.z2);
    return *this;
}
Box
Box::operator* (const float scale) const
{
    double x1s,y1s,z1s;
    double x2s,y2s,z2s;
    x1s = (x2*(1-scale) + x1*(1+scale))*0.5;
    y1s = (y2*(1-scale) + y1*(1+scale))*0.5;
    z1s = (z2*(1-scale) + z1*(1+scale))*0.5;
    x2s = (x2*(1+scale) + x1*(1-scale))*0.5;
    y2s = (y2*(1+scale) + y1*(1-scale))*0.5;
    z2s = (z2*(1+scale) + z1*(1-scale))*0.5;
    return (Box(x1s, y1s, z1s, x2s, y2s, z2s));
}

void
crop_a_box(const PTC::ConstPtr source, PTC::Ptr& dest, const Box lim, const bool remove_inside,
        const Eigen::Matrix4f &trans, const bool keep_organized)
{
    if(!source){
        ROS_WARN("[%s]\tUninitialized source cloud pointer recived, not doing anything...",__func__);
        return;
    }
    if(source->empty()){
        ROS_WARN("[%s]\tEmpty source cloud recived, not doing anything...",__func__);
        return;
    }
    if(!dest)
        dest.reset(new PTC);
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
create_box_marker(const Box lim, visualization_msgs::Marker &marker, bool cube_type)
{
    if(!cube_type){
        // Create lines
        //Does not set time header, ref_frame, namespace and id of marker
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.header.stamp = ros::Time();
        //adjust these two values later if needed
        marker.ns = "box";
        marker.id = 0;
        marker.scale.x = 0.002;
        marker.action = visualization_msgs::Marker::ADD;
        //white color default
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        //pose Identity
        marker.pose.position.x=0.0f;
        marker.pose.position.y=0.0f;
        marker.pose.position.z=0.0f;
        marker.pose.orientation.x=0.0f;
        marker.pose.orientation.y=0.0f;
        marker.pose.orientation.z=0.0f;
        marker.pose.orientation.w=1.0f;
        marker.lifetime = ros::Duration(1);
        geometry_msgs::Point p, pf;
        //0-1
        p.x = lim.x1;
        p.y = lim.y1;
        p.z = lim.z1;
        pf.x = lim.x2;
        pf.y = lim.y1;
        pf.z = lim.z1;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //2-3
        pf.x = lim.x1;
        pf.y = lim.y2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //4-5
        pf.x = lim.x1;
        pf.y = lim.y1;
        pf.z = lim.z2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //6-7
        p.x = lim.x2;
        p.y = lim.y2;
        p.z = lim.z2;
        pf.x = lim.x2;
        pf.y = lim.y2;
        pf.z = lim.z1;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //8-9
        pf.x = lim.x1;
        pf.y = lim.y2;
        pf.z = lim.z2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //10-11
        pf.x = lim.x2;
        pf.y = lim.y1;
        pf.z = lim.z2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //12-13
        p.x = lim.x1;
        p.y = lim.y1;
        p.z = lim.z2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //14-15
        pf.x = lim.x1;
        pf.y = lim.y2;
        pf.z = lim.z2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //16-17
        p.x = lim.x2;
        p.y = lim.y2;
        p.z = lim.z1;
        pf.x = lim.x2;
        pf.y = lim.y1;
        pf.z = lim.z1;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //18-19
        pf.x = lim.x1;
        pf.y = lim.y2;
        pf.z = lim.z1;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //20-21
        p.x = lim.x1;
        p.y = lim.y2;
        p.z = lim.z2;
        marker.points.push_back(p);
        marker.points.push_back(pf);
        //22-23
        p.x = lim.x2;
        p.y = lim.y1;
        p.z = lim.z2;
        pf.x = lim.x2;
        pf.y = lim.y1;
        pf.z = lim.z1;
        marker.points.push_back(p);
        marker.points.push_back(pf);
    }
    else{
        // create a cube
        //Does not set time header, ref_frame, namespace and id of marker
        marker.type = visualization_msgs::Marker::CUBE;
        marker.header.stamp = ros::Time();
        //adjust these two values later if needed
        marker.ns = "box";
        marker.id = 0;
        marker.scale.x = (lim.x2-lim.x1)*0.5;
        marker.scale.y = (lim.y2-lim.y1)*0.5;
        marker.scale.z = (lim.z2-lim.z1)*0.5;
        marker.action = visualization_msgs::Marker::ADD;
        //white color default, transparent
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.3f;
        //pose Identity
        marker.pose.position.x=0.0f;
        marker.pose.position.y=0.0f;
        marker.pose.position.z=0.0f;
        marker.pose.orientation.x=0.0f;
        marker.pose.orientation.y=0.0f;
        marker.pose.orientation.z=0.0f;
        marker.pose.orientation.w=1.0f;
        marker.points.clear();
        geometry_msgs::Point p;
        p.x = (lim.x2+lim.x1)*0.5;
        p.y = (lim.y2+lim.y1)*0.5;
        p.z = (lim.z2+lim.z1)*0.5;
        marker.points.push_back(p);
        marker.lifetime = ros::Duration(1);
    }
}

//copypaste from utils TODO
/*
void
VisionNode::create_arm_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, int i, bool right)
{
    this->broadcaster_module->create_box_marker(marker, lim);
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    geometry_msgs::Pose pose;
    tf::Transform tf;
    fromEigen(t, pose, tf);
    marker.pose = pose;
    if (right)
        marker.ns = "Right Arm Boxes";
    else
        marker.ns = "Left Arm Boxes";
    marker.id = i+1;
}

void
VisionNode::create_hand_box_marker(Eigen::Matrix4f& t, visualization_msgs::Marker &marker, const Box lim, bool right)
{
    this->broadcaster_module->create_box_marker(marker, lim);
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    geometry_msgs::Pose pose;
    tf::Transform tf;
    fromEigen(t, pose, tf);
    marker.pose = pose;
    if (right)
        marker.ns = "Right Hand Box";
    else
        marker.ns = "Left Hand Box";
    marker.id = 0;
}

void
VisionNode::crop_arm(PC::Ptr source, PC::Ptr& dest, bool right)
{
    if(!source)
        return;
    if(!dest)
        dest.reset(new PC);
    if (right){
        if (!this->storage->read_right_arm(right_arm)){
            right_arm.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);
            right_arm->resize(6);
            for (auto& x: *right_arm)
                x.setIdentity();
        }
        for(int i=0; i<right_arm->size(); ++i)
        {
            crop_a_box(source, dest, right_arm->at(i), lwr_arm[i]*box_scale, true, false);
            pcl::copyPointCloud(*dest, *source);
        }
    }
    else{
        if (!this->storage->read_left_arm(left_arm)){
            left_arm.reset(new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);
            left_arm->resize(6);
            for (auto& x: *left_arm)
                x.setIdentity();
        }
        for(int i=0; i<left_arm->size(); ++i)
        {
            crop_a_box(source, dest, left_arm->at(i), lwr_arm[i]*box_scale, true, false);
            pcl::copyPointCloud(*dest, *source);
        }
    }
}

void
VisionNode::crop_hand(PC::Ptr source, PC::Ptr& dest, bool right)
{
    if(!source)
        return;
    if(!dest)
        dest.reset(new PC);
    if (right){
        this->storage->read_right_hand(right_hand);
        crop_a_box(source, dest, *right_hand, hand_right*box_scale, true, false);
    }
    else{
        this->storage->read_left_hand(left_hand);
        crop_a_box(source, dest, *left_hand, hand_left*box_scale, true, false);
    }
}

*/
