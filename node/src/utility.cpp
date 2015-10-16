#include <pacman_vision/utility.h>

void fromEigen(Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest)
{
  Eigen::Matrix3f rot;
  rot << source(0,0), source(0,1), source(0,2),
      source(1,0), source(1,1), source(1,2),
      source(2,0), source(2,1), source(2,2);
  Eigen::Quaternionf quad(rot);
  quad.normalize();
  tf::Quaternion q(quad.x(), quad.y(), quad.z(), quad.w());
  tf_dest.setOrigin(tf::Vector3(source(0,3), source(1,3), source(2,3)));
  tf_dest.setRotation(q);
  dest.orientation.x = quad.x();
  dest.orientation.y = quad.y();
  dest.orientation.z = quad.z();
  dest.orientation.w = quad.w();
  dest.position.x = source(0,3);
  dest.position.y = source(1,3);
  dest.position.z = source(2,3);
}

void fromPose(geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest)
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

void fromTF(tf::StampedTransform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest)
{
  Eigen::Quaternionf q(source.getRotation().getW(), source.getRotation().getX(), source.getRotation().getY(), source.getRotation().getZ());
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

void fromTF(tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest)
{
  Eigen::Quaternionf q(source.getRotation().getW(), source.getRotation().getX(), source.getRotation().getY(), source.getRotation().getZ());
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

void crop_a_box(PC::Ptr source, PC::Ptr& dest, const Eigen::Matrix4f& trans, const Box lim, bool remove_inside, bool organized)
{
  if(!source)
    return;
  if(!dest)
    dest.reset(new PC);
  pcl::CropBox<PT> cb;
  cb.setKeepOrganized(organized);
  cb.setInputCloud (source);
  Eigen::Vector4f min,max;
  min << lim.x1, lim.y1, lim.z1, 1;
  max << lim.x2, lim.y2, lim.z2, 1;
  cb.setMin(min);
  cb.setMax(max);
  //Note this transform is applied to the box, not the cloud
  Eigen::Matrix3f Rot = trans.block<3,3>(0,0); //3x3 block starting at 0,0
  Eigen::Vector3f angles = Rot.eulerAngles(0,1,2);
  Eigen::Vector3f translation( trans(0,3), trans(1,3), trans(2,3));
  cb.setTranslation(translation);
  cb.setRotation(angles);
  cb.setNegative(remove_inside);
  cb.filter (*dest);
}
