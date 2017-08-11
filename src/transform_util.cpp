/*!
 * \file transform_util.cpp
 * \author Aleksandra Karbarczyk
 */

#include <irp6_grasping/transform_util.h>

namespace irp6_grasping
{
geometry_msgs::Vector3 quat2rot(const geometry_msgs::Quaternion &quaternion)
{
  tf::Quaternion tf_quaternion;
  tf::quaternionMsgToTF(quaternion, tf_quaternion);

  // the tf::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  return rpy;
}

geometry_msgs::Quaternion rot2quat(const geometry_msgs::Vector3 &rpy)
{
  // translate roll, pitch and yaw into a Quaternion
  tf::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);
  geometry_msgs::Quaternion quaternion;
  tf::quaternionTFToMsg(tf_quaternion, quaternion);
  return quaternion;
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose &start_pose, const tf::StampedTransform &end_tf)
{
    tf::Transform start_tf;
    tf::poseMsgToTF(start_pose, start_tf);
    tf::Transform start_end_tf = end_tf * start_tf;

    geometry_msgs::Pose transformed_pose;
    tf::poseTFToMsg(start_end_tf, transformed_pose);
    return transformed_pose;
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::Transform &tf)
{
    tf::Point initial_tf_point(point.x, point.y, point.z);
    tf::Point result_tf_point = tf * initial_tf_point;

    geometry_msgs::Point result_point;
    result_point.x = result_tf_point.getX();
    result_point.y = result_tf_point.getY();
    result_point.z = result_tf_point.getZ();
    return result_point;
}
}   // irp6_grasping
