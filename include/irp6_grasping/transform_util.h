/*!
 * \file transform_util.h
 * \author Aleksandra Karbarczyk
 */

#ifndef IRP6_GRASPING_TRANSFORM_UTIL_H
#define IRP6_GRASPING_TRANSFORM_UTIL_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_datatypes.h>

namespace irp6_grasping
{
  /// Function converts quaternion to roll-pitch-yaw vector.
  geometry_msgs::Vector3 quat2rot(const geometry_msgs::Quaternion &quaternion);

  /// Function converts roll-pitch-yaw vector to quaternion.
  geometry_msgs::Quaternion rot2quat(const geometry_msgs::Vector3 &rpy);

  /// Function transforms pose from one to another coordinate frame.
  geometry_msgs::Pose transformPose(const geometry_msgs::Pose &start_pose, const tf::StampedTransform &end_tf);

  /// Function transforms point from one to another coordinate frame.
  geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::Transform &tf);
}

#endif //IRP6_GRASPING_TRANSFORM_UTIL_H
