/*!
 * \file PoseKalmanFilter.h
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#ifndef IRP6_GRASPING_POSE_KALMAN_FILTER_H
#define IRP6_GRASPING_POSE_KALMAN_FILTER_H

// Header containing Kalman filter.
#include <opencv2/video/tracking.hpp>

//#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

namespace irp6_grasping
{
// TODO move to another file
class PoseData
{
public:
  geometry_msgs::Pose getPose() const;
  PoseData();
  explicit PoseData(const geometry_msgs::Pose &pose_data);

  geometry_msgs::Point position;       // x, y, z
  geometry_msgs::Vector3 orientation;  // roll, pitch, yaw
  geometry_msgs::Twist velocity;
//  geometry_msgs::Accel acceleration;
};

class PoseKalmanFilter
{
public:
  /// Initialize Kalman Filter
  void initKalmanFilter(double dt);

  /// Initialize measurements - size, type
  static void initMeasurements(cv::Mat &measurements);

  /// Fill measurements based on mesured pose
  static void fillMeasurements(const geometry_msgs::Pose &measured_pose, cv::Mat &measurements);

  /// Update kalman filter and calculate estimated position
  void updateKalmanFilter(const cv::Mat &measurement, PoseData &pose_data);

private:
  cv::KalmanFilter kf_;
};

}  // irp6_grasping

#endif  // IRP6_GRASPING_POSE_KALMAN_FILTER_H
