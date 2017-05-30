//
// Created by ola on 27.05.17.
//

#ifndef IRP6_GRASPING_POSE_ESTIMATOR_H
#define IRP6_GRASPING_POSE_ESTIMATOR_H

#include <geometry_msgs/Pose.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include "pose_kalman_filter.h"

namespace irp6_grasping
{
class ObjectPoseEstimator
{
public:
  ObjectPoseEstimator(double min_confidence);
  ~ObjectPoseEstimator();
  ros::Time getLastPoseEstimationTime() const;
  bool canEstimatePose() const;
  bool canEstimatePose(int view_id) const;
  void estimatePose(int view_id, ros::Time time_stamp, geometry_msgs::Pose pose, double confidence,
                    PoseData &pose_data);

  void addData(int view_id, object_recognition_msgs::RecognizedObject &object) const;

private:
  struct SingleViewPoseEstimator
  {
    SingleViewPoseEstimator(const ros::Time &time, const geometry_msgs::Pose &pose, double confidence);
    ~SingleViewPoseEstimator();
    void estimatePose(const ros::Time &time, const geometry_msgs::Pose &pose, double confidence);

    ros::Time last_update_;
    double confidence_;
    geometry_msgs::Pose pose_;
    PoseKalmanFilter *kalman_;
    cv::Mat measurements_;
  };

  std::map<int, SingleViewPoseEstimator *> view_estimators_;
  double min_confidence_;
};
}  // irp6_grasping

#endif  // IRP6_GRASPING_POSE_ESTIMATOR_H
