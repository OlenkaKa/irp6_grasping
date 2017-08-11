//
// Created by ola on 27.05.17.
//

#ifndef IRP6_GRASPING_POSE_ESTIMATOR_H
#define IRP6_GRASPING_POSE_ESTIMATOR_H

#include <mutex>
#include <geometry_msgs/Pose.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include "pose_kalman_filter.h"

namespace irp6_grasping
{
class ObjectPoseEstimator
{
public:
  ObjectPoseEstimator(const std::string &object_pose_estimation_strategy, double min_confidence);
  ~ObjectPoseEstimator();

  ros::Time getLastPoseEstimationTime() const;

  bool canEstimatePose() const;
  bool canEstimatePose(int view_id) const;

  void estimatePose(int view_id, ros::Time time_stamp, geometry_msgs::Pose pose, double confidence,
                    PoseData &pose_data);

  void addData(int view_id, object_recognition_msgs::RecognizedObject &object) const;
  void addData(object_recognition_msgs::RecognizedObject &object) const;

  void clear();
  void clearView(int view_id);

private:
  struct SingleViewPoseEstimator
  {
    SingleViewPoseEstimator(const ros::Time &time, const geometry_msgs::Pose &pose, double confidence);
    ~SingleViewPoseEstimator();
    void estimatePose(const ros::Time &time, const geometry_msgs::Pose &pose, double confidence, const std::string &object_pose_estimation_strategy);
    void getCurrentPoseData(PoseData &pose_data, const std::string &object_pose_estimation_strategy);

    ros::Time last_update_;
    double confidence_;
    PoseData pose_data_;
    PoseKalmanFilter *kalman_;
    cv::Mat measurements_;
    mutable std::mutex mutex_;
  };

  std::pair<int, SingleViewPoseEstimator*> findBestEstimationView() const;

  std::map<int, SingleViewPoseEstimator *> view_estimators_;
  std::string object_pose_estimation_strategy_;
  double min_confidence_;
  mutable std::mutex mutex_;
};
}  // irp6_grasping

#endif  // IRP6_GRASPING_POSE_ESTIMATOR_H
