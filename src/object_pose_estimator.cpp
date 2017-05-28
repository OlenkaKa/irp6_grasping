//
// Created by ola on 27.05.17.
//

#include <irp6_grasping/object_pose_estimator.h>

using namespace irp6_grasping;

ObjectPoseEstimator::SingleViewPoseEstimator::SingleViewPoseEstimator(const ros::Time &time,
                                                                      const geometry_msgs::Pose &pose,
                                                                      double confidence)
  : last_update_(time), pose_(pose), confidence_(confidence)
{
  kalman_ = new PoseKalmanFilter();
  kalman_->initKalmanFilter(0.033);  // TODO
  kalman_->initMeasurements(measurements_);
}

ObjectPoseEstimator::SingleViewPoseEstimator::~SingleViewPoseEstimator()
{
  delete kalman_;
}

void ObjectPoseEstimator::SingleViewPoseEstimator::estimatePose(const ros::Time &time, const geometry_msgs::Pose &pose,
                                                                double confidence)
{
  geometry_msgs::Pose estimated_pose(pose);
  if (true)  // TODO use_kalman_filter
  {
    kalman_->fillMeasurements(pose, measurements_);
    kalman_->updateKalmanFilter(measurements_, estimated_pose);
  }
  this->pose_ = estimated_pose;

  double estimation_confidence = this->confidence_ / 2 + confidence;
  this->confidence_ = estimation_confidence > 1 ? 1 : estimation_confidence;

  this->last_update_ = time;
}

ObjectPoseEstimator::ObjectPoseEstimator(double min_confidence, bool use_kalman_filter)
  : min_confidence_(min_confidence), use_kalman_filter_(use_kalman_filter)
{
}

ObjectPoseEstimator::~ObjectPoseEstimator()
{
  for (auto it = view_estimators_.begin(), end_it = view_estimators_.end(); it != end_it; it++)
  {
    delete it->second;
  }
}

ros::Time ObjectPoseEstimator::getLastPoseEstimationTime() const
{
  ros::Time last_update(ros::TIME_MIN);
  for (auto it = view_estimators_.begin(), end_it = view_estimators_.end(); it != end_it; it++)
  {
    if (it->second->last_update_ > last_update)
    {
      last_update = it->second->last_update_;
    }
  }
  return last_update;
}

bool ObjectPoseEstimator::canEstimatePose() const
{
  return !view_estimators_.empty();
}

void ObjectPoseEstimator::estimatePose(int view_id, ros::Time time_stamp, geometry_msgs::Pose pose, double confidence,
                                       PoseData &pose_data)
{
  if (view_estimators_.find(view_id) == view_estimators_.end())
  {
    view_estimators_.insert(
        std::pair<int, SingleViewPoseEstimator *>(view_id, new SingleViewPoseEstimator(time_stamp, pose, confidence)));
  }
  else if (confidence > min_confidence_)
  {
    SingleViewPoseEstimator *view_estimator = view_estimators_.at(view_id);
    view_estimator->estimatePose(time_stamp, pose, confidence);
    view_estimator->kalman_->getCurrentPoseData(pose_data);
  }
}

void ObjectPoseEstimator::addData(int view_id, object_recognition_msgs::RecognizedObject &object) const
{
  SingleViewPoseEstimator *view_estimator = view_estimators_.at(view_id);
  object.header.stamp = view_estimator->last_update_;
  object.pose.header.stamp = view_estimator->last_update_;
  object.pose.pose.pose = view_estimator->pose_;
}
