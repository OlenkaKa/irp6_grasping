//
// Created by ola on 27.05.17.
//

#include <algorithm>
#include <irp6_grasping/object_pose_estimator.h>
#include <ros/ros.h>

using namespace irp6_grasping;
using namespace std;

// SingleViewPoseEstimator

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

void ObjectPoseEstimator::SingleViewPoseEstimator::estimatePose(const ros::Time &last_update, const geometry_msgs::Pose &pose,
                                                                double confidence)
{
  geometry_msgs::Pose estimated_pose;

  kalman_->fillMeasurements(pose, measurements_);
  kalman_->updateKalmanFilter(measurements_, estimated_pose);

  pose_ = estimated_pose;

  double estimation_confidence = (confidence_ + confidence) / 2;
  confidence_ = estimation_confidence > 1.0 ? 1.0 : estimation_confidence;

  last_update_ = last_update;
}

// ObjectPoseEstimator

ObjectPoseEstimator::ObjectPoseEstimator(double min_confidence)
  : min_confidence_(min_confidence)
{
}

ObjectPoseEstimator::~ObjectPoseEstimator()
{
  for (auto it = view_estimators_.begin(), end_it = view_estimators_.end(); it != end_it; it++)
    delete it->second;
}

ros::Time ObjectPoseEstimator::getLastPoseEstimationTime() const
{
  ros::Time last_update(ros::TIME_MIN);
  std::lock_guard<std::mutex> lock(mutex_);
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
  std::lock_guard<std::mutex> lock(mutex_);
  return !view_estimators_.empty();
}

bool ObjectPoseEstimator::canEstimatePose(int view_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return view_estimators_.find(view_id) != view_estimators_.end() && view_estimators_.at(view_id) != NULL;
}

void ObjectPoseEstimator::estimatePose(int view_id, ros::Time time_stamp, geometry_msgs::Pose pose, double confidence,
                                       PoseData &pose_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
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

    ROS_INFO("view: %d\t\treceived confidence %f\t\tresult confidence: %f", view_id, confidence, view_estimator->confidence_);
  }
}

void ObjectPoseEstimator::addData(int view_id, object_recognition_msgs::RecognizedObject &object) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (view_estimators_.find(view_id) != view_estimators_.end())
  {
    SingleViewPoseEstimator *view_estimator = view_estimators_.at(view_id);
    object.header.stamp = view_estimator->last_update_;
    object.pose.header.stamp = view_estimator->last_update_;
    object.pose.pose.pose = view_estimator->pose_;
  }
}

void ObjectPoseEstimator::addData(object_recognition_msgs::RecognizedObject &object) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  int view_id;
  const SingleViewPoseEstimator &view_estimator = findBestEstimationView(view_id);
  ROS_INFO("Best estimation view_id: %d, confidence: %f", view_id, view_estimator.confidence_);
  object.header.stamp = view_estimator.last_update_;
  object.pose.header.stamp = view_estimator.last_update_;
  object.pose.pose.pose = view_estimator.pose_;
}

void ObjectPoseEstimator::clear()
{
  view_estimators_.clear();
}

void ObjectPoseEstimator::clearView(int view_id)
{
  view_estimators_.erase(view_id);
}

const ObjectPoseEstimator::SingleViewPoseEstimator& ObjectPoseEstimator::findBestEstimationView() const {
  return *std::max_element(view_estimators_.begin(), view_estimators_.end(),
                  [](const pair<int, SingleViewPoseEstimator *> &p1,
                     const pair<int, SingleViewPoseEstimator *> &p2)
                  {
                      return p1.second->confidence_ < p2.second->confidence_;
                  })->second;
}

// TODO remove
const ObjectPoseEstimator::SingleViewPoseEstimator& ObjectPoseEstimator::findBestEstimationView(int &view_id) const {
  auto res = std::max_element(view_estimators_.begin(), view_estimators_.end(),
                           [](const pair<int, SingleViewPoseEstimator *> &p1,
                              const pair<int, SingleViewPoseEstimator *> &p2)
                           {
                               return p1.second->confidence_ < p2.second->confidence_;
                           });
  view_id = 2;
  return *(view_estimators_.at(view_estimators_.find(2) == view_estimators_.end() ? 1 : 2));
}
