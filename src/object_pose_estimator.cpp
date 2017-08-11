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
  : last_update_(time), pose_data_(pose), confidence_(confidence)
{
  kalman_ = new PoseKalmanFilter();
  kalman_->initKalmanFilter(1.0);
  kalman_->initMeasurements(measurements_);
}

ObjectPoseEstimator::SingleViewPoseEstimator::~SingleViewPoseEstimator()
{
  delete kalman_;
}

void ObjectPoseEstimator::SingleViewPoseEstimator::estimatePose(const ros::Time &last_update, const geometry_msgs::Pose &pose,
                                                                double confidence, const string &object_pose_estimation_strategy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (object_pose_estimation_strategy == "kalman")
  {
    kalman_->fillMeasurements(pose, measurements_);
    kalman_->updateKalmanFilter(measurements_, pose_data_);

    double estimation_confidence = (confidence_ + confidence) / 2;
    confidence_ = estimation_confidence > 1.0 ? 1.0 : estimation_confidence;
  }
  else if (object_pose_estimation_strategy == "highest_confidence")
  {
    if (confidence > confidence_)
    {
      pose_data_ = PoseData(pose);
      confidence_ = confidence;
    }
  }
  last_update_ = last_update;
}

void ObjectPoseEstimator::SingleViewPoseEstimator::getCurrentPoseData(PoseData &pose_data,
                                                                      const std::string &object_pose_estimation_strategy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_data = pose_data_;
}

// ObjectPoseEstimator

ObjectPoseEstimator::ObjectPoseEstimator(const string &object_pose_estimation_strategy, double min_confidence)
  : object_pose_estimation_strategy_(object_pose_estimation_strategy), min_confidence_(min_confidence)
{
  if (object_pose_estimation_strategy != "kalman" && object_pose_estimation_strategy != "highest_confidence")
    throw invalid_argument("Invalid object pose estimation strategy: " + object_pose_estimation_strategy);
}

ObjectPoseEstimator::~ObjectPoseEstimator()
{
  for (auto &view_estimator : view_estimators_)
    delete view_estimator.second;
}

ros::Time ObjectPoseEstimator::getLastPoseEstimationTime() const
{
  ros::Time last_update(ros::TIME_MIN);
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &view_estimator : view_estimators_)
  {
    if (view_estimator.second->last_update_ > last_update)
      last_update = view_estimator.second->last_update_;
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
  return view_estimators_.find(view_id) != view_estimators_.end() && view_estimators_.at(view_id) != nullptr;
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

  SingleViewPoseEstimator *view_estimator = view_estimators_.at(view_id);
  if (confidence >= /*view_estimator->confidence_*/min_confidence_)
    view_estimator->estimatePose(time_stamp, pose, confidence, object_pose_estimation_strategy_);

  view_estimator->getCurrentPoseData(pose_data, object_pose_estimation_strategy_);
  ROS_INFO("view: %d\t\treceived confidence %f\t\tresult confidence: %f", view_id, confidence, view_estimator->confidence_);
}

void ObjectPoseEstimator::addData(int view_id, object_recognition_msgs::RecognizedObject &object) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (view_estimators_.find(view_id) != view_estimators_.end())
  {
    SingleViewPoseEstimator *view_estimator = view_estimators_.at(view_id);
    object.header.stamp = view_estimator->last_update_;
    object.pose.header.stamp = view_estimator->last_update_;
    object.pose.pose.pose = view_estimator->pose_data_.getPose();
  }
}

void ObjectPoseEstimator::addData(object_recognition_msgs::RecognizedObject &object) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const pair<int, ObjectPoseEstimator::SingleViewPoseEstimator*> &view_estimator_pair = findBestEstimationView();
  ROS_INFO("Best estimation view_id: %d, confidence: %f", view_estimator_pair.first, view_estimator_pair.second->confidence_);
  object.header.stamp = view_estimator_pair.second->last_update_;
  object.confidence = (float) view_estimator_pair.second->confidence_;
  object.pose.header.stamp = view_estimator_pair.second->last_update_;
  object.pose.pose.pose = view_estimator_pair.second->pose_data_.getPose();
}

void ObjectPoseEstimator::clear()
{
  view_estimators_.clear();
}

void ObjectPoseEstimator::clearView(int view_id)
{
  view_estimators_.erase(view_id);
}

pair<int, ObjectPoseEstimator::SingleViewPoseEstimator*> ObjectPoseEstimator::findBestEstimationView() const {
  if (view_estimators_.size() == 1)
    return *view_estimators_.begin();

  return *std::max_element(++view_estimators_.begin(), view_estimators_.end(),
                  [](const pair<int, SingleViewPoseEstimator *> &p1,
                     const pair<int, SingleViewPoseEstimator *> &p2)
                  {
                      return p1.second->confidence_ < p2.second->confidence_;
                  });
}
