/*!
 * \file PoseKalmanFilter.cpp
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#include <ros/ros.h>

#include <irp6_grasping/pose_kalman_filter.h>
#include <irp6_grasping/transform_util.h>

using namespace irp6_grasping;
using namespace cv;

PoseData::PoseData() = default;

PoseData::PoseData(const geometry_msgs::Pose &pose)
{
  position = pose.position;
  orientation = quat2rot(pose.orientation);
}

geometry_msgs::Pose PoseData::getPose() const
{
  geometry_msgs::Pose result;
  result.position = position;
  result.orientation = rot2quat(orientation);
  return result;
}

const int nStates = 12;       // the number of states
const int nMeasurements = 6;  // the number of measured states
const int nInputs = 0;        // the number of action control

void PoseKalmanFilter::initKalmanFilter(double dt)
{
  kf_.init(nStates, nMeasurements, nInputs, CV_64F);                // init Kalman Filter
  cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-5));      // set process noise
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-4));  // set measurement noise
  cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));            // error covariance

  /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 0 0 0  0  0  0]
  //  [0 1 0  0 dt  0 0 0 0  0  0  0]
  //  [0 0 1  0  0 dt 0 0 0  0  0  0]
  //  [0 0 0  1  0  0 0 0 0  0  0  0]
  //  [0 0 0  0  1  0 0 0 0  0  0  0]
  //  [0 0 0  0  0  1 0 0 0  0  0  0]
  //  [0 0 0  0  0  0 1 0 0 dt  0  0]
  //  [0 0 0  0  0  0 0 1 0  0 dt  0]
  //  [0 0 0  0  0  0 0 0 1  0  0 dt]
  //  [0 0 0  0  0  0 0 0 0  1  0  0]
  //  [0 0 0  0  0  0 0 0 0  0  1  0]
  //  [0 0 0  0  0  0 0 0 0  0  0  1]

  // position
  kf_.transitionMatrix.at<double>(0, 3) = dt;
  kf_.transitionMatrix.at<double>(1, 4) = dt;
  kf_.transitionMatrix.at<double>(2, 5) = dt;

  // orientation
  kf_.transitionMatrix.at<double>(6, 9) = dt;
  kf_.transitionMatrix.at<double>(7, 10) = dt;
  kf_.transitionMatrix.at<double>(8, 11) = dt;

  /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 1 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 1 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 1 0 0 0]

  kf_.measurementMatrix.at<double>(0, 0) = 1;   // x
  kf_.measurementMatrix.at<double>(1, 1) = 1;   // y
  kf_.measurementMatrix.at<double>(2, 2) = 1;   // z
  kf_.measurementMatrix.at<double>(3, 6) = 1;   // roll
  kf_.measurementMatrix.at<double>(4, 7) = 1;   // pitch
  kf_.measurementMatrix.at<double>(5, 8) = 1;   // yaw
}

void PoseKalmanFilter::initMeasurements(cv::Mat &measurements)
{
  measurements = Mat(nMeasurements, 1, CV_64F);
  measurements.setTo(Scalar(0));
}

void PoseKalmanFilter::fillMeasurements(const geometry_msgs::Pose &measured_pose, cv::Mat &measurements)
{
  // Set measurement to predict
  measurements.at<double>(0) = measured_pose.position.x;  // x
  measurements.at<double>(1) = measured_pose.position.y;  // y
  measurements.at<double>(2) = measured_pose.position.z;  // z

  geometry_msgs::Vector3 rpy = quat2rot(measured_pose.orientation);
  measurements.at<double>(3) = rpy.x;  // roll
  measurements.at<double>(4) = rpy.y;  // pitch
  measurements.at<double>(5) = rpy.z;  // yaw

//  ROS_INFO("Measured position: %f %f %f", measured_pose.position.x, measured_pose.position.y, measured_pose.position.z);
//  ROS_INFO("Measured roll-pitch-yaw: %f %f %f", rpy.x, rpy.y, rpy.z);
}

void PoseKalmanFilter::updateKalmanFilter(const cv::Mat &measurement, PoseData &pose_data)
{
  // First predict, to update the internal statePre variable
  kf_.predict();

  // The "correct" phase that is going to use the predicted value and our measurement
  Mat estimated = kf_.correct(measurement);

  // Estimated translation
  pose_data.position.x = estimated.at<double>(0);
  pose_data.position.y = estimated.at<double>(1);
  pose_data.position.z = estimated.at<double>(2);

  // Estimated euler angles
  pose_data.orientation.x = estimated.at<double>(6);
  pose_data.orientation.y = estimated.at<double>(7);
  pose_data.orientation.z = estimated.at<double>(8);

  // Estimated translation velocity
  pose_data.velocity.linear.x = estimated.at<double>(3);
  pose_data.velocity.linear.y = estimated.at<double>(4);
  pose_data.velocity.linear.z = estimated.at<double>(5);

  // Estimated euler angles velocity
  pose_data.velocity.angular.x = estimated.at<double>(9);
  pose_data.velocity.angular.y = estimated.at<double>(10);
  pose_data.velocity.angular.z = estimated.at<double>(11);

  ROS_INFO("Estimated position: %f %f %f", pose_data.position.x, pose_data.position.y, pose_data.position.z);
  ROS_INFO("Estimated roll-pitch-yaw: %f %f %f", pose_data.orientation.x, pose_data.orientation.y, pose_data.orientation.z);
}
