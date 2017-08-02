/*!
 * \file PoseKalmanFilter.cpp
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#include "irp6_grasping/pose_kalman_filter.h"

#include <ros/ros.h>
#include "tf/transform_datatypes.h"

using namespace irp6_grasping;
using namespace cv;

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

void PoseKalmanFilter::updateKalmanFilter(const cv::Mat &measurement, geometry_msgs::Pose &estimated_pose)
{
  // First predict, to update the internal statePre variable
  kf_.predict();

  // The "correct" phase that is going to use the predicted value and our measurement
  Mat estimated = kf_.correct(measurement);

  // Estimated translation
  estimated_pose.position.x = estimated.at<double>(0);
  estimated_pose.position.y = estimated.at<double>(1);
  estimated_pose.position.z = estimated.at<double>(2);

  // Estimated euler angles
  geometry_msgs::Vector3 rpy;
  rpy.x = estimated.at<double>(6);
  rpy.y = estimated.at<double>(7);
  rpy.z = estimated.at<double>(8);
  estimated_pose.orientation = rot2quat(rpy);

//  ROS_INFO("Estimated position: %f %f %f", estimated_pose.position.x, estimated_pose.position.y,
//           estimated_pose.position.z);
//  ROS_INFO("Estimated roll-pitch-yaw: %f %f %f", rpy.x, rpy.y, rpy.z);
}

void PoseKalmanFilter::getCurrentPoseData(PoseData &pose_data)
{
  Mat state = kf_.statePost;

  // Pose
  pose_data.position.x = state.at<double>(0);
  pose_data.position.y = state.at<double>(1);
  pose_data.position.z = state.at<double>(2);
  pose_data.orientation.x = state.at<double>(6);
  pose_data.orientation.y = state.at<double>(7);
  pose_data.orientation.z = state.at<double>(8);

  // Velocity
  pose_data.velocity.linear.x = state.at<double>(3);
  pose_data.velocity.linear.y = state.at<double>(4);
  pose_data.velocity.linear.z = state.at<double>(5);
  pose_data.velocity.angular.x = state.at<double>(9);
  pose_data.velocity.angular.y = state.at<double>(10);
  pose_data.velocity.angular.z = state.at<double>(11);
  //
  //    // Acceleration
  //    pose_data.acceleration.linear.x = state.at<double>(6);
  //    pose_data.acceleration.linear.y = state.at<double>(7);
  //    pose_data.acceleration.linear.z = state.at<double>(8);
  //    pose_data.acceleration.angular.x = state.at<double>(15);
  //    pose_data.acceleration.angular.y = state.at<double>(16);
  //    pose_data.acceleration.angular.z = state.at<double>(17);
  //
  //    // Covariances
  //    pose_data.processNoiseCov = kf_.processNoiseCov;
  //    pose_data.measurementNoiseCov = kf_.measurementNoiseCov;
  //    pose_data.errorCovPost = kf_.errorCovPost;
}

geometry_msgs::Vector3 PoseKalmanFilter::quat2rot(const geometry_msgs::Quaternion &quaternion)
{
  tf::Quaternion tf_quaternion;
  tf::quaternionMsgToTF(quaternion, tf_quaternion);

  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  // the found angles are written in a geometry_msgs::Vector3
  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  return rpy;
}

geometry_msgs::Quaternion PoseKalmanFilter::rot2quat(const geometry_msgs::Vector3 rpy)
{
  // translate roll, pitch and yaw into a Quaternion
  tf::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);
  geometry_msgs::Quaternion quaternion;
  tf::quaternionTFToMsg(tf_quaternion, quaternion);
  return quaternion;
}