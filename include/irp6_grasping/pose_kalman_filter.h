/*!
 * \file PoseKalmanFilter.h
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#ifndef IRP6_GRASPING_POSEKALMANFILTER_H
#define IRP6_GRASPING_POSEKALMANFILTER_H

// Header containing Kalman filter.
#include <opencv2/video/tracking.hpp>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class PoseData {
public:
    geometry_msgs::Point position;      // x, y, z
    geometry_msgs::Vector3 orientation; // roll, pitch, yaw
//    geometry_msgs::Twist velocity;
//    geometry_msgs::Accel acceleration;
//
//    cv::Mat processNoiseCov;
//    cv::Mat measurementNoiseCov;
//    cv::Mat errorCovPost;
};

class PoseKalmanFilter {
public:
    /// Initialize Kalman Filter
    void initKalmanFilter(double dt);

    /// Initialize measurements - size, type
    static void initMeasurements(cv::Mat &measurements);

    /// Fill measurements based on mesured pose
    static void fillMeasurements(const geometry_msgs::Pose &measured_pose, cv::Mat &measurements);

    /// Update kalman filter and calculate estimated position
    void updateKalmanFilter(const cv::Mat &measurement, geometry_msgs::Pose &estimated_pose);

    void getCurrentPoseData(PoseData &pose_data);

private:
    cv::KalmanFilter kf_;

    static geometry_msgs::Vector3 quat2rot_(const geometry_msgs::Quaternion &quaternion);
    static geometry_msgs::Quaternion rot2quat_(const geometry_msgs::Vector3 rpy);
};

#endif //IRP6_GRASPING_KALMANFILTER_H
