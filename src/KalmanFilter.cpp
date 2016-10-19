/*!
 * \file KalmanFilter.cpp
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(int nStates, int nMeasurements, int nInputs, double dt) {
    kf_.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));             // error covariance

    /* DYNAMIC MODEL */
    //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
    //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

    // position
    kf_.transitionMatrix.at<double>(0, 3) = dt;
    kf_.transitionMatrix.at<double>(1, 4) = dt;
    kf_.transitionMatrix.at<double>(2, 5) = dt;
    kf_.transitionMatrix.at<double>(3, 6) = dt;
    kf_.transitionMatrix.at<double>(4, 7) = dt;
    kf_.transitionMatrix.at<double>(5, 8) = dt;
    kf_.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
    kf_.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
    kf_.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);

    // orientation
    kf_.transitionMatrix.at<double>(9, 12) = dt;
    kf_.transitionMatrix.at<double>(10, 13) = dt;
    kf_.transitionMatrix.at<double>(11, 14) = dt;
    kf_.transitionMatrix.at<double>(12, 15) = dt;
    kf_.transitionMatrix.at<double>(13, 16) = dt;
    kf_.transitionMatrix.at<double>(14, 17) = dt;
    kf_.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
    kf_.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
    kf_.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);


    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

    kf_.measurementMatrix.at<double>(0, 0) = 1;  // x
    kf_.measurementMatrix.at<double>(1, 1) = 1;  // y
    kf_.measurementMatrix.at<double>(2, 2) = 1;  // z
    kf_.measurementMatrix.at<double>(3, 9) = 1;  // roll
    kf_.measurementMatrix.at<double>(4, 10) = 1; // pitch
    kf_.measurementMatrix.at<double>(5, 11) = 1; // yaw
}