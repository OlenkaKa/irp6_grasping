/*!
 * \file KalmanFilter.h
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#ifndef IRP6_GRASPING_KALMANFILTER_H
#define IRP6_GRASPING_KALMANFILTER_H

// Header containing Kalman filter.
#include <opencv2/video/tracking.hpp>

class KalmanFilter {
public:
    KalmanFilter(int nStates, int nMeasurements, int nInputs, double dt);
private:
    cv::KalmanFilter kf_;
};

#endif //IRP6_GRASPING_KALMANFILTER_H
