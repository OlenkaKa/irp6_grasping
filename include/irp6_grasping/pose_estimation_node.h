//
// Created by ola on 28.05.17.
//

#ifndef IRP6_GRASPING_POSE_ESTIMATION_NODE_H
#define IRP6_GRASPING_POSE_ESTIMATION_NODE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <irp6_grasping_msgs/EstimatePose.h>
#include <irp6_grasping_msgs/GetRecognizedObjectPose.h>
#include <irp6_grasping_msgs/GetRecognizedObjectsList.h>

#include "object_maker_publisher.h"
#include "object_model.h"
#include "object_pose_estimator.h"
#include "pose_kalman_filter.h"
#include "result_file_writer.h"

namespace irp6_grasping
{
class PoseEstimationNode
{
public:
  void execute(ros::NodeHandle &nh);

private:
  void finish();

  object_recognition_msgs::RecognizedObject createViewRecognizedObject() const;

  object_recognition_msgs::RecognizedObject createRecognizedObject() const;

  void publishRecognizedObjects();

  // Service callbacks

  void recognizedObjectCallback(const object_recognition_msgs::RecognizedObject &received_sensor_object);

  /// Service responsible for turning on/off pose estimation
  bool estimatePoseCallback(irp6_grasping_msgs::EstimatePose::Request &request,
                            irp6_grasping_msgs::EstimatePose::Response &response, ros::NodeHandle &node_handle);

  /// Service responsible for returning recognized object with given object_id and not older that given age.
  bool returnRecognizedObjectPoseServiceCallback(irp6_grasping_msgs::GetRecognizedObjectPose::Request &request,
                                                 irp6_grasping_msgs::GetRecognizedObjectPose::Response &response);

  /// Service responsible for returning list of recognized object model_ids with with given object_id and not older that
  /// given age.
  bool returnRecognizedObjectsListServiceCallback(irp6_grasping_msgs::GetRecognizedObjectsList::Request &request,
                                                  irp6_grasping_msgs::GetRecognizedObjectsList::Response &response);

  /// Variables
  int view_id_;

  ros::Subscriber recognized_object_sub;

  ros::Publisher recognized_object_pub;

  /// Marker publisher - used for displaying objects in rviz.
  ObjectMarkerPublisher *marker_publisher_;

  /// Worlds frame id - coordinate frame common for all readings (ROS param).
  std::string world_frame_id;

  /// Result file writer
  ResultFileWriter *result_writer_;

  /// Recognized object input result file writer
  ResultFileWriter *input_writer_;

  ObjectModel object_model_;
  ObjectPoseEstimator *object_pose_estimator_;
};

}  // irp6_grasping

#endif  // IRP6_GRASPING_POSE_ESTIMATION_NODE_H
