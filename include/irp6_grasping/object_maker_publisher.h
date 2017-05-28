//
// Created by ola on 28.05.17.
//

#ifndef IRP6_GRASPING_OBJECT_MAKER_PUBLISHER_H
#define IRP6_GRASPING_OBJECT_MAKER_PUBLISHER_H

#include <ros/ros.h>
#include <object_recognition_msgs/RecognizedObject.h>

namespace irp6_grasping
{
class ObjectMarkerPublisher
{
public:
  ObjectMarkerPublisher(ros::NodeHandle &nh, const std::string &frame_id);
  void createMarkers(const object_recognition_msgs::RecognizedObject &object);
  void deleteMarkers();

private:
  ros::Publisher publisher_;
  std::string frame_id_;
  std::string namespace_;
  int num_displayed_markers_;
  double max_age_;
};
}  // irp6_grasping

#endif  // IRP6_GRASPING_OBJECT_MAKER_PUBLISHER_H
