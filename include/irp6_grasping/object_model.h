//
// Created by ola on 27.05.17.
//

#ifndef IRP6_GRASPING_OBJECT_MODEL_H
#define IRP6_GRASPING_OBJECT_MODEL_H

#include <shape_msgs/Mesh.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include "pose_kalman_filter.h"

namespace irp6_grasping
{
class ObjectModel
{
public:
  ObjectModel();
  void setBoundingMesh(const shape_msgs::Mesh &bounding_mesh);
  void setId(const std::string &id);
  void setInitialized(bool initialized);

  const std::string &getId() const;
  bool isInitialized() const;

  void addData(object_recognition_msgs::RecognizedObject &object) const;

private:
  std::string id_;
  shape_msgs::Mesh bounding_mesh_;
  bool initialized_;
};
}  // irp6_grasping

#endif  // IRP6_GRASPING_OBJECT_MODEL_H
