//
// Created by ola on 27.05.17.
//

#include <irp6_grasping/object_model.h>

using namespace irp6_grasping;

ObjectModel::ObjectModel()
{
  bounding_mesh_.vertices.clear();
  bounding_mesh_.triangles.clear();
}

void ObjectModel::setBoundingMesh(const shape_msgs::Mesh &bounding_mesh)
{
  if (bounding_mesh_.vertices.empty() || bounding_mesh_.triangles.empty())
  {
    bounding_mesh_ = bounding_mesh;
  }
}
void ObjectModel::setId(const std::string &id)
{
  id_ = id;
}

const std::string &ObjectModel::getId() const
{
  return id_;
}

void ObjectModel::addData(object_recognition_msgs::RecognizedObject &object) const
{
  object.type.key = id_;
  object.bounding_mesh = bounding_mesh_;
}
