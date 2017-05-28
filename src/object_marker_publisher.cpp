//
// Created by ola on 28.05.17.
//

#include <irp6_grasping/object_maker_publisher.h>
#include <visualization_msgs/Marker.h>

using namespace irp6_grasping;

ObjectMarkerPublisher::ObjectMarkerPublisher(ros::NodeHandle &nh, const std::string &frame_id)
  : frame_id_(frame_id), num_displayed_markers_(0)
{
  nh.param<std::string>("marker_namespace", namespace_, "object_marker_namespace");
  nh.param<double>("marker_max_age", max_age_, 5.0);
  publisher_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

void ObjectMarkerPublisher::deleteMarkers()
{
  visualization_msgs::Marker del_marker;
  del_marker.ns = namespace_;
  del_marker.header.frame_id = frame_id_;
  del_marker.header.stamp = ros::Time();
  del_marker.action = visualization_msgs::Marker::DELETEALL;
  publisher_.publish(del_marker);
  num_displayed_markers_ = 0;
}

void ObjectMarkerPublisher::createMarkers(const object_recognition_msgs::RecognizedObject &object)
{
  std_msgs::Header header = object.header;
  if (header.stamp < ros::Time::now() - ros::Duration(max_age_))
  {
    return;
  }

  visualization_msgs::Marker marker;

  marker.header.stamp = header.stamp;
  marker.header.frame_id = frame_id_;
  marker.ns = namespace_;
  marker.id = num_displayed_markers_++;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(8.0);
  marker.pose = object.pose.pose.pose;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // Add triangle list
  std_msgs::ColorRGBA base_color;
  base_color.r = (num_displayed_markers_ % 1 == 0);
  base_color.g = (num_displayed_markers_ % 2 == 0);
  base_color.b = (num_displayed_markers_ % 3 == 0);
  base_color.a = 1.0f;

  shape_msgs::Mesh bounding_mesh = object.bounding_mesh;

  for (size_t i = 0, size = bounding_mesh.triangles.size(); i < size; i++)
  {
    shape_msgs::MeshTriangle mt = bounding_mesh.triangles[i];

    marker.points.push_back(bounding_mesh.vertices[mt.vertex_indices[0]]);
    marker.points.push_back(bounding_mesh.vertices[mt.vertex_indices[1]]);
    marker.points.push_back(bounding_mesh.vertices[mt.vertex_indices[2]]);

    std_msgs::ColorRGBA color;
    color.r = (float)(i + 1) / (size + 1) * base_color.r;
    color.g = (float)(i + 1) / (size + 1) * base_color.g;
    color.b = (float)(i + 1) / (size + 1) * base_color.b;
    color.a = base_color.a;

    marker.colors.push_back(color);
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  }
  publisher_.publish(marker);
}
