/*!
 * \file PoseEstimation.cpp
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_recognition_msgs/RecognizedObject.h>

#include <irp6_grasping_msgs/GetRecognizedObjectPose.h>
#include <irp6_grasping_msgs/GetRecognizedObjectsList.h>

#include "KalmanFilter.h"

using namespace std;


/// Marker publisher - used for displaying objects in rviz.
ros::Publisher marker_publisher;
string marker_namespace;
double marker_max_age;
int displayed_markers_number;

/// Worlds frame id - coordinate frame common for all readings (ROS param).
string world_frame_id;

/// Object information in world coordinates
object_recognition_msgs::RecognizedObject estimated_object;


/// Function transforms pose from one to another coordinate frame.
geometry_msgs::Pose transformPose(const geometry_msgs::Pose &start_pose, const tf::StampedTransform &end_tf) {
    tf::Transform start_tf;
    tf::poseMsgToTF(start_pose, start_tf);
    tf::Transform start_end_tf = end_tf * start_tf;

    geometry_msgs::Pose transformed_pose;
    tf::poseTFToMsg(start_end_tf, transformed_pose);
    return transformed_pose;
}

/// Function transforms point from one to another coordinate frame.
geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::Transform tf) {
    tf::Point initial_tf_point(point.x, point.y, point.z);
//    sensor_pt_xyz[0] = sensor_pt_.x;
//    sensor_pt_xyz[1] = sensor_pt_.y;
//    sensor_pt_xyz[2] = sensor_pt_.z;
    tf::Point result_tf_point = tf * initial_tf_point;

    geometry_msgs::Point result_point;
    result_point.x = result_tf_point.getX();
    result_point.y = result_tf_point.getY();
    result_point.z = result_tf_point.getZ();
//    world_pt.x = world_pt_xyz[0];
//    world_pt.y = world_pt_xyz[1];
//    world_pt.z = world_pt_xyz[2];
    return result_point;
}

void recognizedObjectCallback(const object_recognition_msgs::RecognizedObject &sensor_object) {
    static tf::TransformListener tf_listener;
    static tf::StampedTransform world_sensor_tf;

    ROS_DEBUG("I received: %s in %s reference frame.", sensor_object.type.key.c_str(),
              sensor_object.pose.header.frame_id.c_str());

    if (estimated_object.type.key.compare(sensor_object.type.key) != 0) {
        ROS_INFO("New object %s ignored.", sensor_object.type.key.c_str());
        return;
    }

    try {
        ROS_DEBUG("Getting transform from %s to %s", world_frame_id.c_str(),
                  sensor_object.pose.header.frame_id.c_str());
        tf_listener.lookupTransform(world_frame_id, sensor_object.pose.header.frame_id,
                                    sensor_object.pose.header.stamp, world_sensor_tf);

        // Update object information
        estimated_object.header.stamp = sensor_object.pose.header.stamp;
        estimated_object.pose.header.stamp = sensor_object.pose.header.stamp;

        float new_confidence = estimated_object.confidence / 2 + sensor_object.confidence;
        estimated_object.confidence = new_confidence > 1 ? 1 : new_confidence;

        geometry_msgs::Pose sensor_object_world_pose = transformPose(sensor_object.pose.pose.pose,
                                                                     world_sensor_tf);

        // TODO kalman filter
        estimated_object.pose.pose.pose = sensor_object_world_pose;

        estimated_object.bounding_mesh.vertices.clear();
        for (size_t i = 0, size = sensor_object.bounding_mesh.vertices.size(); i < size; ++i) {
            estimated_object.bounding_mesh.vertices.push_back(
                    transformPoint(sensor_object.bounding_mesh.vertices[i], world_sensor_tf));
        }
    } catch (tf::TransformException &e) {
        ROS_ERROR("%s %s", e.what(), world_frame_id.c_str());
    }
}

void deleteMarkers() {
    visualization_msgs::Marker del_marker;
    del_marker.ns = marker_namespace;
    del_marker.header.frame_id = world_frame_id;
    del_marker.header.stamp = ros::Time();
    del_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_publisher.publish(del_marker);
    displayed_markers_number = 0;
//    for(size_t marker_i=0; marker_i < displayed_markers_size; marker_i++) {
//        visualization_msgs::Marker del_marker;
//        del_marker.ns = "marker_object_meshes";
//        del_marker.id = marker_i;
//        del_marker.header.frame_id = world_frame_id;
//        del_marker.header.stamp = ros::Time();
//        del_marker.action = visualization_msgs::Marker::DELETE;
//        marker_publisher.publish( del_marker );
//    }
}

void createMarkers(const object_recognition_msgs::RecognizedObject &object) {
    // TODO
}

bool inline estimatedObjectExists() {
    return estimated_object.type.key.compare("") != 0;
}

void publishRecognizedObjects() {
    static tf::TransformBroadcaster br;

    if (!estimatedObjectExists()) {
        return;
    }

    deleteMarkers();
    ROS_DEBUG("Adding transform from %s to %s to broadcasted TF message", estimated_object.header.frame_id.c_str(),
              estimated_object.type.key.c_str());
    ROS_DEBUG("Transform %f %f %f ", estimated_object.pose.pose.pose.position.x,
              estimated_object.pose.pose.pose.position.y,
              estimated_object.pose.pose.pose.position.z);

    tf::Transform transform;
    tf::poseMsgToTF(estimated_object.pose.pose.pose, transform);

    tf::StampedTransform stamped_transform(transform, estimated_object.pose.header.stamp,
                                           estimated_object.header.frame_id, estimated_object.type.key);
//        // Publish mesh as marker.
//        publish_object_mesh_as_marker(displayed_markers_number, estimated_object.header, estimated_object.bounding_mesh);
    br.sendTransform(stamped_transform);
    createMarkers(estimated_object);
}


/// Service callbacks

/// Service responsible for returning recognized object with given object_id and not older that given age.
bool returnRecognizedObjectPoseServiceCallback(irp6_grasping_msgs::GetRecognizedObjectPose::Request &request,
                                               irp6_grasping_msgs::GetRecognizedObjectPose::Response &response) {
    if (!estimatedObjectExists()) {
        response.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::NO_OBJECTS;
        ROS_INFO("No object available.");
        return true;
    }

    if (estimated_object.type.key.compare(request.object_id) != 0 ||
        estimated_object.header.stamp < ros::Time::now() - request.age) {

        response.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::OBJECT_NOT_FOUND;
        ROS_INFO("Object %s not found.", request.object_id.c_str());
        return true;
    }

    response.object = estimated_object;
    response.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::OBJECT_FOUND;
    ROS_INFO("Returning object %s (%f).", response.object.type.key.c_str(), response.object.confidence);
    return true;
}

/// Service responsible for returning list of recognized object model_ids with with given object_id and not older that given age.
bool returnRecognizedObjectsListServiceCallback(irp6_grasping_msgs::GetRecognizedObjectsList::Request &request,
                                                irp6_grasping_msgs::GetRecognizedObjectsList::Response &response) {
    if (!estimatedObjectExists() || estimated_object.header.stamp < ros::Time::now() - request.age) {
        ROS_INFO("No object available.");
        return true;
    }

    //Limit the size of returned vector - 0 means that there is no limit.
    if ((request.limit > 0) && (response.object_ids.size() >= request.limit)) {
        ROS_INFO("Cannot insert object due to limit.");
        return true;
    }

    // Fill oid data.
    irp6_grasping_msgs::ObjectID oid;
    oid.id = estimated_object.type.key;
    oid.confidence = estimated_object.confidence;

    // First case: insert first oid.
    if (response.object_ids.size() == 0) {
        response.object_ids.push_back(oid);
    } else {
        // Second case: insert in proper order.
        bool added = false;
        for (vector<irp6_grasping_msgs::ObjectID>::iterator it = response.object_ids.begin(), end_it = response.object_ids.end(); it != end_it; ++it) {
            ROS_DEBUG("Service - iterating: %f < %f ?", it->confidence, oid.confidence);
            if (it->confidence < oid.confidence) {
                // Insert here! (i.e. before)
                response.object_ids.insert(it, oid);
                added = true;
                break;
            }
        }
        // Third case: insert at the end.
        if (!added)
            response.object_ids.push_back(oid);
    }
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "recognized_objects_pose_estimation");
    ros::NodeHandle nh;

    // Read parameters
    nh.param<string>("world_frame_id", world_frame_id, "/tl_base");
    nh.param<string>("marker_namespace", marker_namespace, "object_marker_namespace");
    nh.param<double>("marker_max_age", marker_max_age, 5.0);
    displayed_markers_number = 0;

    estimated_object.type.key = "herbapol_mieta1";
    estimated_object.header.frame_id = world_frame_id;
    estimated_object.pose.header.frame_id = world_frame_id;

    // Recognized objects subscriber.
    ros::Subscriber sub = nh.subscribe("recognized_objects", 1000, recognizedObjectCallback);

    // Initialize services.
    ros::ServiceServer object_pose_service = nh.advertiseService("return_recognized_object_pose",
                                                                 returnRecognizedObjectPoseServiceCallback);
    ros::ServiceServer object_list_service = nh.advertiseService("return_recognized_objects_list",
                                                                 returnRecognizedObjectsListServiceCallback);

    // Initialize marker publisher.
    marker_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    ROS_INFO("Ready for recognized object pose update and estimation.");

    ros::Rate r(10);
    while (ros::ok()) {
        publishRecognizedObjects();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}