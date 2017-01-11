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

#include <irp6_grasping_msgs/EstimatePose.h>
#include <irp6_grasping_msgs/GetRecognizedObjectPose.h>
#include <irp6_grasping_msgs/GetRecognizedObjectsList.h>

#include <boost/bind.hpp>

#include "PoseKalmanFilter.h"
#include "ResultFileWriter.h"

using namespace std;
using namespace cv;

ros::Subscriber recognized_object_sub;

ros::Publisher recognized_object_pub;

/// Marker publisher - used for displaying objects in rviz.
ros::Publisher marker_publisher;
string marker_namespace;
double marker_max_age;
int displayed_markers_number;

/// Worlds frame id - coordinate frame common for all readings (ROS param).
string world_frame_id;

/// Object information in world coordinates
object_recognition_msgs::RecognizedObject estimated_object;

/// Object initialization
bool estimated_object_initialized;

bool inline estimatedObjectExists() {
    return estimated_object_initialized;
}

void inline setEstimatedObjectExistance(bool exists) {
    estimated_object_initialized = exists;
}

/// Min object confidence - all objects with less confidence are ignored
double min_object_confidence;

/// Kalman filer
double dt = 0.033;           // time between measurements (1/FPS)
int minInliersKalman = 30;   // TODO

PoseKalmanFilter kalman;
Mat measurements;
bool use_kalman_filter;

/// Result file writer
string result_root_dir;
ResultFileWriter result_writer;


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
    tf::Point result_tf_point = tf * initial_tf_point;

    geometry_msgs::Point result_point;
    result_point.x = result_tf_point.getX();
    result_point.y = result_tf_point.getY();
    result_point.z = result_tf_point.getZ();
    return result_point;
}

void recognizedObjectCallback(const object_recognition_msgs::RecognizedObject &received_sensor_object) {
    static tf::TransformListener tf_listener;
    static tf::StampedTransform world_sensor_tf;

    object_recognition_msgs::RecognizedObject sensor_object = received_sensor_object;

    ROS_DEBUG("I received: %s in %s reference frame.", sensor_object.type.key.c_str(),
              sensor_object.pose.header.frame_id.c_str());

    // TODO! what if we do not receive new object for a long time?
    if (estimated_object.type.key.compare(sensor_object.type.key) != 0) {
        ROS_INFO("New object %s ignored.", sensor_object.type.key.c_str());
        if (estimated_object_initialized) {
            sensor_object = estimated_object;
        } else {
            return;
        }
    } else if (sensor_object.confidence < min_object_confidence) {
        if (sensor_object.confidence < 0.001) {
            return;
        }

        ROS_INFO("New object %s has low confidence.", sensor_object.type.key.c_str());
        if (estimatedObjectExists()) {
            sensor_object = estimated_object;
        }
    }
    if (!estimatedObjectExists()) {
        estimated_object.bounding_mesh = sensor_object.bounding_mesh;
        setEstimatedObjectExistance(true);
    }

    try {
        ROS_DEBUG("Getting transform from %s to %s", world_frame_id.c_str(),
                  sensor_object.pose.header.frame_id.c_str());
        // TODO synchronize clock on gerwazy!
        tf_listener.lookupTransform(world_frame_id, sensor_object.pose.header.frame_id,
                                    /*sensor_object.pose.header.stamp*/ ros::Time(0), world_sensor_tf);

        // Update object information
        estimated_object.header.stamp = sensor_object.pose.header.stamp;
        estimated_object.pose.header.stamp = sensor_object.pose.header.stamp;

        float new_confidence = estimated_object.confidence / 2 + sensor_object.confidence;
        estimated_object.confidence = new_confidence > 1 ? 1 : new_confidence;

        geometry_msgs::Pose sensor_object_world_pose = transformPose(sensor_object.pose.pose.pose,
                                                                     world_sensor_tf);
        geometry_msgs::Pose estimated_pose;

        if (use_kalman_filter) {
            kalman.fillMeasurements(sensor_object_world_pose, measurements);
            kalman.updateKalmanFilter(measurements, estimated_pose);
        } else {
            estimated_pose = sensor_object_world_pose;
        }
        estimated_object.pose.pose.pose = estimated_pose;

        PoseData kalman_pose_data;
        kalman.getCurrentPoseData(kalman_pose_data);
        result_writer.writePoseData(kalman_pose_data);

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
}

void createMarkers(const object_recognition_msgs::RecognizedObject &object) {
    std_msgs::Header header = object.header;
    if (header.stamp < ros::Time::now() - ros::Duration(marker_max_age)) {
        return;
    }

    visualization_msgs::Marker marker;

    marker.header.stamp = header.stamp;
    marker.header.frame_id = world_frame_id;
    marker.ns = "marker_object_meshes";
    marker.id = displayed_markers_number++;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(8.0);
    marker.pose = object.pose.pose.pose;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Add triangle list
    std_msgs::ColorRGBA base_color;
    base_color.r = (displayed_markers_number % 1 == 0);
    base_color.g = (displayed_markers_number % 2 == 0);
    base_color.b = (displayed_markers_number % 3 == 0);
    base_color.a = 1.0f;

    shape_msgs::Mesh bounding_mesh = object.bounding_mesh;

    for (size_t i = 0, size = bounding_mesh.triangles.size(); i < size; i++) {
        shape_msgs::MeshTriangle mt = bounding_mesh.triangles[i];

        marker.points.push_back(bounding_mesh.vertices[mt.vertex_indices[0]]);
        marker.points.push_back(bounding_mesh.vertices[mt.vertex_indices[1]]);
        marker.points.push_back(bounding_mesh.vertices[mt.vertex_indices[2]]);

        std_msgs::ColorRGBA color;
        color.r = (float) (i + 1) / (size + 1) * base_color.r;
        color.g = (float) (i + 1) / (size + 1) * base_color.g;
        color.b = (float) (i + 1) / (size + 1) * base_color.b;
        color.a = base_color.a;

        marker.colors.push_back(color);
        marker.colors.push_back(color);
        marker.colors.push_back(color);
    }
    marker_publisher.publish(marker);
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

    br.sendTransform(stamped_transform);
    createMarkers(estimated_object);

    recognized_object_pub.publish(estimated_object);
}


/// Service callbacks

/// Service responsible for turning on/off pose estimation
bool estimatePoseCallback(irp6_grasping_msgs::EstimatePose::Request &request,
                          irp6_grasping_msgs::EstimatePose::Response &response,
                          ros::NodeHandle &node_handle) {
    if (request.calculate_pose) {
        recognized_object_sub = node_handle.subscribe("recognized_objects", 1000, recognizedObjectCallback);
        ROS_INFO("Start pose estimation");
    } else {
        recognized_object_sub.shutdown();
        ROS_INFO("Stop pose estimation");
    }
    return true;
}

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
    nh.param<bool>("use_kalman_filter", use_kalman_filter, true);
    nh.param<double>("min_object_confidence", min_object_confidence, 0.5);
    nh.param<string>("result_root_dir", result_root_dir, "~");

    displayed_markers_number = 0;
    setEstimatedObjectExistance(false);

    estimated_object.type.key = "herbapol_mieta1";
    estimated_object.header.frame_id = world_frame_id;
    estimated_object.pose.header.frame_id = world_frame_id;

    kalman.initKalmanFilter(dt);
    PoseKalmanFilter::initMeasurements(measurements);

    result_writer.init("pose_estimation", result_root_dir);

    // Initialize services.
    ros::ServiceServer estimate_pose = nh.advertiseService<irp6_grasping_msgs::EstimatePose::Request, irp6_grasping_msgs::EstimatePose::Response>(
            "estimate_pose", boost::bind(estimatePoseCallback, _1, _2, boost::ref(nh)));

    ros::ServiceServer object_pose_service = nh.advertiseService("return_recognized_object_pose",
                                                                 returnRecognizedObjectPoseServiceCallback);

    ros::ServiceServer object_list_service = nh.advertiseService("return_recognized_objects_list",
                                                                 returnRecognizedObjectsListServiceCallback);

    // Initialize marker publisher.
    marker_publisher = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    recognized_object_pub = nh.advertise<object_recognition_msgs::RecognizedObject>("global_recognized_objects", 0);

    ROS_INFO("Ready for recognized object pose update and estimation.");

    ros::Rate r(10);
    while (ros::ok()) {
        publishRecognizedObjects();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
