#include <ros/ros.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//#include <tf_conversions/transform_datatypes.h>

// service for acquiring recognized object poses.
#include <irp6_grasping_msgs/GetRecognizedObjectPose.h>



// List of recognized objects.
//std::vector<> estimated_objects;

// TF listener.
tf::TransformListener* listener;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void recognizedObjectPoseUpdateCallback(const object_recognition_msgs::RecognizedObject& object)
{
	ROS_INFO("I heard: [%s]", object.type.key.c_str());


	std::string parent_frame_id = "/tl_base";

	ros::Time now = ros::Time::now();

	ROS_INFO("Waiting for transform from %s to %s", parent_frame_id.c_str(), object.pose.header.frame_id.c_str());

	try{
		// Get sensor-camera transformation.
		tf::StampedTransform world_sensor_tf;
		listener->lookupTransform(parent_frame_id, object.pose.header.frame_id, ros::Time(0), world_sensor_tf);
		ROS_INFO("Got it!");

		// Get object pose in sensor frame.
		tf::Transform sensor_object_tf;
		tf::poseMsgToTF (object.pose.pose.pose , sensor_object_tf);

		// Compute pose in original (kinect/camera) reference frame.
		tf::Transform world_object_tf = world_sensor_tf * sensor_object_tf;

		ROS_INFO("Broadcasting transform from %s to %s", parent_frame_id.c_str(), object.type.key.c_str());

		// Broadcaster.
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(world_object_tf, ros::Time::now(), parent_frame_id, object.type.key));
	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}//: catch
}


// Service responsible for returning recognized object with given object_id.
bool returnRecognizedObjectPoseServiceCallback(irp6_grasping_msgs::GetRecognizedObjectPose::Request &req, irp6_grasping_msgs::GetRecognizedObjectPose::Response &res) {

	return true;
}



int main(int argc, char **argv)
{
	// Initialize node.
	ros::init(argc, argv, "recognized_objects_pose_estimation");

	// Handle to communication ports.
	ros::NodeHandle node;

	// Initialize TF listener.
	listener = new tf::TransformListener();

	// Create subscriber to topic containing recognized objects.
	ros::Subscriber sub = node.subscribe("RecognizedObjects", 1000, recognizedObjectPoseUpdateCallback);

	// Initialize service.
	ros::ServiceServer service = node.advertiseService("returnRecognizedObjectPose", returnRecognizedObjectPoseServiceCallback);

	ROS_INFO("Ready for recognized object pose update and estimation.");

	// Enter main loop, pumping callbacks.
	ros::spin();

	return 0;
}
