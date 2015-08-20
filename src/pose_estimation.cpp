#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_recognition_msgs/RecognizedObject.h>

typedef std::vector<object_recognition_msgs::RecognizedObject>::iterator ro_it_t;

// Service for acquiring recognized object pose.
#include <irp6_grasping_msgs/GetRecognizedObjectPose.h>

// Service for acquisition of list of recognized objects.
#include <irp6_grasping_msgs/GetRecognizedObjectsList.h>

typedef std::vector<irp6_grasping_msgs::ObjectID>::iterator oid_it_t;


// List of recognized AND estimated objects.
std::vector<object_recognition_msgs::RecognizedObject> estimated_objects;


// TF listener.
tf::TransformListener* listener;

/// Worlds frame id - coordinate frame common for all readings (ROS param).
std::string world_frame_id;




/**
 * Function recalculates object pose from sensor frame (given in message) to world frame (in TF).
 */
tf::Transform transformFromSensorPoseMsgToWorldTF(
	const geometry_msgs::Pose & sensor_object_msg_pose_,
	const std::string sensor_frame_id_,
	const ros::Time stamp_)
{
	// Get object pose in sensor frame.
	tf::Transform sensor_object_tf;
	tf::poseMsgToTF (sensor_object_msg_pose_ , sensor_object_tf);

	ROS_DEBUG("Waiting for transform from %s to %s", world_frame_id.c_str(), sensor_frame_id_.c_str());

	// Get sensor-camera transformation.
	tf::StampedTransform world_sensor_tf;

	// Throws exception that must be handled outside of the function.
	listener->lookupTransform(world_frame_id, sensor_frame_id_, stamp_, world_sensor_tf);

	// Compute pose in original (kinect/camera) reference frame.
	return (world_sensor_tf * sensor_object_tf);
}//: end

/**
 * @brief Function checks whether there is that object already on the list.
 * @return index od object of -1 if not found.
 */
int tryToFindObject(const object_recognition_msgs::RecognizedObject& object_) {
	for (size_t i=0; i < estimated_objects.size(); i++ ) {
		// TODO: compare pose!!
		if (estimated_objects[i].type.key == object_.type.key){
			ROS_DEBUG("Object %s found on the list", object_.type.key.c_str());
			return (int)i;
		}//: if
	}//: for

	return -1;
}//: end


/**
 * @brief Broadcasts poses of recognized objects on TF topic (in single message).
 */
void broadcastRecognizedObjectsTFs() {
	// Broadcaster.
	static tf::TransformBroadcaster br;
	std::vector< tf::StampedTransform > transforms;

	// Broadcast the possessed poses.
	for(ro_it_t obj=estimated_objects.begin(); obj != estimated_objects.end(); obj++) {
		ROS_DEBUG("Adding transform from %s to %s to broadcasted TF message", obj->header.frame_id.c_str(), obj->type.key.c_str());
		tf::Transform transform;
		ROS_DEBUG("Transform %f %f %f ", obj->pose.pose.pose.position.x, obj->pose.pose.pose.position.y, obj->pose.pose.pose.position.z);
		tf::poseMsgToTF (obj->pose.pose.pose , transform);
		transforms.push_back(
					tf::StampedTransform(transform, obj->pose.header.stamp, obj->header.frame_id, obj->type.key));
	}//: for

	ROS_DEBUG ("Broadcasting [%d] pose(s) at once", (int)transforms.size());
	// Publish all stamped transforms at once.
	br.sendTransform(transforms);

}


/**
 * Listener responsible for acquiring and estimating poses of recognized objects in world reference frame.
 */
void recognizedObjectPoseUpdateCallback(const object_recognition_msgs::RecognizedObject& object_)
{
	ROS_DEBUG("I received: %s in %s reference frame ", object_.type.key.c_str(), object_.pose.header.frame_id.c_str());

	try{
		tf::Transform world_object_tf = transformFromSensorPoseMsgToWorldTF(object_.pose.pose.pose, object_.pose.header.frame_id, object_.pose.header.stamp);

		// Try to find the object with the same model_id and pose.
		int index = tryToFindObject(object_);
		if (index >= 0) {
			// If found - update its pose and time stamps.
			ROS_DEBUG("Object found");
			// Set new timestamp!
			estimated_objects[index].header.stamp = object_.pose.header.stamp;
			estimated_objects[index].pose.header.stamp = object_.pose.header.stamp;

			// Update confidence
			estimated_objects[index].confidence = estimated_objects[index].confidence/2 + object_.confidence;


			// Update old object pose wrt to world coordinate frame. - TODO Kalman!!!!
			geometry_msgs::Pose tmp_msg;
			tf::poseTFToMsg(world_object_tf, tmp_msg);
			estimated_objects[index].pose.pose.pose = tmp_msg;

		} else {
			// If not - create new "object instance".
			ROS_DEBUG("New Object");

			// TODO: Kalman filter!

			// Set object pose wrt to world coordinate frame.
			geometry_msgs::Pose tmp_msg;
			tf::poseTFToMsg(world_object_tf, tmp_msg);

			// Copy data.
			object_recognition_msgs::RecognizedObject tmp_object (object_);
			// Change pose and set adequate reference frames.
			tmp_object.pose.pose.pose = tmp_msg;
			tmp_object.header.frame_id = world_frame_id;
			tmp_object.pose.header.frame_id = world_frame_id;

			// Add object to list.
			estimated_objects.push_back(tmp_object);
		}//: else


	} catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
	}//: catch
}//: end


/// Service responsible for returning recognized object with given object_id and not older that given age.
bool returnRecognizedObjectPoseServiceCallback(
		irp6_grasping_msgs::GetRecognizedObjectPose::Request &req_,
		irp6_grasping_msgs::GetRecognizedObjectPose::Response &res_)
{
	// Check size
	if (estimated_objects.size() == 0) {
		res_.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::NO_OBJECTS;
		ROS_INFO("No objects available on the list.");
		return true;
	}//: if

	return true;
}//: end


/// Service responsible for returning list of recognized object model_ids with with given object_id and not older that given age.
bool returnRecognizedObjectsListServiceCallback(
		irp6_grasping_msgs::GetRecognizedObjectsList::Request &req_,
		irp6_grasping_msgs::GetRecognizedObjectsList::Response &res_)
{
	// Special case: do not return anything if t here are no objects;)
	if (estimated_objects.size() == 0){
		ROS_INFO("No objects available on the list.");
		return true;
	}//: if


	// Iterate and add names to list - in proper order, sorted by confidence.
	for(ro_it_t ro_i=estimated_objects.begin(); ro_i != estimated_objects.end(); ro_i++) {
		// Check object age - if it is older than age - skip.
		if (ro_i->header.stamp < ros::Time::now() - req_.age)
			continue;

		// Fill oid data.
		irp6_grasping_msgs::ObjectID oid;
		oid.id = ro_i->type.key;
		oid.confidence = ro_i->confidence;

		// First case: insert first oid.
		if (res_.object_ids.size() == 0) {
			ROS_INFO("Special case! - wstawiam");
			res_.object_ids.push_back(oid);
			continue;
		}//: if

		// Second case: insert in proper order.
		bool added = false;
		for (oid_it_t oid_i=res_.object_ids.begin(); oid_i<res_.object_ids.end(); oid_i++) {
			ROS_INFO("Service - iterating: %f  < %f ?",oid_i->confidence, oid.confidence);

			if (oid_i->confidence < oid.confidence){
				// Insert here! (i.e. before)
				res_.object_ids.insert(oid_i, oid);
				ROS_INFO("Wstawiam! - break!");
				added = true;
				break;
			}//: if
		}//: for*/

		// Third case: insert at the end.
		if (!added)
			res_.object_ids.push_back(oid);

		// Limit the size of returned vector - 0 means that there is no limit.
		if ((req_.limit> 0)&&(res_.object_ids.size() > req_.limit))
			break;
	}//: for

	return true;
}//: end




int main(int argc, char **argv)
{
	// Initialize node.
	ros::init(argc, argv, "recognized_objects_pose_estimation");

	// Handle to communication ports.
	ros::NodeHandle node;

	// Read parameter - name of the world coordinate frame.
	node.param<std::string>("world_frame_id", world_frame_id, "/tl_base");

	// Initialize TF listener.
	listener = new tf::TransformListener();

	// Create subscriber to topic containing recognized objects.
	ros::Subscriber sub = node.subscribe("recognized_objects", 1000, recognizedObjectPoseUpdateCallback);

	// Initialize services.
	ros::ServiceServer service1 = node.advertiseService("return_recognized_object_pose", returnRecognizedObjectPoseServiceCallback);
	ros::ServiceServer service2 = node.advertiseService("return_recognized_objects_list", returnRecognizedObjectsListServiceCallback);

	ROS_INFO("Ready for recognized object pose update and estimation.");

	// Enter main loop, pumping callbacks.
	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		broadcastRecognizedObjectsTFs();
		ros::spinOnce();
		r.sleep();
	}


	return 0;
}//: end main
