#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <object_recognition_msgs/RecognizedObject.h>

// Header containing Kalman filter.
#include <opencv2/video/tracking.hpp>


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

// Marker publisher - used for displaying objects.
ros::Publisher vis_marker_publisher;

/// Worlds frame id - coordinate frame common for all readings (ROS param).
std::string world_frame_id;

/// Max age of marker - afterwards it will not be displayed.
double marker_max_age;




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


/// Variable storing number of displayed markers.
int displayed_markers_size = 0;


void publish_object_mesh_as_marker(int & marker_id_, std_msgs::Header header_, shape_msgs::Mesh bounding_mesh_){
	// Check marker age - if it is older than age - skip.
	if (header_.stamp < ros::Time::now() - ros::Duration(marker_max_age))
		return;

	visualization_msgs::Marker marker;
	// Set header.
	marker.header = header_;

	marker.ns = "marker_object_meshes";
	marker.id = marker_id_++;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	std_msgs::ColorRGBA c;
	c.r = (marker_id_%1 == 0);
	c.g = (marker_id_%2 == 0);
	c.b = (marker_id_%3 == 0);
	// Alpha (opacity).
	// c.a = 1.0f;
	int size = bounding_mesh_.triangles.size();

	// Iterate on mesh triangles.
	for (size_t i=0; i< size; i++) {
		shape_msgs::MeshTriangle mt = bounding_mesh_.triangles[i];
		// Add triangle vertices.
		marker.points.push_back(bounding_mesh_.vertices[mt.vertex_indices[0]]);
		marker.points.push_back(bounding_mesh_.vertices[mt.vertex_indices[1]]);
		marker.points.push_back(bounding_mesh_.vertices[mt.vertex_indices[2]]);
		// Add triangle colour
		std_msgs::ColorRGBA ct;
		ct.r = (double)(i+1)/(size+1) *c.r;
		ct.g = (double)(i+1)/(size+1) *c.g;
		ct.b = (double)(i+1)/(size+1) *c.b;
		// Alpha (opacity).
		ct.a = 1.0f;

		marker.colors.push_back(ct);
		marker.colors.push_back(ct);
		marker.colors.push_back(ct);
	}//: for

	vis_marker_publisher.publish( marker );

}


/**
 * @brief Broadcasts poses of recognized objects: poses on TF topic (in single message) and meshes on "virau
 */
void broadcastRecognizedObjectsTFs() {
	// Broadcaster.
	static tf::TransformBroadcaster br;
	std::vector< tf::StampedTransform > transforms;

	// Delete all markers.
	// del_marker.action = visualization_msgs::Marker::DELETEALL; -- UNAVAILABLE IN INDIGO!
	for(size_t marker_i=0; marker_i < displayed_markers_size; marker_i++) {
		visualization_msgs::Marker del_marker;
		del_marker.ns = "marker_object_meshes";
		del_marker.id = marker_i;
		del_marker.header.frame_id = world_frame_id;
		del_marker.header.stamp = ros::Time();
		del_marker.action = visualization_msgs::Marker::DELETE;
		vis_marker_publisher.publish( del_marker );
	}//: for


	displayed_markers_size=0;
	// Broadcast the possessed poses.
	for(ro_it_t obj=estimated_objects.begin(); obj != estimated_objects.end(); obj++) {
		// FOR DEBUG PURPOSES!!
/*		obj->header.stamp = ros::Time::now();
		obj->pose.header.stamp = ros::Time::now();/**/

		ROS_DEBUG("Adding transform from %s to %s to broadcasted TF message", obj->header.frame_id.c_str(), obj->type.key.c_str());
		tf::Transform transform;
		ROS_DEBUG("Transform %f %f %f ", obj->pose.pose.pose.position.x, obj->pose.pose.pose.position.y, obj->pose.pose.pose.position.z);
		tf::poseMsgToTF (obj->pose.pose.pose , transform);
		// Add transform to published vector.
		transforms.push_back(
					tf::StampedTransform(transform, obj->pose.header.stamp, obj->header.frame_id, obj->type.key));
		// Publish mesh as marker.
		publish_object_mesh_as_marker(displayed_markers_size, obj->header, obj->bounding_mesh);

	}//: for

	ROS_DEBUG ("Broadcasting [%d] pose(s) at once", (int)transforms.size());
	// Publish all stamped transforms at once.
	br.sendTransform(transforms);

}



/// Function transforms pose from sensor to world coordinate frame.
geometry_msgs::Pose sensorToWorlMsgPose(
	const geometry_msgs::Pose & sensor_object_msg_pose_,
	const tf::Transform world_sensor_tf_)
{
	// Get object pose in sensor frame.
	tf::Transform sensor_object_tf;
	tf::poseMsgToTF (sensor_object_msg_pose_ , sensor_object_tf);
	// Compute pose in original (kinect/camera) reference frame.
	tf::Transform world_sensor_tf = world_sensor_tf_ * sensor_object_tf;
	geometry_msgs::Pose world_sensor_msg_pose;
	tf::poseTFToMsg(world_sensor_tf, world_sensor_msg_pose);
	return world_sensor_msg_pose;
}//: end



/// Function transforms point from sensor to world coordinate frame.
static inline geometry_msgs::Point sensorToWorldMsgPoint(
		const geometry_msgs::Point & sensor_pt_,
		const tf::Transform world_sensor_tf_)
{
	// Copy data to "normal point".
	tf::Point sensor_pt_xyz;
	sensor_pt_xyz[0] = sensor_pt_.x;
	sensor_pt_xyz[1] = sensor_pt_.y;
	sensor_pt_xyz[2] = sensor_pt_.z;
	// Transform.
	tf::Point world_pt_xyz = world_sensor_tf_ * sensor_pt_xyz;
	// Copy result to "msg point".
	geometry_msgs::Point world_pt;
	world_pt.x = world_pt_xyz[0];
	world_pt.y = world_pt_xyz[1];
	world_pt.z = world_pt_xyz[2];
	return world_pt;
}

void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{
  KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
				 /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);
  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);
	   /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}


// Converts a given Rotation Matrix to Euler angles
cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
  cv::Mat euler(3,1,CV_64F);

  double m00 = rotationMatrix.at<double>(0,0);
  double m02 = rotationMatrix.at<double>(0,2);
  double m10 = rotationMatrix.at<double>(1,0);
  double m11 = rotationMatrix.at<double>(1,1);
  double m12 = rotationMatrix.at<double>(1,2);
  double m20 = rotationMatrix.at<double>(2,0);
  double m22 = rotationMatrix.at<double>(2,2);

  double x, y, z;

  // Assuming the angles are in radians.
  if (m10 > 0.998) { // singularity at north pole
	x = 0;
	y = CV_PI/2;
	z = atan2(m02,m22);
  }
  else if (m10 < -0.998) { // singularity at south pole
	x = 0;
	y = -CV_PI/2;
	z = atan2(m02,m22);
  }
  else
  {
	x = atan2(-m12,m11);
	y = asin(m10);
	z = atan2(-m20,m00);
  }

  euler.at<double>(0) = x;
  euler.at<double>(1) = y;
  euler.at<double>(2) = z;

  return euler;
}

// Converts a given Euler angles to Rotation Matrix
cv::Mat euler2rot(const cv::Mat & euler)
{
  cv::Mat rotationMatrix(3,3,CV_64F);

  double x = euler.at<double>(0);
  double y = euler.at<double>(1);
  double z = euler.at<double>(2);

  // Assuming the angles are in radians.
  double ch = cos(z);
  double sh = sin(z);
  double ca = cos(y);
  double sa = sin(y);
  double cb = cos(x);
  double sb = sin(x);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh*sb - ch*sa*cb;
  m02 = ch*sa*sb + sh*cb;
  m10 = sa;
  m11 = ca*cb;
  m12 = -ca*sb;
  m20 = -sh*ca;
  m21 = sh*sa*cb + ch*sb;
  m22 = -sh*sa*sb + ch*cb;

  rotationMatrix.at<double>(0,0) = m00;
  rotationMatrix.at<double>(0,1) = m01;
  rotationMatrix.at<double>(0,2) = m02;
  rotationMatrix.at<double>(1,0) = m10;
  rotationMatrix.at<double>(1,1) = m11;
  rotationMatrix.at<double>(1,2) = m12;
  rotationMatrix.at<double>(2,0) = m20;
  rotationMatrix.at<double>(2,1) = m21;
  rotationMatrix.at<double>(2,2) = m22;

  return rotationMatrix;
}


void fillMeasurements( cv::Mat &measurements,
				   const cv::Mat &translation_measured, const cv::Mat &rotation_measured)
{
	// Convert rotation matrix to euler angles
	cv::Mat measured_eulers(3, 1, CV_64F);
	measured_eulers = rot2euler(rotation_measured);
	// Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); // x
	measurements.at<double>(1) = translation_measured.at<double>(1); // y
	measurements.at<double>(2) = translation_measured.at<double>(2); // z
	measurements.at<double>(3) = measured_eulers.at<double>(0);      // roll
	measurements.at<double>(4) = measured_eulers.at<double>(1);      // pitch
	measurements.at<double>(5) = measured_eulers.at<double>(2);      // yaw
}


void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
					 cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{
	// First predict, to update the internal statePre variable
	cv::Mat prediction = KF.predict();
	// The "correct" phase that is going to use the predicted value and our measurement
	cv::Mat estimated = KF.correct(measurement);
	// Estimated translation
	translation_estimated.at<double>(0) = estimated.at<double>(0);
	translation_estimated.at<double>(1) = estimated.at<double>(1);
	translation_estimated.at<double>(2) = estimated.at<double>(2);
	// Estimated euler angles
	cv::Mat eulers_estimated(3, 1, CV_64F);
	eulers_estimated.at<double>(0) = estimated.at<double>(9);
	eulers_estimated.at<double>(1) = estimated.at<double>(10);
	eulers_estimated.at<double>(2) = estimated.at<double>(11);
	// Convert estimated quaternion to rotation matrix
	rotation_estimated = euler2rot(eulers_estimated);
}


/**
 * @brief estimated_pose Returns estimated object pose in cartesian frame
 */
geometry_msgs::Pose kalman_pose_estimation(geometry_msgs::Pose previous_pose_, geometry_msgs::Pose measured_pose_) {

	// TODO: create kalman filter for each estimated object separatelly.
	cv::KalmanFilter KF;         // instantiate Kalman Filter
	int nStates = 18;            // the number of states
	int nMeasurements = 6;       // the number of measured states
	int nInputs = 0;             // the number of action control
	double dt = 0.125;           // time between measurements (1/FPS)
	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
	cv::Mat measurements(nMeasurements, 1, CV_64F);
	measurements.setTo(cv::Scalar(0));

	// Get object pose in sensor frame.
	tf::Transform measured_tf;
	tf::poseMsgToTF (measured_pose_ , measured_tf);
//	tf::btMatrix3x3 measured_rot = measured_tf.getBasis();

	// Get the measured translation
	cv::Mat translation_measured(3, 1, CV_64F);
	// translation_measured = ... TODO!
	// Get the measured rotation
	cv::Mat rotation_measured(3, 3, CV_64F);
	//rotation_measured = ... TODO!
	// fill the measurements vector
	fillMeasurements(measurements, translation_measured, rotation_measured);

	// Instantiate estimated translation and rotation
	cv::Mat translation_estimated(3, 1, CV_64F);
	cv::Mat rotation_estimated(3, 3, CV_64F);
	// update the Kalman filter with good measurements
	updateKalmanFilter( KF, measurements, translation_estimated, rotation_estimated);

	geometry_msgs::Pose estimated_pose;
	return estimated_pose;
}


/**
 * Listener responsible for acquiring and estimating poses of recognized objects in world reference frame.
 */
void recognizedObjectPoseUpdateCallback(const object_recognition_msgs::RecognizedObject& object_)
{
	ROS_DEBUG("I received: %s in %s reference frame ", object_.type.key.c_str(), object_.pose.header.frame_id.c_str());

	try{
		// Get world-sensor(camera) transformation.
		tf::StampedTransform world_sensor_tf;
		ROS_DEBUG("Getting transform from %s to %s", world_frame_id.c_str(), object_.pose.header.frame_id.c_str());
		// Throws exception that must be handled outside of the function.
		listener->lookupTransform(world_frame_id, object_.pose.header.frame_id, object_.pose.header.stamp, world_sensor_tf);


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



			// Update old object pose wrt to world coordinate frame.
			geometry_msgs::Pose tmp_msg_pose = sensorToWorlMsgPose(object_.pose.pose.pose, world_sensor_tf);

			// TODO: get kalman filter for a given object!
			//geometry_msgs::Pose estimated_pose = kalman_pose_estimation(estimated_objects[index].pose.pose.pose, tmp_msg_pose);
			// Compute pose update and modify world_sensor_tf - TODO!

			// Transform the object pose.
			estimated_objects[index].pose.pose.pose = tmp_msg_pose;


			// Transform the associated mesh pose.
			estimated_objects[index].bounding_mesh.vertices.clear();
			// Add transformed vertices one by one.
			for (size_t pt_i=0; pt_i < object_.bounding_mesh.vertices.size(); pt_i++) {
				estimated_objects[index].bounding_mesh.vertices.push_back(sensorToWorldMsgPoint(object_.bounding_mesh.vertices[pt_i], world_sensor_tf));
			}//: for

		} else {
			// If not - create new "object instance".
			ROS_DEBUG("New Object");

			// Set object pose wrt to world coordinate frame.
			geometry_msgs::Pose tmp_msg_pose = sensorToWorlMsgPose(object_.pose.pose.pose, world_sensor_tf);

			// Copy data.
			object_recognition_msgs::RecognizedObject tmp_object (object_);
			// Set adequate reference frames.
			tmp_object.header.frame_id = world_frame_id;
			tmp_object.pose.header.frame_id = world_frame_id;

			// Transform the object pose.
			tmp_object.pose.pose.pose = tmp_msg_pose;
			// Transform the associated mesh pose.
			tmp_object.bounding_mesh.vertices.clear();
			// Add transformed vertices one by one.
			for (size_t pt_i=0; pt_i < object_.bounding_mesh.vertices.size(); pt_i++) {
				tmp_object.bounding_mesh.vertices.push_back(sensorToWorldMsgPoint(object_.bounding_mesh.vertices[pt_i], world_sensor_tf));
			}//: for

			// Add object to list.
			estimated_objects.push_back(tmp_object);

			// TODO: create kalman filter for each estimated object separatelly.

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

	// Try to find object possesing given id and not being older than requested.
	for(ro_it_t ro_i=estimated_objects.begin(); ro_i != estimated_objects.end(); ro_i++) {
		// Check name.
		if (ro_i->type.key != req_.object_id)
			continue;

		// Check object age - if it is older than age - skip.
		if (ro_i->header.stamp < ros::Time::now() - req_.age)
			continue;
		// Ok, got the object - return it and finish.
		res_.object = *ro_i;
		res_.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::OBJECT_FOUND;
		ROS_INFO("Returning object %s (%f)", res_.object.type.key.c_str(), res_.object.confidence);
		return true;
	}//: for
	// Sorry, object was not found.
	res_.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::OBJECT_NOT_FOUND;
	ROS_INFO("Object %s not found on the list.", req_.object_id.c_str());
	return true;
}//: end


/// Service responsible for returning list of recognized object model_ids with with given object_id and not older that given age.
bool returnRecognizedObjectsListServiceCallback(
		irp6_grasping_msgs::GetRecognizedObjectsList::Request &req_,
		irp6_grasping_msgs::GetRecognizedObjectsList::Response &res_)
{
	// Special case: do not return anything if t here are no objects;)
	if (estimated_objects.size() == 0){
		ROS_DEBUG("No objects available on the list.");
		return true;
	}//: if

	// Iterate and add names to list - in proper order, sorted by confidence.
	for(ro_it_t ro_i=estimated_objects.begin(); ro_i != estimated_objects.end(); ro_i++) {
		// Limit the size of returned vector - 0 means that there is no limit.
		if ((req_.limit> 0)&&(res_.object_ids.size() >= req_.limit))
			break;

		// Check object age - if it is older than age - skip.
		if (ro_i->header.stamp < ros::Time::now() - req_.age)
			continue;

		// Fill oid data.
		irp6_grasping_msgs::ObjectID oid;
		oid.id = ro_i->type.key;
		oid.confidence = ro_i->confidence;

		// First case: insert first oid.
		if (res_.object_ids.size() == 0) {
			ROS_DEBUG("Special case! - wstawiam");
			res_.object_ids.push_back(oid);
			continue;
		}//: if

		// Second case: insert in proper order.
		bool added = false;
		for (oid_it_t oid_i=res_.object_ids.begin(); oid_i<res_.object_ids.end(); oid_i++) {
			ROS_DEBUG("Service - iterating: %f  < %f ?",oid_i->confidence, oid.confidence);

			if (oid_i->confidence < oid.confidence){
				// Insert here! (i.e. before)
				res_.object_ids.insert(oid_i, oid);
				added = true;
				break;
			}//: if
		}//: for*/

		// Third case: insert at the end.
		if (!added)
			res_.object_ids.push_back(oid);

	}//: for

	return true;
}//: end




int main(int argc, char **argv)
{
	// Initialize node.
	ros::init(argc, argv, "recognized_objects_pose_estimation");

	// Handle to communication ports.
	ros::NodeHandle node;

	// Read parameters
	// Name of the world coordinate frame.
	node.param<std::string>("world_frame_id", world_frame_id, "/tl_base");\

	// Max age of marker - afterwards it will not be displayed.
	node.param<double>("marker_max_age", marker_max_age, 5.0);

	// Initialize TF listener.
	listener = new tf::TransformListener();

	// Create subscriber to topic containing recognized objects.
	ros::Subscriber sub = node.subscribe("recognized_objects", 1000, recognizedObjectPoseUpdateCallback);

	// Initialize services.
	ros::ServiceServer service1 = node.advertiseService("return_recognized_object_pose", returnRecognizedObjectPoseServiceCallback);
	ros::ServiceServer service2 = node.advertiseService("return_recognized_objects_list", returnRecognizedObjectsListServiceCallback);

	// Initialize marker publisher.
	vis_marker_publisher = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

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
