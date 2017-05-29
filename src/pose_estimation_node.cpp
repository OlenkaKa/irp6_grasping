/*!
 * \file PoseEstimation.cpp
 * \author Aleksandra Karbarczyk
 * \note based on pose_estimation.cpp
 */

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <irp6_grasping/pose_estimation_node.h>

using namespace irp6_grasping;
using namespace std;
using namespace cv;

void PoseEstimationNode::start(ros::NodeHandle &nh)
{
  // Read parameters
  std::string object_id;
  bool use_kalman_filter;
  double min_object_confidence;
  std::string result_root_dir;

  nh.param<string>("world_frame_id", world_frame_id, "/tl_base");
  nh.param<string>("object_id", object_id, "herbapol_mieta1");
  nh.param<double>("min_object_confidence", min_object_confidence, 0.5);
  nh.param<string>("result_root_dir", result_root_dir, "~");

  object_model_.setId(object_id);

  object_pose_estimator_ = new ObjectPoseEstimator(min_object_confidence);

  result_writer_ = new ResultFileWriter("pose_estimation", result_root_dir);

  // Initialize services.
  ros::ServiceServer estimate_pose =
      nh.advertiseService<irp6_grasping_msgs::EstimatePose::Request, irp6_grasping_msgs::EstimatePose::Response>(
          "estimate_pose", boost::bind(&PoseEstimationNode::estimatePoseCallback, this, _1, _2, boost::ref(nh)));

  ros::ServiceServer object_pose_service = nh.advertiseService(
      "return_recognized_object_pose", &PoseEstimationNode::returnRecognizedObjectPoseServiceCallback, this);

  ros::ServiceServer object_list_service = nh.advertiseService(
      "return_recognized_objects_list", &PoseEstimationNode::returnRecognizedObjectsListServiceCallback, this);

  // Initialize marker publisher.
  marker_publisher_ = new ObjectMarkerPublisher(nh, world_frame_id);

  recognized_object_pub = nh.advertise<object_recognition_msgs::RecognizedObject>("global_recognized_objects", 0);

  ROS_INFO("Ready for recognized object pose update and estimation.");

  ros::Rate r(10);
  while (ros::ok())
  {
    publishRecognizedObjects();
    ros::spinOnce();
    r.sleep();
  }
  finish();
}

void PoseEstimationNode::finish()
{
  delete object_pose_estimator_;
  delete marker_publisher_;
  delete result_writer_;
}

object_recognition_msgs::RecognizedObject PoseEstimationNode::createRecognizedObject()
{
  object_recognition_msgs::RecognizedObject object;
  object.header.frame_id = world_frame_id;
  object.pose.header.frame_id = world_frame_id;
  object_model_.addData(object);
  object_pose_estimator_->addData(view_id_, object);
  return object;
}

geometry_msgs::Pose PoseEstimationNode::transformPose(const geometry_msgs::Pose &start_pose,
                                                      const tf::StampedTransform &end_tf)
{
  tf::Transform start_tf;
  tf::poseMsgToTF(start_pose, start_tf);
  tf::Transform start_end_tf = end_tf * start_tf;

  geometry_msgs::Pose transformed_pose;
  tf::poseTFToMsg(start_end_tf, transformed_pose);
  return transformed_pose;
}

geometry_msgs::Point PoseEstimationNode::transformPoint(const geometry_msgs::Point &point, const tf::Transform tf)
{
  tf::Point initial_tf_point(point.x, point.y, point.z);
  tf::Point result_tf_point = tf * initial_tf_point;

  geometry_msgs::Point result_point;
  result_point.x = result_tf_point.getX();
  result_point.y = result_tf_point.getY();
  result_point.z = result_tf_point.getZ();
  return result_point;
}

void PoseEstimationNode::recognizedObjectCallback(const object_recognition_msgs::RecognizedObject &sensor_object)
{
  static tf::TransformListener tf_listener;
  static tf::StampedTransform world_sensor_tf;

  ROS_DEBUG("I received: %s in %s reference frame.", sensor_object.type.key.c_str(),
            sensor_object.pose.header.frame_id.c_str());

  try
  {
    ROS_DEBUG("Getting transform from %s to %s", world_frame_id.c_str(), sensor_object.pose.header.frame_id.c_str());
    // TODO synchronize clock on gerwazy!
    tf_listener.lookupTransform(world_frame_id, sensor_object.pose.header.frame_id,
                                /*sensor_object.pose.header.stamp*/ ros::Time(0), world_sensor_tf);

    geometry_msgs::Pose sensor_object_world_pose = transformPose(sensor_object.pose.pose.pose, world_sensor_tf);

    object_model_.setBoundingMesh(sensor_object.bounding_mesh);
    PoseData pose_data;
    object_pose_estimator_->estimatePose(view_id_, sensor_object.pose.header.stamp, sensor_object_world_pose,
                                        sensor_object.confidence, pose_data);
    result_writer_->writePoseData(pose_data);
  }
  catch (tf::TransformException &e)
  {
    ROS_ERROR("%s %s", e.what(), world_frame_id.c_str());
  }
}

void PoseEstimationNode::publishRecognizedObjects()
{
  static tf::TransformBroadcaster br;

  if (!object_pose_estimator_->canEstimatePose())
    return;

  object_recognition_msgs::RecognizedObject estimated_object = createRecognizedObject();

  marker_publisher_->deleteMarkers();
  ROS_DEBUG("Adding transform from %s to %s to broadcasted TF message", estimated_object.header.frame_id.c_str(),
            estimated_object.type.key.c_str());
  ROS_DEBUG("Transform %f %f %f ", estimated_object.pose.pose.pose.position.x,
            estimated_object.pose.pose.pose.position.y, estimated_object.pose.pose.pose.position.z);

  tf::Transform transform;
  tf::poseMsgToTF(estimated_object.pose.pose.pose, transform);

  tf::StampedTransform stamped_transform(transform, estimated_object.pose.header.stamp,
                                         estimated_object.header.frame_id, estimated_object.type.key);

  br.sendTransform(stamped_transform);
  marker_publisher_->createMarkers(estimated_object);

  recognized_object_pub.publish(estimated_object);
}

bool PoseEstimationNode::estimatePoseCallback(irp6_grasping_msgs::EstimatePose::Request &request,
                                              irp6_grasping_msgs::EstimatePose::Response &response,
                                              ros::NodeHandle &node_handle)
{
  if (request.calculate_pose)
  {
    view_id_ = request.view_id;
    recognized_object_sub =
        node_handle.subscribe("recognized_objects", 1000, &PoseEstimationNode::recognizedObjectCallback, this);
    ROS_INFO("Start pose estimation");
    //        ROS_INFO("Start pose estimation for view id: %d", view_id);
  }
  else
  {
    recognized_object_sub.shutdown();
    ROS_INFO("Stop pose estimation");
  }
  return true;
}

bool PoseEstimationNode::returnRecognizedObjectPoseServiceCallback(
    irp6_grasping_msgs::GetRecognizedObjectPose::Request &request,
    irp6_grasping_msgs::GetRecognizedObjectPose::Response &response)
{
  if (!object_pose_estimator_->canEstimatePose())
  {
    response.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::NO_OBJECTS;
    ROS_INFO("No object available.");
    return true;
  }

  if (object_model_.getId().compare(request.object_id) != 0 ||
      object_pose_estimator_->getLastPoseEstimationTime() < ros::Time::now() - request.age)
  {
    response.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::OBJECT_NOT_FOUND;
    ROS_INFO("Object %s not found.", request.object_id.c_str());
    return true;
  }

  object_recognition_msgs::RecognizedObject estimated_object = createRecognizedObject();
  response.object = estimated_object;
  response.status = irp6_grasping_msgs::GetRecognizedObjectPose::Response::OBJECT_FOUND;
  ROS_INFO("Returning object %s (%f).", response.object.type.key.c_str(), response.object.confidence);
  return true;
}

bool PoseEstimationNode::returnRecognizedObjectsListServiceCallback(
    irp6_grasping_msgs::GetRecognizedObjectsList::Request &request,
    irp6_grasping_msgs::GetRecognizedObjectsList::Response &response)
{
  if (!object_pose_estimator_->canEstimatePose() ||
      object_pose_estimator_->getLastPoseEstimationTime() < ros::Time::now() - request.age)
  {
    ROS_INFO("No object available.");
    return true;
  }

  // Limit the size of returned vector - 0 means that there is no limit.
  if ((request.limit > 0) && (response.object_ids.size() >= request.limit))
  {
    ROS_INFO("Cannot insert object due to limit.");
    return true;
  }

  object_recognition_msgs::RecognizedObject estimated_object = createRecognizedObject();

  // Fill oid data.
  irp6_grasping_msgs::ObjectID oid;
  oid.id = estimated_object.type.key;
  oid.confidence = estimated_object.confidence;

  // First case: insert first oid.
  if (response.object_ids.size() == 0)
  {
    response.object_ids.push_back(oid);
  }
  else
  {
    // Second case: insert in proper order.
    bool added = false;
    for (vector<irp6_grasping_msgs::ObjectID>::iterator it = response.object_ids.begin(),
                                                        end_it = response.object_ids.end();
         it != end_it; ++it)
    {
      ROS_DEBUG("Service - iterating: %f < %f ?", it->confidence, oid.confidence);
      if (it->confidence < oid.confidence)
      {
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recognized_objects_pose_estimation");
  ros::NodeHandle nh;
  PoseEstimationNode node;
  node.start(nh);
  return 0;
}
