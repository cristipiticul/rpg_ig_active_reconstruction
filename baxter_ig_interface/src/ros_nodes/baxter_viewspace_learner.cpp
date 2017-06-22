/**
 * This node reads data from TF and sends the poses to the viewspace module.
 * Relevant information that will be saved:
 *   - pose of camera, in the object-grasping end-effector frame
 *       -> this is saved in the View::pose_ field
 *   - pose of camera, in the base frame
 *       -> saved in View::additional_fields_names_ & values_ (with prefix "cam_")
 *   - pose of the object-grasping end-effector, in the base frame
 *       -> saved in View::additional_fields_names_ & values_ (with prefix "obj_")
 *
 * Services:
 *   - read arm (left or right) TF relative to base
 *   - send (send all frames to the viewspace module)
 *
 * If we have N poses for the camera and M poses for the object end-effector,
 * there will be N * M viewpoints.
 */
#include <ros/ros.h>
#include <ros/service_server.h>

#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

#include <ig_active_reconstruction_msgs/ViewSpaceUpdate.h>

#define NODE_NAME "baxter_viewspace_learner"

#define ADD_VIEWS_SERVICE_NAME "views/add"

static void addPoseToViewAdditionalFields(const geometry_msgs::Pose pose, const std::string name, ig_active_reconstruction_msgs::ViewMsg &view_msg)
{
  {
    view_msg.associated_names.push_back(name + "/position/x");
    view_msg.associated_values.push_back(pose.position.x);

    view_msg.associated_names.push_back(name + "/position/y");
    view_msg.associated_values.push_back(pose.position.y);

    view_msg.associated_names.push_back(name + "/position/z");
    view_msg.associated_values.push_back(pose.position.z);
  }
  {
    view_msg.associated_names.push_back(name + "/orientation/x");
    view_msg.associated_values.push_back(pose.orientation.x);

    view_msg.associated_names.push_back(name + "/orientation/y");
    view_msg.associated_values.push_back(pose.orientation.y);

    view_msg.associated_names.push_back(name + "/orientation/z");
    view_msg.associated_values.push_back(pose.orientation.z);

    view_msg.associated_names.push_back(name + "/orientation/w");
    view_msg.associated_values.push_back(pose.orientation.w);
  }
}

class BaxterViewspaceLearner
{
public:
  BaxterViewspaceLearner( std::string base_frame_name, std::string camera_frame_name, std::string object_eef_frame_name );
  bool readCameraTF( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
  bool readObjectEefTF( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
  bool sendToViewspaceModule( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
protected:
  bool getTransform( std::string target_frame, std::string source_frame, tf::StampedTransform& result );
private:
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;

  std::string base_frame_name_;
  std::string camera_frame_name_;
  std::string object_eef_frame_name_;

  std::vector<tf::Transform> camera_transforms_;
  std::vector<tf::Transform> object_eef_transforms_;

  ros::ServiceServer read_camera_tf_service_;
  ros::ServiceServer read_object_eef_tf_service_;
  ros::ServiceServer send_to_viewspace_module_service_;

  ros::ServiceClient update_viewspace_client_;
};

BaxterViewspaceLearner::BaxterViewspaceLearner(std::string base_frame_name, std::string camera_frame_name, std::string object_eef_frame_name)
  : nh_()
  , tf_listener_(nh_)
  , base_frame_name_(base_frame_name)
  , camera_frame_name_(camera_frame_name)
  , object_eef_frame_name_(object_eef_frame_name)
{
  read_camera_tf_service_ = nh_.advertiseService("read_camera_tf", &BaxterViewspaceLearner::readCameraTF, this);
  read_object_eef_tf_service_ = nh_.advertiseService("read_object_eef_tf", &BaxterViewspaceLearner::readObjectEefTF, this);
  send_to_viewspace_module_service_ = nh_.advertiseService("send_to_viewspace_module", &BaxterViewspaceLearner::sendToViewspaceModule, this);

  update_viewspace_client_ = nh_.serviceClient<ig_active_reconstruction_msgs::ViewSpaceUpdate>(ADD_VIEWS_SERVICE_NAME);
  ROS_INFO("baxter_ig_interface::BaxterViewspaceLearner: Waiting for %s service...", update_viewspace_client_.getService().c_str());
  update_viewspace_client_.waitForExistence();
  ROS_INFO("baxter_ig_interface::BaxterViewspaceLearner: Done!");
}

bool BaxterViewspaceLearner::readCameraTF( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  tf::StampedTransform transform;
  if( getTransform(base_frame_name_, camera_frame_name_, transform) )
  {
    camera_transforms_.push_back(transform);
  }
  return true;
}

bool BaxterViewspaceLearner::readObjectEefTF( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  tf::StampedTransform transform;
  if( getTransform(base_frame_name_, object_eef_frame_name_, transform) )
  {
    object_eef_transforms_.push_back(transform);
  }
  return true;
}

bool BaxterViewspaceLearner::getTransform( std::string target_frame, std::string source_frame, tf::StampedTransform& result )
{
  bool success = false;
  ros::Time now = ros::Time::now();
  static const ros::Duration timeout = ros::Duration(0.5);

  try
  {
    tf_listener_.waitForTransform(target_frame, source_frame, now, timeout);
    tf_listener_.lookupTransform(target_frame, source_frame, now, result);
    success = true;
  }
  catch (tf::TransformException &e)
  {
    ROS_WARN("baxter_ig_interface::BaxterViewspaceLearner: Could not read TF %s -> %s", source_frame.c_str(), target_frame.c_str());
  }

  return success;
}

bool BaxterViewspaceLearner::sendToViewspaceModule( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
  ig_active_reconstruction_msgs::ViewSpaceUpdateRequest request;
  unsigned int view_id = 0;

  for( tf::Transform camera_tf : camera_transforms_ )
  {
    for( tf::Transform object_eef_tf : object_eef_transforms_ )
    {
      tf::Transform viewpoint = object_eef_tf.inverse() * camera_tf;

      ig_active_reconstruction_msgs::ViewMsg view_msg;
      view_msg.index = view_id; // If the id is not set, the messages
      view_id++;
      view_msg.is_bad = false;
      view_msg.is_reachable = true;
      view_msg.source_frame = object_eef_frame_name_;

      tf::poseTFToMsg(viewpoint, view_msg.pose);

      geometry_msgs::Pose camera_pose;
      tf::poseTFToMsg(camera_tf, camera_pose);
      addPoseToViewAdditionalFields(camera_pose, "camera", view_msg);

      geometry_msgs::Pose object_eef_pose;
      tf::poseTFToMsg(object_eef_tf, object_eef_pose);
      addPoseToViewAdditionalFields(object_eef_pose, "object_eef", view_msg);

      request.views.push_back(view_msg);
    }
  }
  ROS_INFO("baxter_ig_interface::BaxterViewspaceLearner: Sending add views request...");
  ig_active_reconstruction_msgs::ViewSpaceUpdateResponse response;
  update_viewspace_client_.call(request, response);
  ROS_INFO("baxter_ig_interface::BaxterViewspaceLearner: Done!");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);
  // TODO: get these from PARAM
  std::string robot_base_frame = "/base";
  std::string camera_optical_frame = "/camera_rgb_optical_frame";
  std::string object_eef_frame = "/left_gripper";
  BaxterViewspaceLearner baxter_viewspace_learner(robot_base_frame, camera_optical_frame, object_eef_frame);
  ros::spin();
  return 0;
}
