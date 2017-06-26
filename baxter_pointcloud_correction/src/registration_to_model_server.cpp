/**
 * Reads a model file from the "data/" folder.
 * Provides a service server. The request contains a pointcloud,
 * and the response contains the registered pointcloud.
 *
 * The input pointcloud will be transformed in the model's
 * frame, registered, and then transformed back to the camera frame.
 */
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/io/io.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_listener.h>

#include "baxter_pointcloud_correction/RegisterPointCloud.h"
#include "baxter_pointcloud_correction/SegmentPointCloud.h"

#define NODE_NAME "registration_to_model_server"
#define POINTCLOUD_REGISTRATION_SERVICE_NAME "register_pointcloud"
#define POINTCLOUD_SEGMENTATION_SERVICE_NAME "segment_pointcloud"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;


/*
 * TODO: add dynamic reconfigure.
 */

class PointCloudRegistrationServer
{
public:
  typedef baxter_pointcloud_correction::RegisterPointCloudRequest Request;
  typedef baxter_pointcloud_correction::RegisterPointCloudResponse Response;
  PointCloudRegistrationServer( std::string path_to_model, std::string model_frame );
  bool register_pointcloud( Request& req, Response& res );
protected:
  bool getTransform( const std::string target_frame, const std::string source_frame, ros::Time time, tf::StampedTransform &result );
  bool segmentCloud( PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_segmented );
  void pairAlign( const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform );
private:
  ros::NodeHandle nh_;

  ros::ServiceServer point_cloud_registration_server_;

  std::string model_frame_;
  PointCloud::Ptr model_;

  tf::TransformListener transform_listener_;

  ros::ServiceClient segmentation_service_client_;

  pcl::visualization::PCLVisualizer visualizer_;
  int visualizer_viewport_1_;
  int visualizer_viewport_2_;
};

void usage()
{
  std::cout << "Usage: rosrun baxter_pointcloud_correction registration_to_model_server <path-to-model-file> <model-frame>\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  if( argc != 3 )
  {
    usage();
    return 0;
  }
  PointCloudRegistrationServer server(argv[1], argv[2]);

  ROS_INFO("PointCloudSegmentationServer: ready to recieve service calls on %s", POINTCLOUD_REGISTRATION_SERVICE_NAME);

  ros::spin();
  return 0;
}

PointCloudRegistrationServer::PointCloudRegistrationServer( std::string path_to_model, std::string model_frame )
  : nh_()
  , model_(new PointCloud)
  , model_frame_(model_frame)
  , transform_listener_(nh_)
  , visualizer_("Registration")
{
  point_cloud_registration_server_ = nh_.advertiseService(POINTCLOUD_REGISTRATION_SERVICE_NAME, &PointCloudRegistrationServer::register_pointcloud, this);

  segmentation_service_client_ = nh_.serviceClient<baxter_pointcloud_correction::SegmentPointCloud>(POINTCLOUD_SEGMENTATION_SERVICE_NAME);
  ROS_INFO("Waiting for service %s...", POINTCLOUD_SEGMENTATION_SERVICE_NAME);
  segmentation_service_client_.waitForExistence();
  ROS_INFO("Done!");

  if( pcl::io::loadPCDFile(path_to_model, *model_) != 0)
  {
    ROS_ERROR("PointCloudRegistrationServer: Could not read the %s file!", path_to_model.c_str());
  }

  visualizer_.createViewPort(0.0, 0.0, 0.5, 1.0, visualizer_viewport_1_);
  visualizer_.createViewPort(0.5, 0.0, 1.0, 1.0, visualizer_viewport_2_);

  ROS_INFO("PointCloudRegistrationService: Service %s is ready for calls", POINTCLOUD_REGISTRATION_SERVICE_NAME);
}

bool PointCloudRegistrationServer::register_pointcloud( Request& req, Response& res )
{
  ROS_INFO("PointCloudRegistrationService: received service call!");
  PointCloud::Ptr cloud(new PointCloud);
  PointCloud::Ptr cloud_in_model_frame(new PointCloud);
  PointCloud::Ptr cloud_segmented(new PointCloud);
  PointCloud::Ptr cloud_registered_in_model_frame(new PointCloud);
  //PointCloud::Ptr cloud_registered(new PointCloud);
  pcl::fromROSMsg(req.cloud_in, *cloud);

  pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color_handler(cloud, 0, 255, 0);
  if( !visualizer_.updatePointCloud(cloud, green_color_handler, "input_cloud") )
  {
    visualizer_.addPointCloud(cloud, green_color_handler, "input_cloud", visualizer_viewport_1_);
  }

  tf::StampedTransform camera_to_model_tf;
  if( !getTransform(model_frame_, cloud->header.frame_id, req.cloud_in.header.stamp, camera_to_model_tf) )
  {
    return false;
  }
  pcl_ros::transformPointCloud(*cloud, *cloud_in_model_frame, camera_to_model_tf);

  if( !segmentCloud(cloud_in_model_frame, cloud_segmented) )
  {
    ROS_WARN("PointCloudRegistrationService: The segmentation service returned an error!");
    return false;
  }

  Eigen::Matrix4f registration_transform;
  pairAlign(cloud_segmented, model_, registration_transform);
  pcl::transformPointCloud(*cloud_in_model_frame, *cloud_registered_in_model_frame, registration_transform);
  cloud_registered_in_model_frame->header.frame_id = model_frame_;
  //pcl_ros::transformPointCloud(*cloud_registered_in_model_frame, *cloud_registered, camera_to_model_tf.inverse());

  pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color_handler(cloud_registered_in_model_frame, 0, 0, 255);
  if( !visualizer_.updatePointCloud(cloud_registered_in_model_frame, blue_color_handler, "cloud_registered") )
  {
    visualizer_.addPointCloud(cloud_registered_in_model_frame, blue_color_handler, "cloud_registered", visualizer_viewport_1_);
  }

  visualizer_.spinOnce();
  ROS_INFO("Cloud registered frame name: %s", cloud_registered_in_model_frame->header.frame_id.c_str());
  pcl::toROSMsg(*cloud_registered_in_model_frame, res.cloud_out);
  ROS_INFO("PointCloudRegistrationService: service call finished successfully");
  return true;
}


bool PointCloudRegistrationServer::getTransform(const std::string target_frame, const std::string source_frame, ros::Time time, tf::StampedTransform &result)
{
  bool success = false;
  tf::StampedTransform transform;
  try
  {
    transform_listener_.waitForTransform(target_frame, source_frame, time, ros::Duration(1.0));
    transform_listener_.lookupTransform(target_frame, source_frame, time, result);

    success = true;
  }
  catch (tf::TransformException &t)
  {
    ROS_WARN(
        "PointCloudRegistrationServer: Could not read transform between frames \"%s\" and \"%s\"",
        source_frame.c_str(), target_frame.c_str());
  }
  return success;
}

/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param final_transform the resultant transform source to target
  */
void PointCloudRegistrationServer::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform)
{
  // Downsample for consistency and speed
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointType> grid;

  grid.setLeafSize (0.005, 0.005, 0.005);
  grid.setInputCloud (cloud_src);
  grid.filter (*src);

  grid.setInputCloud (cloud_tgt);
  grid.filter (*tgt);

  // Align
  pcl::IterativeClosestPointNonLinear<PointType, PointType> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 1cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.01);

  PointCloud::Ptr reg_result(new PointCloud);
  reg.setMaximumIterations(200);
  reg.setInputSource (src);
  reg.setInputTarget (tgt);
  reg.align(*reg_result);
  final_transform = reg.getFinalTransformation();

  visualizer_.removePointCloud ("source", visualizer_viewport_2_);
  visualizer_.removePointCloud ("target", visualizer_viewport_2_);

  pcl::visualization::PointCloudColorHandlerCustom<PointType> green_color_handler(tgt, 0, 255, 0);
  visualizer_.addPointCloud<PointType> (tgt, green_color_handler, "target", visualizer_viewport_2_);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> blue_color_handler(reg_result, 0, 0, 255);
  visualizer_.addPointCloud<PointType> (reg_result, blue_color_handler, "source", visualizer_viewport_2_);
 }

bool PointCloudRegistrationServer::segmentCloud( PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_segmented )
{
  baxter_pointcloud_correction::SegmentPointCloud::Request req;
  baxter_pointcloud_correction::SegmentPointCloud::Response res;
  pcl::toROSMsg(*cloud_in, req.cloud_in);
  if( !segmentation_service_client_.call(req, res) )
  {
    ROS_ERROR("PointCloudRegistrationServer: Call to segmentation service failed!");
    return false;
  }
  pcl::fromROSMsg(res.cloud_out, *cloud_segmented);
  return true;
}
