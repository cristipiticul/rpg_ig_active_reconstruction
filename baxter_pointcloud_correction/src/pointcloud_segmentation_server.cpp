/**
 * Service server:
 *  -> input:  cloud_in: point cloud to be segmented
 *  -> output: cloud_out: segmented point cloud
 *
 * Segmentation parameters are set using dynamic_reconfigure.
 */
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include "baxter_pointcloud_correction/SegmentPointCloud.h"

#define NODE_NAME "pointcloud_segmentation_server"
#define POINTCLOUD_SEGMENTATION_SERVICE_NAME "segment_pointcloud"

/*
 * TODO: add dynamic reconfigure.
 */

class PointCloudSegmentationServer
{
public:
  typedef baxter_pointcloud_correction::SegmentPointCloudRequest Request;
  typedef baxter_pointcloud_correction::SegmentPointCloudResponse Response;
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;
  PointCloudSegmentationServer();
  bool segment( Request& req, Response& res );
private:
  ros::NodeHandle nh_;
  ros::ServiceServer point_cloud_segmentation_server_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  PointCloudSegmentationServer server;

  ROS_INFO("PointCloudSegmentationServer: ready to recieve service calls on %s", POINTCLOUD_SEGMENTATION_SERVICE_NAME);

  ros::spin();
  return 0;
}

PointCloudSegmentationServer::PointCloudSegmentationServer()
  : nh_()
{
  point_cloud_segmentation_server_ = nh_.advertiseService(POINTCLOUD_SEGMENTATION_SERVICE_NAME, &PointCloudSegmentationServer::segment, this);
}

bool PointCloudSegmentationServer::segment( Request& req, Response& res )
{
  PointCloud::Ptr cloud(new PointCloud());
  pcl::fromROSMsg(req.cloud_in, *cloud);

  double x_min = -0.1;
  double x_max = 0.1;
  double y_min = -0.1;
  double y_max = 0.1;
  double z_min = -0.1;
  double z_max = 0.02;

  PointCloud::Ptr cloud_out(new PointCloud());
  cloud_out->header = cloud->header;
  cloud_out->is_dense = false;
  for( unsigned int i = 0; i < cloud->size(); i++ )
  {
    PointType& p = cloud->at(i);
    if( pcl_isfinite(p.x) && pcl_isfinite(p.y) && pcl_isfinite(p.z)
        && p.x > x_min && p.x < x_max
        && p.y > y_min && p.y < y_max
        && p.z > z_min && p.z < z_max)
    {
      cloud_out->push_back(p);
    }
  }

  pcl::toROSMsg(*cloud_out, res.cloud_out);
  return true;
}
