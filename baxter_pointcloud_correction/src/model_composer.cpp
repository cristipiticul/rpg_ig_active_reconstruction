/**
 * Input: a folder name in the data folder, which contains point clouds
 * Output: the model made up by joining the input point clouds
 */
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <boost/filesystem.hpp>

#include <baxter_pointcloud_correction/SegmentPointCloud.h>

#define NODE_NAME "model_composer"
#define POINTCLOUD_SEGMENTATION_SERVICE_NAME "segment_pointcloud"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

const double VOXEL_GRID_LEAF_SIZE = 0.003;

void usage( char* program_name )
{
  std::cout << "Usage: " << program_name << " <folder-name>\n";
}

pcl::visualization::PCLVisualizer::Ptr p;

/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param final_transform the resultant transform source to target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, Eigen::Matrix4f &final_transform)
{
  // Downsample for consistency and speed
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointType> grid;

  grid.setLeafSize (VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE);
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
  reg.setMaximumIterations(2);
  reg.setInputSource (src);
  reg.setInputTarget (tgt);
  reg.align(*reg_result);
  final_transform = reg.getFinalTransformation();

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  pcl::visualization::PointCloudColorHandlerCustom<PointType> green(tgt, 0, 255, 0);
  p->addPointCloud<PointType> (tgt, green, "target");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> blue(reg_result, 0, 0, 255);
  p->addPointCloud<PointType> (reg_result, blue, "source");

  //PCL_INFO ("Press q to continue the registration.\n");
  //p->spin ();
  p->spinOnce();
 }

/**
 * Read the .pcd files in "folder_path" and saves them in a vector.
 * The files are read in increasing order of the filenames.
 */
void readPcdFiles(std::string folder_path, std::vector<PointCloud::Ptr>& clouds)
{
  boost::filesystem::directory_iterator end_it; // default construction yields past-the-end
  std::vector<std::string> file_paths;
  for( boost::filesystem::directory_iterator it(folder_path);
      it != end_it; ++it)
  {
    if( boost::filesystem::is_regular_file(it->status()) && boost::filesystem::extension(it->path()) == ".pcd" )
    {
      file_paths.push_back(it->path().string());
    }
  }

  std::sort(file_paths.begin(), file_paths.end());

  for( unsigned int i = 0; i < file_paths.size(); i++ )
  {
    PointCloud::Ptr cloud(new PointCloud());
    if( pcl::io::loadPCDFile(file_paths[i], *cloud) != 0)
    {
      ROS_ERROR("model_composer: could not read PCD file %s", file_paths[i].c_str());
      exit(-1);
    }
    clouds.push_back(cloud);
  }
}

void registerAllPointClouds( std::vector<PointCloud::Ptr>& clouds, std::vector<int> order );
std::vector<int> circularRange( int start, int minimum, int maximum, int step );

int main( int argc, char** argv )
{
  if( argc != 2 )
  {
    usage(argv[0]);
    return 0;
  }
  p.reset(new pcl::visualization::PCLVisualizer("registration"));
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  std::string folder_name = argv[1];
  std::string folder_path = ros::package::getPath("baxter_pointcloud_correction")
    + "/data/" + folder_name;

  ros::ServiceClient segmentation_service_client = nh.serviceClient<baxter_pointcloud_correction::SegmentPointCloud>(POINTCLOUD_SEGMENTATION_SERVICE_NAME);
  ROS_INFO("model_composer: waiting for service %s...", POINTCLOUD_SEGMENTATION_SERVICE_NAME);
  segmentation_service_client.waitForExistence();
  ROS_INFO("model_composer: done!");

  ROS_INFO("model_composer: folder path: %s", folder_path.c_str());

  std::vector<PointCloud::Ptr> clouds;
  readPcdFiles(folder_path, clouds);

  // Segmentation
  for( unsigned int i = 0; i < clouds.size(); i++ )
  {
    PointCloud::Ptr cloud = clouds[i];

    baxter_pointcloud_correction::SegmentPointCloud::Request request;
    pcl::toROSMsg(*cloud, request.cloud_in);
    baxter_pointcloud_correction::SegmentPointCloud::Response response;
    if( !segmentation_service_client.call(request, response) )
    {
      ROS_ERROR("model_composer: something went wrong when calling service %s", POINTCLOUD_SEGMENTATION_SERVICE_NAME);
      return -1;
    }
    cloud->clear();
    pcl::fromROSMsg(response.cloud_out, *cloud);
  }

  // TODO: Remove outliers

  // Registration and composition
  PointCloud::Ptr model(new PointCloud);
  pcl::visualization::PCLVisualizer visualizer("model");
  visualizer.addCoordinateSystem(0.1);
  const int nr_iterations = 10;
  std::vector<int> increasing_range = circularRange(0, 0, clouds.size() - 1, 1);
  std::vector<int> decreasing_range = circularRange(clouds.size() - 1, 0, clouds.size() - 1, -1);
  for( int i = 0; i < nr_iterations && ros::ok(); i++ )
  {
    ROS_INFO("Iteration %d/%d", i + 1, nr_iterations);
    ROS_INFO("Registration in increasing-order");
    registerAllPointClouds(clouds, increasing_range);

    model->clear();
    for( unsigned int i = 0; i < clouds.size(); i++ )
    {
      *model += *clouds[i];
    }
    if( !visualizer.updatePointCloud<PointType>(model, "model") )
    {
      visualizer.addPointCloud<PointType>(model, "model");
    }
    visualizer.spinOnce();

    ROS_INFO("Registration in decreasing-order");
    registerAllPointClouds(clouds, decreasing_range);

    model->clear();
    for( unsigned int i = 0; i < clouds.size(); i++ )
    {
      *model += *clouds[i];
    }
    if( !visualizer.updatePointCloud<PointType>(model, "model") )
    {
      visualizer.addPointCloud<PointType>(model, "model");
    }
    visualizer.spinOnce();
  }

  pcl::VoxelGrid<PointType> grid;
  PointCloud::Ptr model_downsampled(new PointCloud);
  grid.setLeafSize(VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE);
  grid.setInputCloud(model);
  grid.filter(*model_downsampled);
  visualizer.updatePointCloud<PointType>(model_downsampled, "model");
  visualizer.spin();

  std::string result_file_path = ros::package::getPath("baxter_pointcloud_correction")
    + "/data/" + folder_name + "_registered.pcd";
  pcl::io::savePCDFile(result_file_path, *model_downsampled);

  return 0;
}

std::vector<int> circularRange( int start, int minimum, int maximum, int step )
{
  if( step != 1 && step != -1 )
  {
    ROS_WARN("The model_composer::circularRange function was designed for step==1 or -1. It was called with step %d, which may cause unpredicted results", step);
  }
  std::vector<int> result;
  int i = start;
  do
  {
    result.push_back(i);
    if( i == minimum && step < 0 )
    {
      i = maximum;
    }
    else if( i == maximum && step > 0 )
    {
      i = minimum;
    }
    else
    {
      i = i + step;
    }
  }
  while( i != start );
  return result;
}

void registerAllPointClouds( std::vector<PointCloud::Ptr>& clouds, std::vector<int> order )
{
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
  std::vector<Eigen::Matrix4f> transforms;
  transforms.push_back(GlobalTransform);
  for( unsigned int i = 1; i < order.size(); i++ )
  {
    ROS_INFO("Registering cloud pair %u/%lu", i, order.size() - 1);
    PointCloud::Ptr cloud = clouds[order[i]];
    PointCloud::Ptr prev_cloud = clouds[order[i - 1]];

    Eigen::Matrix4f cloudToPrevCloud;
    pairAlign(cloud, prev_cloud, cloudToPrevCloud);

    //update the global transform
    GlobalTransform = GlobalTransform * cloudToPrevCloud;

    transforms.push_back(GlobalTransform);
  }
  for( unsigned int i = 1; i < order.size(); i++ )
  {
    PointCloud::Ptr& cloud = clouds[order[i]];
    pcl::visualization::PointCloudColorHandlerCustom<PointType> green(cloud, 0, 255, 0);

    //transform current pair into the global transform
    PointCloud::Ptr transformed(new PointCloud);
    pcl::transformPointCloud(*cloud, *transformed, transforms[i]);
    cloud = transformed;
  }
}
