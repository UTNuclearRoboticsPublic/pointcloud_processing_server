
#ifndef POINTCLOUD_PROCESSING_H
#define POINTCLOUD_PROCESSING_H

#include <ros/ros.h>
#include <shape_msgs/Plane.h>
#include "pointcloud_processing_server/pointcloud_process.h"
#include "pointcloud_processing_server/pointcloud_task_creation.h"
#include "pointcloud_processing_server/pointcloud_utilities.h"

#define _USE_MATH_DEFINES

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>

#include <cmath>

//usefule PCL typedefs
//typedef pcl::PointXYZRGB PCLPoint;
//typedef pcl::PointXYZRGBNormal PCLPointNormal;
//typedef pcl::PointCloud<PCLPoint> PC;
//typedef pcl::PointCloud<PCLPoint>::Ptr PCP;
//typedef std::vector<pcl::PointIndices> IndexVector;

template <typename PointType> 
class PointcloudProcessing
{ 
public:
  //usefule PCL typedefs
  typedef typename pcl::PointCloud<PointType> PC;
  typedef typename pcl::PointCloud<PointType>::Ptr PCP;

	PointcloudProcessing();
	bool getShutdownStatus();
	bool pointcloud_service(pointcloud_processing_server::pointcloud_process::Request &req, pointcloud_processing_server::pointcloud_process::Response &res);
	pointcloud_processing_server::pointcloud_task_result transformPointCloud(PCP &input, std::string transform_to_frame, std::string starting_frame);
  pointcloud_processing_server::pointcloud_task_result clipPointCloud(PCP &unclipped, std::vector<float> box_data, std::vector<float> box_pose, bool keep_ordered);
	pointcloud_processing_server::pointcloud_task_result clipPointCloudConditional(PCP &unclipped, std::vector<float> ranges, bool keep_ordered);
	pointcloud_processing_server::pointcloud_task_result voxelizePointCloud(PCP &unvoxelized, std::vector<float> voxel_leaf_size);
	pointcloud_processing_server::pointcloud_task_result segmentTowardPlane(PCP &input, int max_iterations, float threshold_distance, bool remove_cloud);
	pointcloud_processing_server::pointcloud_task_result segmentTowardCylinder(PCP &input, int max_iterations, float threshold_distance, float max_radius, bool remove_cloud);
  pointcloud_processing_server::pointcloud_task_result segmentTowardLine(PCP &input, int max_iterations, float threshold_distance, bool remove_cloud);
  pointcloud_processing_server::pointcloud_task_result radiusOutlierFilter(PCP &unfiltered, float search_radius, int min_neighbors, bool keep_ordered);
  pointcloud_processing_server::pointcloud_task_result statisticalOutlierFilter(PCP &unfiltered, int k_min, float std_mul, bool keep_ordered);
  bool checkMinSize(int cloud_size, int min_num_points, std::string task_name);

protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.

  tf::TransformListener camera_frame_listener_;

  PCP input_cloud_;
  std::string current_cloud_frame_;  // used to retain frame information (which isn't stored in pcl types)

  int max_segmentation_iterations;
  int segmentation_distance_threshold; 
  bool should_shut_down;

  int task_counts_[8];              // For some reason decided to make indices start at 1, not 0. 

};

#endif //POINTCLOUD_PROCESSING_H