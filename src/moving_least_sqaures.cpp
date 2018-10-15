

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_representation.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

template<typename PointType> void
movingLeastSquares(const pcl::PointCloud<PointType> input_cloud, pcl::PointCloud<PointType>& output_cloud, float filter_radius, int )