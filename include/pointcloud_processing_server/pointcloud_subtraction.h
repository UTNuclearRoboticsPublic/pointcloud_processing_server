
#ifndef POINTCLOUD_SUBTRACTION_H
#define POINTCLOUD_SUBTRACTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>

namespace PointcloudSubtraction
{
    sensor_msgs::PointCloud2 subtractClouds(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_intensity=true, bool check_rgb=true, bool check_normals=true);
	template <typename PointType> sensor_msgs::PointCloud2 templatedSubtraction(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_intensity, bool check_rgb, bool check_normals);
	
	template<typename PointType> 
	class PointcloudSubtractor
	{
		typedef typename pcl::PointCloud<PointType>             PC;
	    typedef typename pcl::PointCloud<PointType>::Ptr        PCP;
	    typedef typename pcl::KdTreeFLANN<PointType>            KDTree; 
	
	public:
	    PointcloudSubtractor();
		sensor_msgs::PointCloud2 subtractClouds(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_intensity, bool check_rgb, bool check_normals);
		PCP subtractClouds(PCP minuend, PCP subtrahend, bool check_intensity, bool check_rgb, bool check_normals);
		bool comparePoints(PointType minuend_point, PointType subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals);
	};	
}

#endif //POINTCLOUD_SUBTRACTION_H