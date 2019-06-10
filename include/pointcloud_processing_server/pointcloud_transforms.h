
#ifndef POINTCLOUD_TRANSFORMS_H
#define POINTCLOUD_TRANSFORMS_H

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h> 			// fromROSMsg(), toROSMsg() 
#include <Eigen/Dense>

#include "pointcloud_processing_server/server.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>


namespace PointcloudUtilities
{
	Eigen::Quaternion<float> quaternionFromVectors(Eigen::Vector3f first_vec, Eigen::Vector3f second_vec);
	void doTransform(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &transformed_cloud, Eigen::Matrix4f homogeneous);
	sensor_msgs::PointCloud2 translateCloud(sensor_msgs::PointCloud2 input_cloud, Eigen::Vector3f translation);
	void inverseTransform(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &transformed_cloud, Eigen::Matrix4f homogeneous);
	sensor_msgs::PointCloud2 rotateCloud(sensor_msgs::PointCloud2 input_cloud, Eigen::Quaternion<float> quaternion);
	sensor_msgs::PointCloud2 rotateCloud(sensor_msgs::PointCloud2 input_cloud, geometry_msgs::Quaternion quaternion);
	sensor_msgs::PointCloud2 rotatePlaneToXZ(sensor_msgs::PointCloud2 input_plane, std::vector<float> coefficients);
	sensor_msgs::PointCloud2 rotatePlaneToXZ(sensor_msgs::PointCloud2 input_cloud, std::vector<float> coefficients, Eigen::Matrix4f &transform);
	sensor_msgs::PointCloud2 translatePlaneToXZ(sensor_msgs::PointCloud2 input_cloud, Eigen::Matrix4f &transform);
	sensor_msgs::PointCloud2 translatePlaneToXZ(sensor_msgs::PointCloud2 input_plane);
	
	float minValue(sensor_msgs::PointCloud2 input_cloud, char field_name);
	float maxValue(sensor_msgs::PointCloud2 input_cloud, char field_name);
	float meanValue(sensor_msgs::PointCloud2 input_cloud, char field_name);
	float * cloudLimits(sensor_msgs::PointCloud2 input_cloud);
	void cloudLimits(sensor_msgs::PointCloud2 input_cloud, float* min_x, float* max_x, float* min_y, float* max_y, float* min_z, float* max_z);
	void cloudLimits(sensor_msgs::PointCloud2 input_cloud, float* min_x, float* max_x, float* min_y, float* max_y, float* min_z, float* max_z, float* min_intensity, float* max_intensity);

	geometry_msgs::Transform transformMultiplication(geometry_msgs::Transform first_transform, geometry_msgs::Transform second_transform);
}




#endif // POINTCLOUD_TRANSFORMS_H