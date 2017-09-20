
#ifndef POINTCLOUD_UTILITIES_H
#define POINTCLOUD_UTILITIES_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include "pointcloud_task_creation.h"
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h> 
#include <visualization_msgs/Marker.h>

namespace PointcloudUtilities
{
  bool checkPlaneValidity(std::vector<float> exp_coefficients, std::vector<float> coefficients, bool check_orientation, bool check_distance, float angle_threshold, float dist_threshold); 
  bool checkCylinderValidity(std::vector<float> exp_coefficients, std::vector<float> coefficients, bool check_radius, bool check_orientation, bool check_distance, float radius_threshold, float angle_threshold, float dist_threshold); 
  pointcloud_processing_server::pointcloud_process searchForPlane(ros::ServiceClient *client, sensor_msgs::PointCloud2 input, std::vector<float> expected_coefficients, bool check_orientation, bool check_distance, float angle_threshold, float dist_threshold, std::string name, int max_segmentation_iterations, float distance_threshold, bool should_publish, std::string topic);
  std::vector< pointcloud_processing_server::pointcloud_process > searchForPrimitives(ros::ServiceClient *client, sensor_msgs::PointCloud2 input, std::string yaml_file_name, std::vector<visualization_msgs::Marker> &marker_list);
  geometry_msgs::Pose clippingBoundsPlane(std::vector<float> wall_geometry);
  geometry_msgs::Pose clippingBoundsCylinder(std::vector<float> cylinder_geometry);
  std::vector<float> offsetPlaneCoefficients(std::vector<float> plane_coefficients, std::vector<float> offset);
  std::vector<float> offsetCylinderCoefficients(std::vector<float> cylinder_coefficients, std::vector<float> offset);
  visualization_msgs::Marker makeClippingVisualization(geometry_msgs::Pose clipping_pose, std::vector<float> clipping_box, std::string marker_name, float id, int marker_type);
  std::vector<float> planeCoefficientsFromPose(std::vector<float> wall_geometry, geometry_msgs::Pose robot_pose);
  std::vector<float> planeCoefficientsFromPoints(std::vector<float> wall_geometry);
  std::vector<float> crossProduct(std::vector<float> vector_1, std::vector<float> vector_2);
  float dotProduct(std::vector<float> vector_1, std::vector<float> vector_2);
  float vectorMagnitude(std::vector<float> input);
  std::vector<float> subtractPoints(std::vector<float> point_1, std::vector<float> point_2);
  std::vector<float> scalarMult(std::vector<float> input, float scalar);
  std::vector<float> projectVector(std::vector<float> to_project, std::vector<float> projectee);
  float angleBetweenVectors(std::vector<float> vector_1, std::vector<float> vector_2);
  float distanceFromPlane(std::vector<float> plane_coefficients, std::vector<float> point);
  geometry_msgs::Quaternion quatFromAxis(std::vector<float> axis);
  void tfToStdVector(tf::Vector3 input, std::vector<float> &output);
};

#endif //POINTCLOUD_UTILITIES_H