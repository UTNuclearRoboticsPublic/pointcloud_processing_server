
#ifndef POINTCLOUD_TASK_CREATION_H
#define POINTCLOUD_TASK_CREATION_H

#include <ros/ros.h>
#include <pointcloud_processing_server/pointcloud_process.h>

namespace PointcloudTaskCreation
{
	// Basic task generation functions
	pointcloud_processing_server::pointcloud_task transformTask(std::string name, std::string map_name, bool should_publish, std::string topic);
	pointcloud_processing_server::pointcloud_task clippingTask(std::string name, std::vector<float> box_data, std::vector<float> box_pose, bool keep_ordered, bool should_publish, std::string topic);
	pointcloud_processing_server::pointcloud_task clippingConditionalTask(std::string name, std::vector<float> box_data, bool keep_ordered, bool should_publish, std::string topic);
	pointcloud_processing_server::pointcloud_task voxelizationTask(std::string name, float leaf_size_x, float leaf_size_y, float leaf_size_z, bool keep_ordered, bool should_publish, std::string topic);
	pointcloud_processing_server::pointcloud_task planeSegTask(std::string name, int max_segmentation_iterations, float distance_threshold, bool should_publish, std::string topic, bool should_publish_remainder, std::string topic_remainder, bool remove_cloud);
	pointcloud_processing_server::pointcloud_task cylinderSegTask(std::string name, int max_segmentation_iterations, float distance_threshold, float max_radius, bool should_publish, std::string topic, bool should_publish_remainder, std::string topic_remainder, bool remove_cloud);
	pointcloud_processing_server::pointcloud_task planeSegTask(std::string name, int max_segmentation_iterations, float distance_threshold, bool should_publish, std::string topic, bool should_publish_remainder, std::string topic_remainder, bool remove_cloud);
	// Higher-level task running
	bool processFromYAML(pointcloud_processing_server::pointcloud_process *process, std::string yaml_file_name, std::string prefix);
}; 

#endif //POINTCLOUD_TASK_CREATION_H