
#ifndef POINTCLOUD_PROCESS_PUBLISHER_H
#define POINTCLOUD_PROCESS_PUBLISHER_H

#include <ros/ros.h>
#include <pointcloud_processing_server/pointcloud_process.h>
#include <rosbag/bag.h>

class PointcloudProcessPublisher
{
public:
	PointcloudProcessPublisher();
	PointcloudProcessPublisher(ros::NodeHandle nh);
	PointcloudProcessPublisher(ros::NodeHandle nh, pointcloud_processing_server::pointcloud_process process);
	void updateNodeHandle(ros::NodeHandle nh);
	void updatePublishers(pointcloud_processing_server::pointcloud_process process);
	void publish(pointcloud_processing_server::pointcloud_process process);
	void createBag(std::string file_name);
	void forcePublishing();
	bool isInitialized();

private:
	std::vector<ros::Publisher> publishers_;
	std::vector<ros::Publisher> remainder_publishers_;
	std::vector<bool> should_publish_;
	std::vector<bool> should_publish_remainder_;
	ros::NodeHandle nh_;
	bool is_initialized_;
	bool bag_is_running_;

};

#endif //POINTCLOUD_PROCESS_PUBLISHER_H