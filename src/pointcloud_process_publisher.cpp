
#include "pointcloud_processing_server/pointcloud_process_publisher.h"

PointcloudProcessPublisher::PointcloudProcessPublisher()
{

}

PointcloudProcessPublisher::PointcloudProcessPublisher(ros::NodeHandle nh)
{
	nh_ = nh;
}

PointcloudProcessPublisher::PointcloudProcessPublisher(ros::NodeHandle nh, pointcloud_processing_server::pointcloud_process process)
{
	nh_ = nh;
	this->updatePublishers(process);
}

void PointcloudProcessPublisher::updateNodeHandle(ros::NodeHandle nh)
{
	nh_ = nh;
}

void PointcloudProcessPublisher::updatePublishers(pointcloud_processing_server::pointcloud_process process)
{
	publishers_.clear();
	remainder_publishers_.clear();
	for(int i=0; i<process.request.tasks.size(); i++)
	{
		ros::Publisher temp_publisher = nh_.advertise<sensor_msgs::PointCloud2>(process.request.tasks[i].pub_topic, 1);
		ROS_DEBUG_STREAM("[PointcloudProcessPublisher] Created new publisher on topic " << temp_publisher.getTopic() << "; the " << i << "th publisher for this process.");
		publishers_.push_back(temp_publisher);
		ros::Publisher temp_remainder_publisher = nh_.advertise<sensor_msgs::PointCloud2>(process.request.tasks[i].pub_topic_remainder, 1);
		ROS_DEBUG_STREAM("[PointcloudProcessPublisher] Created new publisher on topic " << temp_remainder_publisher.getTopic() << "; the " << i << "th publisher for this process.");
		remainder_publishers_.push_back(temp_remainder_publisher);

		should_publish_.push_back(process.request.tasks[i].should_publish);
		if( process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK 		|| 
			process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK 	|| 
			process.request.tasks[i].type_ind == pointcloud_processing_server::pointcloud_task::LINE_SEG_TASK 		   )
			should_publish_remainder_.push_back(process.request.tasks[i].should_publish_remainder);
		else should_publish_remainder_.push_back(false);
	}
	is_initialized_ = true;
}

void PointcloudProcessPublisher::publish(pointcloud_processing_server::pointcloud_process process)
{
	if(is_initialized_)
	{
		for(int i=0; i<publishers_.size(); i++)
		{
			if(!process.response.failed || process.response.failed_during_task_num != i) // Don't publish from failed tasks
			{
				// Don't publish unless should_publish, and cloud size > minimum 
				// In theory, failure of the latter condition should already result in a task failure... but check to be sure
				if(should_publish_[i] && process.response.task_results[i].task_pointcloud.width*process.response.task_results[i].task_pointcloud.height > process.request.min_cloud_size)
				{
					ROS_DEBUG_STREAM("[PointcloudProcessPublisher] Attempting to publish cloud of size " << process.response.task_results[i].task_pointcloud.width*process.response.task_results[i].task_pointcloud.height << " from process " << process.request.tasks[i].name << " on topic " << publishers_[i].getTopic() << ".");
					publishers_[i].publish(process.response.task_results[i].task_pointcloud);
				}
				if(should_publish_remainder_[i] && process.response.task_results[i].remaining_pointcloud.width*process.response.task_results[i].remaining_pointcloud.height > process.request.min_cloud_size)
				{
					ROS_DEBUG_STREAM("[PointcloudProcessPublisher] Attempting to publish remainder cloud of size " << process.response.task_results[i].remaining_pointcloud.width*process.response.task_results[i].remaining_pointcloud.height << " from process " << process.request.tasks[i].name << " on topic " << remainder_publishers_[i].getTopic() << ".");
					remainder_publishers_[i].publish(process.response.task_results[i].remaining_pointcloud);
				}
			}
		}
	}
	//if(bag_is_running_)
		// kill 
}

void PointcloudProcessPublisher::createBag(std::string file_name)
{/*
	ROS_ERROR_STREAM("saving");
	rosbag::Bag bag;
	bag.open(bag_name_+".bag", rosbag::bagmode::Write);
	bag.write(cloud_pub_.getTopic(), ros::Time::now(), summed_pointcloud_);  */
}

void PointcloudProcessPublisher::forcePublishing()
{
	should_publish_.clear();
	should_publish_remainder_.clear();
	for(int i=0; i<publishers_.size(); i++)
	{
		should_publish_.push_back(true);
		should_publish_remainder_.push_back(true);
	}
}

bool PointcloudProcessPublisher::isInitialized()
{
	return is_initialized_;
}