
#include "pointcloud_processing_server/pointcloud_task_creation.h"

namespace PointcloudTaskCreation
{
//public:
	// Basic task generation functions
	pointcloud_processing_server::pointcloud_task transformTask(std::string name, std::string map_name, bool should_publish, std::string topic)
	{
		pointcloud_processing_server::pointcloud_task transform_task;
		transform_task.name = name;  
		transform_task.type_ind = 1;
		transform_task.str_parameters.push_back(map_name);
		transform_task.should_publish = should_publish;
		transform_task.pub_topic = topic;
		return transform_task;
	}

	pointcloud_processing_server::pointcloud_task clippingTask(std::string name, std::vector<float> box_data, std::vector<float> box_pose, bool keep_ordered, bool should_publish, std::string topic)
	{
		pointcloud_processing_server::pointcloud_task clipping_task;
		clipping_task.name = name;
		clipping_task.type_ind = 2;
		for(int i=0; i<6; i++)
			clipping_task.parameters.push_back(box_data[i]);       
		for(int i=0; i<6; i++)
			clipping_task.parameters.push_back(box_pose[i]);       
		clipping_task.keep_ordered = keep_ordered;
		clipping_task.should_publish = should_publish;
		clipping_task.pub_topic = topic;
		return clipping_task;
	}

	pointcloud_processing_server::pointcloud_task clippingConditionalTask(std::string name, std::vector<float> box_data, bool keep_ordered, bool should_publish, std::string topic)
	{
		pointcloud_processing_server::pointcloud_task clipping_task;
		clipping_task.name = name;
		clipping_task.type_ind = 3;
		for(int i=0; i<6; i++)
			clipping_task.parameters.push_back(box_data[i]);   
		clipping_task.keep_ordered = keep_ordered;        
		clipping_task.should_publish = should_publish;
		clipping_task.pub_topic = topic;
		return clipping_task;
	}

	pointcloud_processing_server::pointcloud_task voxelizationTask(std::string name, float leaf_size_x, float leaf_size_y, float leaf_size_z, bool keep_ordered, bool should_publish, std::string topic)
	{
		pointcloud_processing_server::pointcloud_task voxelizing_task;
		voxelizing_task.name = name;
		voxelizing_task.type_ind = 4;
		voxelizing_task.parameters.push_back(leaf_size_x);
		voxelizing_task.parameters.push_back(leaf_size_y); 
		voxelizing_task.parameters.push_back(leaf_size_z);
		voxelizing_task.keep_ordered = keep_ordered;
		voxelizing_task.should_publish = should_publish;
		voxelizing_task.pub_topic = topic;
		return voxelizing_task;
	}

	pointcloud_processing_server::pointcloud_task planeSegTask(std::string name, int max_segmentation_iterations, float distance_threshold, bool should_publish, std::string topic, bool should_publish_remainder, std::string topic_remainder, bool remove_cloud)
	{
		pointcloud_processing_server::pointcloud_task plane_task;
		plane_task.name = name;
		plane_task.type_ind = 5;
		plane_task.parameters.push_back(max_segmentation_iterations);        
		plane_task.parameters.push_back(distance_threshold);        
		plane_task.should_publish = should_publish;
		plane_task.pub_topic = topic;
		plane_task.should_publish_remainder = should_publish_remainder;
		plane_task.pub_topic_remainder = topic_remainder;
		plane_task.remove_cloud = remove_cloud;
		return plane_task;
	}

	pointcloud_processing_server::pointcloud_task cylinderSegTask(std::string name, int max_segmentation_iterations, float distance_threshold, float max_radius, bool should_publish, std::string topic, bool should_publish_remainder, std::string topic_remainder, bool remove_cloud)
	{
		pointcloud_processing_server::pointcloud_task cylinder_task;
		cylinder_task.name = name;
		cylinder_task.type_ind = 6;
		cylinder_task.parameters.push_back(max_segmentation_iterations);        
		cylinder_task.parameters.push_back(distance_threshold);        
		cylinder_task.parameters.push_back(max_radius);
		cylinder_task.should_publish = should_publish;
		cylinder_task.pub_topic = topic;
		cylinder_task.should_publish_remainder = should_publish_remainder;
		cylinder_task.pub_topic_remainder = topic_remainder;
		cylinder_task.remove_cloud = remove_cloud;
		return cylinder_task;
	}

	pointcloud_processing_server::pointcloud_task lineSegTask(std::string name, int max_segmentation_iterations, float distance_threshold, bool should_publish, std::string topic, bool should_publish_remainder, std::string topic_remainder, bool remove_cloud)
	{
		pointcloud_processing_server::pointcloud_task line_task;
		line_task.name = name;
		line_task.type_ind = 7;
		line_task.parameters.push_back(max_segmentation_iterations);        
		line_task.parameters.push_back(distance_threshold);        
		line_task.should_publish = should_publish;
		line_task.pub_topic = topic;
		line_task.should_publish_remainder = should_publish_remainder;
		line_task.pub_topic_remainder = topic_remainder;
		line_task.remove_cloud = remove_cloud;
		return line_task;
	}

	pointcloud_processing_server::pointcloud_task radiusFilterTask(std::string name, float search_radius, int min_neighbors, bool keep_ordered, bool should_publish, std::string topic)
	{
		pointcloud_processing_server::pointcloud_task filter_task;
		filter_task.name = name;
		filter_task.type_ind = 9;
		filter_task.parameters.push_back(search_radius);       
		filter_task.parameters.push_back(min_neighbors);       
		filter_task.keep_ordered = keep_ordered;
		filter_task.should_publish = should_publish;
		filter_task.pub_topic = topic;
		return filter_task;
	}

	pointcloud_processing_server::pointcloud_task statisticalFilterTask(std::string name, int k_min, float std_mul, bool keep_ordered, bool should_publish, std::string topic)
	{
		pointcloud_processing_server::pointcloud_task filter_task;
		filter_task.name = name;
		filter_task.type_ind = 8;
		filter_task.parameters.push_back(k_min);       
		filter_task.parameters.push_back(std_mul);          
		filter_task.keep_ordered = keep_ordered;
		filter_task.should_publish = should_publish;
		filter_task.pub_topic = topic;
		return filter_task;
	}

	// Creates a pointcloud_process object from a YAML file template
	bool processFromYAML(pointcloud_processing_server::pointcloud_process *process, std::string yaml_file_name, std::string prefix)
	{ 
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  
		ros::NodeHandle nh;
		ROS_DEBUG_STREAM("[PointcloudTaskCreation] Beginning to attempt to initialize parameters from server for process " + yaml_file_name + ".");
		std::vector<std::string> task_list;
		if( !nh.getParam(prefix + "/" + yaml_file_name + "/task_list", task_list) )
		{
			ROS_ERROR("[PointcloudTaskCreation] Failed to get task names from parameter server.");
			return false;
		}
		else ROS_DEBUG_STREAM("[PointcloudTaskCreation] Successfully retrieved task names from parameter server.");

		float temp_float;
		std::string temp_string;
		bool temp_bool;
		std::vector<float> temp_float_vector;
		for (int i=0; i<task_list.size(); i++)
		{ 
			std::string task_type_str;
			pointcloud_processing_server::pointcloud_task task;
			// Capture task type from parameter (either as integer code or string)
			if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/type", task.type_ind) )
				if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/type", task_type_str) )
				{
					ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get task type for task " << task_list[i] << ", either as a string name or int code." );
					return false;
				} 
				else 		// Allow for task specification as a string from the following list:
				{
					ROS_DEBUG_STREAM("[PointcloudTaskCreation] Type for task " << task_list[i] << " retrieved as a string, with value " << task_type_str);
					if(task_type_str.compare("transform")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::TRANSFORM_TASK;
					else if(task_type_str.compare("clipping")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::CLIPPING_TASK;
					else if(task_type_str.compare("voxelizing")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::VOXELIZING_TASK;
					else if(task_type_str.compare("plane_seg")==0 || task_type_str.compare("plane_segmentation")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK;
					else if(task_type_str.compare("cylinder_seg")==0 || task_type_str.compare("cylinder_segmentation")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK;
					else if(task_type_str.compare("line_seg")==0 || task_type_str.compare("line_segmentation")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::LINE_SEG_TASK;
					else if(task_type_str.compare("radius_filter")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::RADIUS_FILTER_TASK;
					else if(task_type_str.compare("statistical_filter")==0)
						task.type_ind = pointcloud_processing_server::pointcloud_task::STATISTICAL_FILTER_TASK;
					else 
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Task type for task " << task_list[i] << " was specified as string " << task_type_str << " which is not from the accepted list. Returning as failed...");
						return false;
					}
				}
			ROS_DEBUG_STREAM("[PointcloudTaskCreation] Successfully retrieved task type for task " << task_list[i] << " as " << task.type_ind);	
			switch(task.type_ind)
			{
				case pointcloud_processing_server::pointcloud_task::TRANSFORM_TASK: 		// Transformation
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/map_name", temp_string) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /map_name for transform, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					} 
					task.str_parameters.push_back(temp_string);
					break;
				case pointcloud_processing_server::pointcloud_task::CLIPPING_TASK:			// Clipping 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/box", task.parameters) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /box for clipping, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/keep_ordered", temp_bool) )
					{
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /keep_ordered for clipping, in " << task_list[i] << " task of process " << yaml_file_name << ". Defaulting to true." );
						temp_bool = true;
					}
					task.keep_ordered = temp_bool;					
					break;
				case pointcloud_processing_server::pointcloud_task::VOXELIZING_TASK:			// Voxelization 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/leaf_sizes", task.parameters) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /leaf_sizes for voxelization, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/keep_ordered", temp_bool) )
					{
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /keep_ordered for voxelization, in " << task_list[i] << " task of process " << yaml_file_name << ". Defaulting to true." );
						temp_bool = true;
					}
					break;
				case pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK:			// Planar Segmentation 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/max_iterations", temp_float) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /max_iterations for plane_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/dist_threshold", temp_float) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /dist_threshold for plane_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					task.parameters.push_back(temp_float);
					nh.param<bool>(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/should_publish_r", temp_bool, true);
					task.should_publish_remainder = temp_bool;
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/publish_topic_r", temp_string) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /publish_topic_r for plane_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					else
						task.pub_topic_remainder = temp_string;
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/remove_cloud", temp_bool) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /remove_cloud for plane_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					else 
						task.remove_cloud = temp_bool;
					break;
				case pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK:			// Planar Segmentation 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/max_iterations", temp_float) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /max_iterations for cylinder_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/dist_threshold", temp_float) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /dist_threshold for cylinder_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/max_radius", temp_float) )
					{
						temp_float = 1.0;
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /max_radius for cylinder_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << ". Defaulting to " << temp_float << ".");
					}
					task.parameters.push_back(temp_float);
					nh.param<bool>(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/should_publish_r", temp_bool, true);
					task.should_publish_remainder = temp_bool;
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/publish_topic_r", temp_string) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /publish_topic_r for cylinder_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					else
						task.pub_topic_remainder = temp_string;
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/remove_cloud", temp_bool) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /remove_cloud for cylinder_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					else 
						task.remove_cloud = temp_bool;
					break;
				case pointcloud_processing_server::pointcloud_task::LINE_SEG_TASK:			// Line Segmentation 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/max_iterations", temp_float) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /max_iterations for line_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/dist_threshold", temp_float) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /dist_threshold for line_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					nh.param<bool>(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/should_publish_r", temp_bool, true);
					task.should_publish_remainder = temp_bool;
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/publish_topic_r", temp_string) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /publish_topic_r for line_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					else
						task.pub_topic_remainder = temp_string;
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/remove_cloud", temp_bool) )
					{
						ROS_ERROR_STREAM("[PointcloudTaskCreation] Failed to get /remove_cloud for line_segmentation, in " << task_list[i] << " task of process " << yaml_file_name << "." );
						return false;
					}
					else 
						task.remove_cloud = temp_bool;
					break;
				case pointcloud_processing_server::pointcloud_task::RADIUS_FILTER_TASK:			// Radius Filter 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/search_radius", temp_float) )
					{
						task.parameters.push_back(0.02);
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /search_radius for clipping, in " << task_list[i] << " task of process " << yaml_file_name << "." << "Defaulting to " << task.parameters[0] << "." );
					}
					else 
						task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/min_neighbors", temp_float) )
					{
						task.parameters.push_back(10);
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /min_neighbors for clipping, in " << task_list[i] << " task of process " << yaml_file_name << "." << "Defaulting to " << task.parameters[0] << "." );
					}
					else 
						task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/keep_ordered", temp_bool) )
					{
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /keep_ordered for clipping, in " << task_list[i] << " task of process " << yaml_file_name << ". Defaulting to true." );
						temp_bool = true;
					}
					task.keep_ordered = temp_bool;					
					break;
				case pointcloud_processing_server::pointcloud_task::STATISTICAL_FILTER_TASK:			// Statistical Filter 
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/k_min", temp_float) )
					{
						task.parameters.push_back(10);
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /k_min for clipping, in " << task_list[i] << " task of process " << yaml_file_name << "." << "Defaulting to " << task.parameters[0] << "." );
					}
					else 
						task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/std_mul", temp_float) )
					{
						task.parameters.push_back(1);
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /std_mul for clipping, in " << task_list[i] << " task of process " << yaml_file_name << "." << "Defaulting to " << task.parameters[0] << "." );
					}
					else 
						task.parameters.push_back(temp_float);
					if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/keep_ordered", temp_bool) )
					{
						ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get /keep_ordered for clipping, in " << task_list[i] << " task of process " << yaml_file_name << ". Defaulting to true." );
						temp_bool = true;
					}
					task.keep_ordered = temp_bool;					
					break;
			}
			// Should task publish its output cloud?
			nh.param<bool>(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/should_publish", temp_bool, true);
			task.should_publish = temp_bool;
			// Where should task publish its output cloud?
			if( !nh.getParam(prefix + "/" + yaml_file_name + "/tasks/" + task_list[i] + "/publish_topic", temp_string) )
			{
				ROS_WARN_STREAM("[PointcloudTaskCreation] Failed to get publishing topic in " << task_list[i] << " task of process " << yaml_file_name << "." );
				task.pub_topic = "default_cloud_output_topic";
			}
			else 
				task.pub_topic = temp_string;
			// Set task name and add to process
			task.name = task_list[i];
			process->request.tasks.push_back(task);
			ROS_DEBUG_STREAM("[PointcloudTaskCreation] Created task " << task_list[i] << " and added it to process " << yaml_file_name << ".");
		}
		nh.param<int>(prefix + "/" + yaml_file_name + "/min_cloud_size", process->request.min_cloud_size, 50);

		ROS_DEBUG_STREAM("[PointcloudTaskCreation] Successfully loaded all data from parameter server for process " << yaml_file_name << ".");

		return true;
	}    

}; 