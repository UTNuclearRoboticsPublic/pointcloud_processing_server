
sensor_msgs/PointCloud2 input_pointcloud 		# pointcloud input for task 
sensor_msgs/PointCloud2 task_pointcloud 		# pointcloud found via task operation
sensor_msgs/PointCloud2 remaining_pointcloud 	# pointcloud remaining given removal of found cloud from input
float32[] primitive_coefficients 				# to be populated for segmentation tasks with coefficients of the found primitive
bool primitive_found