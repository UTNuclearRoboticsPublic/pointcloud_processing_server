/* NOTES





*/

#include "pointcloud_processing_server/server.h"

template class PointcloudProcessing<pcl::PointXYZ>;
template class PointcloudProcessing<pcl::PointXYZI>;
template class PointcloudProcessing<pcl::PointXYZRGB>;

template <typename PointType> 
PointcloudProcessing<PointType>::PointcloudProcessing()
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  

  should_shut_down = false;
  // this needs to be modified! 
  ROS_DEBUG("[PointcloudProcessing] Server object initialized; creating advertisers.");

  std::string service_name;
  nh_.param<std::string>("pointcloud_service_name", service_name, "pointcloud_service");
  ros::ServiceServer service_server = nh_.advertiseService(service_name, &PointcloudProcessing<PointType>::pointcloud_service, this);
  ROS_INFO_STREAM("PointcloudProcessing: Service " << service_name << " is up!");

  ros::spin();
}

template <typename PointType> bool 
PointcloudProcessing<PointType>::getShutdownStatus()
{
  return should_shut_down;
}

// -------------------------------------------------------- START OF CALLBACK FUNCTION -------------------------------------------------------- 

template <typename PointType> bool 
PointcloudProcessing<PointType>::pointcloud_service(pointcloud_processing_server::pointcloud_process::Request &req, pointcloud_processing_server::pointcloud_process::Response &res)
{
  ros::Time time_start_process = ros::Time::now();

  ROS_DEBUG_STREAM("[PointcloudProcessing] Service request received.");

  res.failed = false; 

  int cloud_size = req.pointcloud.width * req.pointcloud.height;
  // Check Cloud Size > Minimum

  input_cloud_ = PCP(new PC()); //req.pointcloud;
  pcl::fromROSMsg(req.pointcloud, *input_cloud_);                   // creates input_cloud_ from service input message

  current_cloud_frame_ = req.pointcloud.header.frame_id;

  for (int i=0; i<req.tasks.size(); i++)
  {
    ros::Time time_start = ros::Time::now();

    ROS_DEBUG_STREAM("[PointcloudProcessing] Beginning to work on task <" << req.tasks[i].name << "> with type_ind " << req.tasks[i].type_ind << ".");
    
    if ( !(checkMinSize(cloud_size, req.min_cloud_size, req.tasks[i].name)) ) 
    {
      res.failed = true;
      res.failed_during_task_num = i;
      return true;
    }

    switch(req.tasks[i].type_ind){    
      // -------------- PointCloud TF Transform --------------
      case pointcloud_processing_server::pointcloud_task::TRANSFORM_TASK:
      {
        if(i==0)    // If the first task, transform from original input pointcloud frame 
          res.task_results.push_back( transformPointCloud(input_cloud_, req.tasks[i].str_parameters[0], req.pointcloud.header.frame_id) );
        else        // Otherwise, transform from frame of pointcloud output of last process 
          res.task_results.push_back( transformPointCloud(input_cloud_, req.tasks[i].str_parameters[0], res.task_results[i-1].task_pointcloud.header.frame_id) );
        cloud_size = input_cloud_->points.size();
        break;
      }
      // ----------- Clip PointCloud -----------
      case pointcloud_processing_server::pointcloud_task::CLIPPING_TASK:
      {
        std::vector<float> box_data(&req.tasks[i].parameters[0],&req.tasks[i].parameters[6]);
        std::vector<float> box_pose(&req.tasks[i].parameters[6],&req.tasks[i].parameters[12]);
        /*box_pose[0] += box_data[0];
        box_pose[1] += box_data[2];
        box_pose[2] += box_data[4]; */
        ROS_DEBUG_STREAM("[PointcloudProcessing]   Box Dimensions: " << box_data[0] << " " << box_data[1] << " " << box_data[2] << " " << box_data[3] << " " << box_data[4] << " " << box_data[5]);
        ROS_DEBUG_STREAM("[PointcloudProcessing]   Box Pose:       " << box_pose[0] << " " << box_pose[1] << " " << box_pose[2] << " " << box_pose[3] << " " << box_pose[4] << " " << box_pose[5]);
        res.task_results.push_back( clipPointCloud(input_cloud_, box_data, box_pose, req.tasks[i].keep_ordered) );
        cloud_size = input_cloud_->points.size();
        break;
      }

      // ----------- Voxelize PointCloud -----------
      case pointcloud_processing_server::pointcloud_task::VOXELIZING_TASK:
        ROS_DEBUG_STREAM("[PointcloudProcessing]   Voxel Leaf Sizes: " << req.tasks[i].parameters[0] << " " << req.tasks[i].parameters[1] << " " << req.tasks[i].parameters[2]);
        res.task_results.push_back( voxelizePointCloud(input_cloud_, req.tasks[i].parameters) );
        cloud_size = input_cloud_->points.size();
        break;

      // ----------- Plane Segmentation -----------
      case pointcloud_processing_server::pointcloud_task::PLANE_SEG_TASK:
        res.task_results.push_back(segmentTowardPlane(input_cloud_, req.tasks[i].parameters[0], req.tasks[i].parameters[1], req.tasks[i].remove_cloud));
        cloud_size = input_cloud_->points.size();
        if(!res.task_results[i].primitive_found)
        {
          res.failed = true;
          res.failed_during_task_num = i;
          return true;
        }
        break;

      // ----------- Cylinder Segmentation -----------
      case pointcloud_processing_server::pointcloud_task::CYLINDER_SEG_TASK:
        res.task_results.push_back(segmentTowardCylinder(input_cloud_, req.tasks[i].parameters[0], req.tasks[i].parameters[1], req.tasks[i].parameters[2], req.tasks[i].remove_cloud));
        cloud_size = input_cloud_->points.size();
        if(!res.task_results[i].primitive_found)
        {
          res.failed = true;
          res.failed_during_task_num = i;
          return true;
        }
        break;

      // ----------- Line Segmentation -----------
      case pointcloud_processing_server::pointcloud_task::LINE_SEG_TASK:
        res.task_results.push_back(segmentTowardLine(input_cloud_, req.tasks[i].parameters[0], req.tasks[i].parameters[1], req.tasks[i].remove_cloud));
        cloud_size = input_cloud_->points.size();
        if(!res.task_results[i].primitive_found)
        {
          res.failed = true;
          res.failed_during_task_num = i;
          return true;
        }
        break;

      // ----------- Radius Filter -----------
      case pointcloud_processing_server::pointcloud_task::RADIUS_FILTER_TASK:
      {
        ROS_DEBUG_STREAM("[PointcloudProcessing]   Search radius: " << req.tasks[i].parameters[0] << "  Min Neighbors: " << req.tasks[i].parameters[1]);
        res.task_results.push_back( radiusOutlierFilter(input_cloud_, req.tasks[i].parameters[0], req.tasks[i].parameters[1], req.tasks[i].keep_ordered) );
        cloud_size = input_cloud_->points.size();
        break;
      }

      // ----------- Statistical Filter -----------
      case pointcloud_processing_server::pointcloud_task::STATISTICAL_FILTER_TASK:
      {
        ROS_DEBUG_STREAM("[PointcloudProcessing]   Min K: " << req.tasks[i].parameters[0] << "  Stdev Mult: " << req.tasks[i].parameters[1]);
        res.task_results.push_back( statisticalOutlierFilter(input_cloud_, req.tasks[i].parameters[0], req.tasks[i].parameters[1], req.tasks[i].keep_ordered) );
        cloud_size = input_cloud_->points.size();
        break;
      }

    }   // SWITCH 

    ros::Duration task_duration = ros::Time::now() - time_start;
    ROS_DEBUG_STREAM("[PointcloudProcessing]   Current cloud size: " << cloud_size << "  Height: " << input_cloud_->height << "  Width: " << input_cloud_->width << "; Task took " << task_duration << " seconds.");
  }   // FOR LOOP

  ros::Duration process_duration = ros::Time::now() - time_start_process;
  ROS_INFO_STREAM("[PointcloudProcessing]  Finished a Process! Entire process took " << process_duration << " seconds.");

  return true;

}
// -------------------------------------------------------- END OF CALLBACK FUNCTION -------------------------------------------------------- 

  // -------------------------------- Transform Pointcloud --------------------------------
/* 
PointCloud needs to be transformed, since it has the Camera's Center at Origin (not localized in Map frame)
Basic logical structure:
  1) Checks whether a transform is necessary (SHOULD_TRANSFORM_FRAME, passed into function at higher level from req)
  2) If not, just creates INPUT_CLOUD_ object based on cloud input to this function, then exits
  3) If yes, attempts to get transform between frame of cloud input to this function, and the TRANSFORM_TO_FRAME target (also from req at a higher level)
  4) On success, transforms the cloud input to transformed_cloud_pc2 (a sensor_msgs::PointCloud2), then creates INPUT_CLOUD_ from this message, then exits
  5) On failure, performs functionality from (2) above
*/
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::transformPointCloud(PCP &input_cloud_, std::string transform_to_frame, std::string starting_frame)
{
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Transformation request received. Transforming cloud from " << starting_frame << " to " << transform_to_frame << ".");
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input_cloud_, input_pc2);
  input_pc2.header.frame_id = starting_frame;

  pointcloud_processing_server::pointcloud_task_result task_result;
  task_result.input_pointcloud = input_pc2;

  // Transform pointcloud in space to align origin to Camera Frame
  if(camera_frame_listener_.waitForTransform(starting_frame, transform_to_frame, ros::Time::now(), ros::Duration(0.5)))  
  {
    sensor_msgs::PointCloud2 transformed_cloud_pc2;
    pcl_ros::transformPointCloud (transform_to_frame, input_pc2, transformed_cloud_pc2, camera_frame_listener_);  	// transforms input_pc2 into process_message
    pcl::fromROSMsg(transformed_cloud_pc2, *input_cloud_);
    current_cloud_frame_ = transform_to_frame;	
    //transformed_cloud_pc2.header.frame_id = current_cloud_frame_; 
    task_result.task_pointcloud = transformed_cloud_pc2;

    return task_result;
  }
  else {  													// if Transform request times out... Continues WITHOUT TRANSFORM
    ROS_WARN_THROTTLE(60, "[PointcloudProcessing] listen for transformation from %s to %s timed out. Proceeding...", input_pc2.header.frame_id.c_str(), transform_to_frame.c_str());
    task_result.task_pointcloud = input_pc2;
    ROS_DEBUG_STREAM("[PointcloudProcessing]   Transform failed. Returning untransformed cloud...");
    return task_result;
  }

} // -------------------------------- Transform Pointcloud --------------------------------


  // -------------------------------- Clip Pointcloud --------------------------------
/*
Clip the pointcloud, using a box with arbitrary dimensions and pose. 

*/
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::clipPointCloud(PCP &unclipped, std::vector<float> box_data, std::vector<float> box_pose, bool keep_ordered)
{ 
  // Create input message
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input_cloud_, input_pc2);
  input_pc2.header.frame_id = current_cloud_frame_;

  // Insert input message into results
  pointcloud_processing_server::pointcloud_task_result task_result;
  task_result.input_pointcloud = input_pc2;

  pcl::CropBox<PointType> crop;
  crop.setInputCloud(unclipped);
  // Set dimensions of clipping box:
  Eigen::Vector4f min_point = Eigen::Vector4f(box_data[0], box_data[2], box_data[4], 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(box_data[1], box_data[3], box_data[5], 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  // Set pose of clipping box: 
  Eigen::Vector3f translation = Eigen::Vector3f(box_pose[0], box_pose[1], box_pose[2]);
  Eigen::Vector3f rotation = Eigen::Vector3f(box_pose[3], box_pose[4], box_pose[5]);
  crop.setTranslation(translation);
  crop.setRotation(rotation);   

  if(keep_ordered)
  {
  /*If true, points that do not pass the filter will be set to a certain value (default NaN).
    If false, they will be just removed, but that could break the structure of the cloud.     */
    crop.setKeepOrganized(true);    /*
    Organized clouds are clouds taken from camera-like sensors that return a matrix-like image
    If keep organized was set true, points that failed the test will have their Z value set to the following:   */
    crop.setUserFilterValue(0.0);
  }
  else 
    crop.setKeepOrganized(false);

  PCP temp_pcp = PCP(new PC());
  crop.filter(*temp_pcp);
  input_cloud_ = temp_pcp;

  // Create output message
  sensor_msgs::PointCloud2 output_pc2;
  pcl::toROSMsg(*input_cloud_, output_pc2);
  output_pc2.header.frame_id = current_cloud_frame_;

  // Insert output message into results
  task_result.task_pointcloud = output_pc2;

  return task_result; 

} // -------------------------------- Clip Pointcloud --------------------------------


  // -------------------------------- Voxelize Pointcloud --------------------------------
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::voxelizePointCloud(PCP &unvoxelized, std::vector<float> voxel_leaf_size)
{
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input_cloud_, input_pc2);
  input_pc2.header.frame_id = current_cloud_frame_;

  pointcloud_processing_server::pointcloud_task_result task_result;
  task_result.input_pointcloud = input_pc2;

  // Create the Filtering object: downsample the dataset
  pcl::VoxelGrid<PointType> vg;
  vg.setInputCloud(unvoxelized);
  vg.setLeafSize(float(voxel_leaf_size[0]), float(voxel_leaf_size[1]), float(voxel_leaf_size[2]));
  // Apply Filter and return Voxelized Data
  PCP temp_pcp = PCP(new PC());
  vg.filter(*temp_pcp);
  input_cloud_ = temp_pcp;

  sensor_msgs::PointCloud2 output_pc2;
  pcl::toROSMsg(*input_cloud_, output_pc2);
  output_pc2.header.frame_id = current_cloud_frame_;

  task_result.task_pointcloud = output_pc2;

  return task_result;


} // -------------------------------- Voxelize Pointcloud --------------------------------

  // -------------------------------- Segment Toward Plane --------------------------------
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::segmentTowardPlane(PCP &input, int max_iterations, float threshold_distance, bool remove_cloud)
{

  pointcloud_processing_server::pointcloud_task_result task_result;
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input, input_pc2);
  input_pc2.header.frame_id = current_cloud_frame_;
  task_result.input_pointcloud = input_pc2;

  // Starting cloud (populated later via Filter):
  PCP plane_cloud( new PC() );
  PCP remaining_cloud( new PC() );
  plane_cloud->resize(0); 		// probably not necessary?
  remaining_cloud->resize(0); 	// probably not necessary?

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointType> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (max_iterations);
  seg.setDistanceThreshold (threshold_distance);
  seg.setInputCloud (input);
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Performing planar RANSAC segmentation.");
  seg.segment (*inliers, *coefficients);
  
  bool plane_found;
  if(inliers->indices.size() == 0) 
  {
    ROS_WARN("[PointcloudProcessing]   Could not find any planes in the point cloud.");
    task_result.primitive_found = false;
  }  
  else 
  {
    task_result.primitive_found = true;
  }

  // Create object to Segment (extracting plane from the rest of the cloud)
  pcl::ExtractIndices<PointType> extract (true); 	// "Initializing with true will allow us to extract the removed indices" (ie, appending "(true)" after the variable name causes points to be REMOVED from the input cloud?)
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(false); 	// Remove points given by indices
  // Actually segment out plane pointcloud, and set input to match new, smaller cloud
  extract.filter(*plane_cloud); 

  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(true); 		// Remove points NOT given by indices
  // Actually segment out plane pointcloud, and set input to match new, smaller cloud
  extract.filter(*remaining_cloud); 
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Plane size: " << plane_cloud->width*plane_cloud->height << " Input Size: " << input_cloud_->width*input_cloud_->height);
  if(remove_cloud)
    extract.filter(*input_cloud_);

  // Create Messages
  if(task_result.primitive_found)
  {
    // Output cloud for found primitive
    sensor_msgs::PointCloud2 plane_pc2;
    pcl::toROSMsg(*plane_cloud, plane_pc2);
    plane_pc2.header.frame_id = current_cloud_frame_;
    task_result.task_pointcloud = plane_pc2;
    
    // Output cloud for remainder
    sensor_msgs::PointCloud2 remaining_pc2;
    pcl::toROSMsg(*remaining_cloud, remaining_pc2);
    remaining_pc2.header.frame_id = current_cloud_frame_;
    task_result.remaining_pointcloud = remaining_pc2;

    // Output primitive coefficients
    for (int i=0; i < coefficients->values.size(); i++)
    {
      task_result.primitive_coefficients.push_back(coefficients->values[i]);
      ROS_DEBUG_STREAM("[PointcloudProcessing]   Plane Coefficient " << i << ": " << coefficients->values[i]);
    }
  }
  else    // Actually not sure if this is necessary? This might be the default behavior of the filter when it fails to segment an object
  {
    ROS_ERROR_STREAM("[PointcloudProcessing]   Failed to find a plane within the search cloud.");
    sensor_msgs::PointCloud2 empty_cloud;
    empty_cloud.header.frame_id = current_cloud_frame_;
    task_result.task_pointcloud = empty_cloud;            // In failure, return an empty cloud for primitive
    task_result.remaining_pointcloud = input_pc2;         // In failure, return starting cloud for remainder
    for (int i=0; i < coefficients->values.size(); i++)
    {
      task_result.primitive_coefficients.push_back(-1);   // Failure value --> all coefficients = -1
    }
  }
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Returning process after searching for cloud.");

  return task_result; 
  
} // -------------------------------- Segment Toward Plane --------------------------------


  // -------------------------------- Segment Toward Cylinder --------------------------------
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::segmentTowardCylinder(PCP &input, int max_iterations, float threshold_distance, float max_radius, bool remove_cloud)
{

  pointcloud_processing_server::pointcloud_task_result task_result;
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input, input_pc2);
  task_result.input_pointcloud = input_pc2;

  // Starting cloud (populated later via Filter):
  PCP cylinder_cloud( new PC() );
  PCP remaining_cloud( new PC() );
  cylinder_cloud->resize(0); 		// probably not necessary?
  remaining_cloud->resize(0); 	// probably not necessary?

  ROS_DEBUG_STREAM("[PointcloudProcessing]   Estimating cloud normals...");
  pcl::NormalEstimationOMP<PointType, pcl::Normal> ne;
  typedef typename pcl::search::KdTree<PointType>::Ptr KdTreePtr;
  KdTreePtr tree (new pcl::search::KdTree<PointType> ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod (tree);
  ne.setInputCloud (input);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (max_iterations);
  seg.setDistanceThreshold (threshold_distance);
  seg.setRadiusLimits (0, max_radius);
  seg.setInputCloud (input);
  seg.setInputNormals (cloud_normals);
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Performing cylinder RANSAC segmentation.");
  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers, *coefficients);
  //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  if(inliers->indices.size() == 0) 
  {
    ROS_WARN("[PointcloudProcessing]   Could not find any cylinders in the point cloud.");
    task_result.primitive_found = false;
  }  
  else 
  {
    task_result.primitive_found = true;
  }

  // Write the cylinder inliers to disk
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cylinder_cloud);

  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(true);    // Remove points NOT given by indices
  // Actually segment out plane pointcloud, and set input to match new, smaller cloud
  extract.filter(*remaining_cloud); 
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Cylinder size: " << cylinder_cloud->width*cylinder_cloud->height << " Remainder Size: " << input_cloud_->width*input_cloud_->height);
  if(remove_cloud)
    extract.filter(*input_cloud_);

  if(task_result.primitive_found)
  {
    // Output cylinder cloud
    sensor_msgs::PointCloud2 cylinder_pc2;
    pcl::toROSMsg(*cylinder_cloud, cylinder_pc2);
    cylinder_pc2.header.frame_id = current_cloud_frame_;
    task_result.task_pointcloud = cylinder_pc2;
    // Output remainder cloud
    sensor_msgs::PointCloud2 remaining_pc2;
    pcl::toROSMsg(*remaining_cloud, remaining_pc2);
    remaining_pc2.header.frame_id = current_cloud_frame_;
    task_result.remaining_pointcloud = remaining_pc2;
    // Output cylinder coefficients
    for (int i=0; i < coefficients->values.size(); i++)
    {
      task_result.primitive_coefficients.push_back(coefficients->values[i]);
      ROS_DEBUG_STREAM("[PointcloudProcessing]   Cylinder Coefficient " << i << ": " << coefficients->values[i]);
    }
  }
  else    // Actually not sure if this is necessary? This might be the default behavior of the filter when it fails to segment an object
  {
    ROS_ERROR_STREAM("[PointcloudProcessing]   Failed to find a cylinder within the search cloud.");
    // In failure, return an empty cloud for primitive
    sensor_msgs::PointCloud2 empty_cloud;
    empty_cloud.header.frame_id = current_cloud_frame_;
    task_result.task_pointcloud = empty_cloud;            
    // In failure, return starting cloud for remainder
    task_result.remaining_pointcloud = input_pc2;         
    // Failure value --> all coefficients = -1
    for (int i=0; i < coefficients->values.size(); i++)
    {
      task_result.primitive_coefficients.push_back(-1);   
    }
  }
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Returning process after searching for cloud.");

  return task_result;
} // -------------------------------- Segment Toward Cylinder --------------------------------

  // -------------------------------- Segment Toward Line --------------------------------
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::segmentTowardLine(PCP &input, int max_iterations, float threshold_distance, bool remove_cloud)
{
  pointcloud_processing_server::pointcloud_task_result task_result;
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input, input_pc2);
  task_result.input_pointcloud = input_pc2;

  // Starting cloud (populated later via Filter):
  PCP line_cloud( new PC() );
  PCP remaining_cloud( new PC() );
  line_cloud->resize(0);    // probably not necessary?
  remaining_cloud->resize(0);   // probably not necessary?

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;

  // Create the segmentation object for line segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (max_iterations);
  seg.setDistanceThreshold (threshold_distance);
  seg.setInputCloud (input);
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Performing line RANSAC segmentation.");
  // Obtain the line inliers and coefficients
  seg.segment (*inliers, *coefficients);
  //std::cerr << "Line coefficients: " << *coefficients_cylinder << std::endl;

  if(inliers->indices.size() == 0) 
  {
    ROS_INFO("[PointcloudProcessing]   Could not find any lines in the point cloud.");
    task_result.primitive_found = false;
  }  
  else 
  {
    ROS_INFO("[PointcloudProcessing]   Successfully removed a line. Size = %lu", inliers->indices.size());
    task_result.primitive_found = true;
  }

  // Write the line inliers to disk
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*line_cloud);

  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(true);    // Remove points NOT given by indices
  // Actually segment out plane pointcloud, and set input to match new, smaller cloud
  extract.filter(*remaining_cloud); 
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Line size: " << line_cloud->width*line_cloud->height << " Remainder Size: " << input_cloud_->width*input_cloud_->height);
  if(remove_cloud)
    extract.filter(*input_cloud_);

  if(task_result.primitive_found)
  {
    // Output line cloud
    sensor_msgs::PointCloud2 line_pc2;
    pcl::toROSMsg(*line_cloud, line_pc2);
    line_pc2.header.frame_id = current_cloud_frame_;
    task_result.task_pointcloud = line_pc2;
    // Output remainder cloud
    sensor_msgs::PointCloud2 remaining_pc2;
    pcl::toROSMsg(*remaining_cloud, remaining_pc2);
    remaining_pc2.header.frame_id = current_cloud_frame_;
    task_result.remaining_pointcloud = remaining_pc2;
    // Output line coefficients
    for (int i=0; i < coefficients->values.size(); i++)
    {
      task_result.primitive_coefficients.push_back(coefficients->values[i]);
      ROS_DEBUG_STREAM("[PointcloudProcessing]   Line Coefficient " << i << ": " << coefficients->values[i]);
    }
  }
  else    // Actually not sure if this is necessary? This might be the default behavior of the filter when it fails to segment an object
  {
    ROS_ERROR_STREAM("[PointcloudProcessing]   Failed to find a line within the search cloud.");
    // In failure, return an empty cloud for primitive
    sensor_msgs::PointCloud2 empty_cloud;
    empty_cloud.header.frame_id = current_cloud_frame_;
    task_result.task_pointcloud = empty_cloud;            
    // In failure, return starting cloud for remainder
    task_result.remaining_pointcloud = input_pc2;         
    // Failure value --> all coefficients = -1
    for (int i=0; i < coefficients->values.size(); i++)
    {
      task_result.primitive_coefficients.push_back(-1);   
    }
  }
  ROS_DEBUG_STREAM("[PointcloudProcessing]   Returning process after searching for cloud.");

  return task_result;
} // -------------------------------- Segment Toward Line --------------------------------


  // -------------------------------- Radius Outlier Removal --------------------------------
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::radiusOutlierFilter(PCP &unfiltered, float search_radius, int min_neighbors, bool keep_ordered)
{ 
  // Create input message
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input_cloud_, input_pc2);
  input_pc2.header.frame_id = current_cloud_frame_;

  // Insert input message into results
  pointcloud_processing_server::pointcloud_task_result task_result;
  task_result.input_pointcloud = input_pc2;

  // Construct filter
  pcl::RadiusOutlierRemoval<PointType> filter;
  filter.setInputCloud(unfiltered);
  filter.setRadiusSearch(search_radius);
  filter.setMinNeighborsInRadius(min_neighbors);
  filter.setKeepOrganized(false);
  // Perform filtering
  filter.filter(*input_cloud_);

  // Create output message
  sensor_msgs::PointCloud2 output_pc2;
  pcl::toROSMsg(*input_cloud_, output_pc2);
  output_pc2.header.frame_id = current_cloud_frame_;

  // Insert output message into results
  task_result.task_pointcloud = output_pc2;

  return task_result; 
} // -------------------------------- Radius Outlier Removal --------------------------------

  // -------------------------------- Statistical Outlier Removal --------------------------------
template <typename PointType> pointcloud_processing_server::pointcloud_task_result 
PointcloudProcessing<PointType>::statisticalOutlierFilter(PCP &unfiltered, int k_min, float std_mul, bool keep_ordered)
{ 
  // Create input message
  sensor_msgs::PointCloud2 input_pc2;
  pcl::toROSMsg(*input_cloud_, input_pc2);
  input_pc2.header.frame_id = current_cloud_frame_;

  // Insert input message into results
  pointcloud_processing_server::pointcloud_task_result task_result;
  task_result.input_pointcloud = input_pc2;

  // Construct filter
  pcl::StatisticalOutlierRemoval<PointType> filter;
  filter.setInputCloud(unfiltered);
  filter.setMeanK(k_min);
  filter.setStddevMulThresh(std_mul);
  filter.setKeepOrganized(false);
  // Perform filtering
  filter.filter(*input_cloud_);

  // Create output message
  sensor_msgs::PointCloud2 output_pc2;
  pcl::toROSMsg(*input_cloud_, output_pc2);
  output_pc2.header.frame_id = current_cloud_frame_;

  // Insert output message into results
  task_result.task_pointcloud = output_pc2;

  return task_result; 
} // -------------------------------- Statistical Outlier Removal --------------------------------


  // -------------------------------- Check Min Size --------------------------------
// Check to see if current cloud (with processing up until this point) is too small to continue
template <typename PointType> bool 
PointcloudProcessing<PointType>::checkMinSize(int cloud_size, int min_num_points, std::string task_name)
{
  if(cloud_size <= min_num_points) 
  {
    ROS_ERROR("[PointcloudProcessing] Point cloud is too small to proces: Min: %u, actual: %u", min_num_points, cloud_size);
    ROS_ERROR("[PointcloudProcessing] Exiting during %s phase of processing.", task_name.c_str());
    return false;
  }
  return true;
} // -------------------------------- Check Min Size --------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_processing_server");

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if( !strcmp(argv[argc-1], "-intensity" ) )
  {
    ROS_INFO_STREAM("[PointcloudProcessing] Initializing server using pcl::PointXYZI data type. (Argument " << argv[argc-1] << " selected).");
    PointcloudProcessing<pcl::PointXYZI> server;
  }
  else if( !strcmp(argv[argc-1], "-color" ) )
  {
    ROS_INFO_STREAM("[PointcloudProcessing] Initializing server using pcl::PointXYZRGB data type. (Argument " << argv[argc-1] << " selected).");
    PointcloudProcessing<pcl::PointXYZRGB> server;
  }
  else if( !strcmp(argv[argc-1], "-xyz" ) )
  {
    ROS_WARN_STREAM("[PointcloudProcessing] Initializing server using pcl::PointXYZ data type. (Argument " << argv[argc-1] << " selected).");
    PointcloudProcessing<pcl::PointXYZ> server;
  }
  else
  {
    ROS_WARN_STREAM("[PointcloudProcessing] Point type not specified! Initializing server and defaulting to pcl::PointXYZ.");
    PointcloudProcessing<pcl::PointXYZ> server;
  }
  
  //while( !server.getShutdownStatus() && ros::ok() );

  return 0;
}
