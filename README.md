# Table of Contents
1. [About](#about)
2. [Parameter Setup](#parameter-setup)
3. [Running the Server](#running-the-server)

***

## About 
Server which performs basic processing of pointclouds using PCL - filtering, clipping, transformations, voxelizations, segmentations. This is essentially a convenient ROS wrapper for the [Point Cloud Library](http://pointclouds.org/) - or maybe, an extension of the existing in-core ROS PCL wrapping. The aim is just to make usage more convenient and easily accessible even for people with no background in pointcloud operations by creating a very user-friendly, modular, and easily-modified interface. As well, because this system is managed solely through topics and services, it is language-agnostic from the client perspective and can be used either in C++ or Python. 

The user creates a pointcloud_process - a ROS service object which contains a list of pointcloud_task. Each individual pcl_task corresponds to a single operation on a pointcloud. When the server is fed the pointcloud_process it executes the list of pointcloud_tasks in order on the input cloud and publishes and returns the various outputs, depending on the parameter specifications. Generally, the user should not have to interface directly with the pointcloud_process or pointcloud_task types, because they can be initialized using custom yaml file layouts.

## Parameter Setup
pointcloud_process service objects are initialized based on yaml files stored in param/pointcloud_process_from_yaml. These yaml files themselves contain lists of task specifications. The list is preceded by the following two parameters
- min_cloud_size: the service will fail and return if the cloud size ever falls below this threshold
- task_list: the list of task names. It is VERY IMPORTANT that these names match those specified in the yaml file below

Next, each individual task is specified, nested under its name. The format varies across different kinds of processes, so I've explained them each separately below. 

### Transform Tasks
This kind of task allows transformation of the cloud from one frame to another. It is not a rotation or translation of the cloud - it just alters to cloud coordinates so that they can be expressed in the new frame, given that the existing frame and new frame are correctly specified in the same TF tree. 
- type: 1
- map_name: the name of the frame the cloud should be transformed TO (the FROM frame is already in the sensor_msgs/PointCloud2 input)
- should_publish: should the output transformed cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published


### Cropbox Clipping Tasks
This allows clipping of the cloud to include only those points INSIDE a cuboid of arbitrary size and pose in space. Later on I plan to extend this to allow selective REMOVAL of points inside the cuboid, but this doesn't exist yet. 
- type: 2
- box: the definition of the cuboid - 12 parameters, in the following order: 
  - size of the cuboid about the origin (prior to transform): +x, -x, +y, -y, +z, -z 
  - translation of the cuboid: x, y, z
  - rotation of the cuboid: yaw, pitch, roll
  - note that specifying the last 6 parameters as 0 will give a cuboid centered on the origin, oriented to the parent coordinate frame
- keep_ordered: should the cloud be kept ordered, if it was started out ordered? This should probably be false
- should_publish: should the output clipped cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published

### Conditional Clipping Tasks
This is an older implementation from before I set up the cropbox solution. It also crops the cloud by a cuboid, but requries that the cuboid be centered on the origin and unrotated from the parent frame. This task is redundant with the Cropbox task and I might remove it later. 
- type: 3
- box: size of the cuboid in +x, -x, +y, -y, +z, -z 
- keep_ordered: should the cloud be kept ordered, if it was started out ordered? This should probably be false
- should_publish: should the output clipped cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published

### Voxelization Tasks
Voxelization is a downsampling process in which the cloud is broken up into a rectangular grid, and only one point is kept within each grid cell. The lengths in XYZ of the cells can be specified separately (leaf sizes). When multiple points co-occur in a box, they are replaced with their positional centroid. 
- type: 4
- leaf_size: side-lengths of each grid cell, in x, y, z
- keep_ordered: should the cloud be kept ordered, if it was started out ordered? This should probably be false
- should_publish: should the output voxelized cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published

### Planar Segmentation Tasks
A RANSAC routine which searches for planar objects within the cloud. 
- type: 5
- max_iterations: the maximum number of RANSAC iterations allowed (will usually not run to the max number, unless it doesn't find a good solution) 
- dist_threshold: the maximum distance a point can be from the plane and still qualify as part of the model (an inlier)
- should_publish: should the output plane transformed cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published
- should_publish_r: should the REMAINDER cloud (excluding the plane) be published
- publish_topic_r: the name of the topic to publish the REMAINDER cloud (excluding the plane) to, if it should be published
- remove_cloud: should the plane be removed from the cloud before the pointcloud_process proceeds to the next pointcloud_task

### Cylinder Segmentation Tasks
A RANSAC routine which searches for cylindrical objects within the cloud. 
- type: 6
- max_iterations: the maximum number of RANSAC iterations allowed (will usually not run to the max number, unless it doesn't find a good solution) 
- dist_threshold: the maximum distance a point can be from the cylinder and still qualify as part of the model (an inlier)
- max_radius: the maximum radius allowed for the cylinder model
- should_publish: should the output cylinder transformed cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published
- should_publish_r: should the REMAINDER cloud (excluding the cylinder) be published
- publish_topic_r: the name of the topic to publish the REMAINDER cloud (excluding the cylinder) to, if it should be published
- remove_cloud: should the cylinder be removed from the cloud before the pointcloud_process proceeds to the next pointcloud_task

### Line Segmentation Tasks
A RANSAC routine which searches for cylindrical objects within the cloud. 
- type: 7
- max_iterations: the maximum number of RANSAC iterations allowed (will usually not run to the max number, unless it doesn't find a good solution) 
- dist_threshold: the maximum distance a point can be from the line and still qualify as part of the model (an inlier)
- should_publish: should the output line transformed cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published
- should_publish_r: should the REMAINDER cloud (excluding the line) be published
- publish_topic_r: the name of the topic to publish the REMAINDER cloud (excluding the line) to, if it should be published
- remove_cloud: should the line be removed from the cloud before the pointcloud_process proceeds to the next pointcloud_task


### Radius Filter Tasks
A filter which removes points which do not have a certain minimum number of neighbors within a specified distance. 
- type: 8
- search_radius: window in which to look for neighbors
- min_neighbors: the required number of neighbors to persist following the filter
- keep_ordered: should the cloud be kept ordered, if it was started out ordered? This should probably be false
- should_publish: should the output line transformed cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published

### Statistical Filter Tasks
A filter which removes points that qualify as statistical outliers versus the rest of the cloud. 
- type: 8
- k_min: probably something to do with a number of neighbors? been a while since I looked this up or used it :)
- std_mul: yeah don't remember this one at all! coming back to this...
- keep_ordered: should the cloud be kept ordered, if it was started out ordered? This should probably be false
- should_publish: should the output line transformed cloud be published
- publish_topic: the name of the topic to publish the output cloud to, if it should be published

## Running the Server
Run the server itself:

```
rosrun pointcloud_processing_server pointcloud_processing_server
```

Next, set up the pointcloud_process service object in a client file. The preferred way to do this is with yaml files, which can be loaded via launch files or the rosparam terminal command. Once this has been done the client can call the following:

```
pointcloud_processing_server::pointcloud_process process_name;
PointcloudTaskCreation::processFromYAML(&process_name, "yaml_file_name", "pointcloud_process");
```

yaml_file_name above must match the first line from within the yaml file as well, to ensure that the parameter names are found as expected. 

Finally, the server can be run by calling a ros::ServiceClient object:

```
ros::ServiceClient pointcloud_process_client = nh.serviceClient<pointcloud_processing_server::pointcloud_process>("pointcloud_service");
while(!pointcloud_process_client.call(process_name) && ros::ok() )
{
  ROS_WARN("Pointcloud Processing Server call failed - trying again...");
  ros::Duration(1.0).sleep();
}
```
