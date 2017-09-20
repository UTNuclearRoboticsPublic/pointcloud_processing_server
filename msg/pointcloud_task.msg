
# ************************************** Actual Data Structure ***************************************


string name

int32 type_ind 					# 1=Transform;  2=Clip;  3=Voxelize;  4=Plane_Seg;  5=Cylinder_Seg
float32[] parameters
string[] str_parameters
bool keep_ordered

string pub_topic
string pub_topic_remainder
bool should_publish
bool should_publish_remainder
bool remove_cloud


# *********************************************** USAGE ***********************************************

# For all task types, PUB_TOPIC and SHOULD_PUBLISH are self-explanatory. 
# Currently, planned or existing support for the following task types:

# -------------- move-it! ---------------
# Allows movement of an serial robot arm to re-position a camera. 
#   name = "move_it";
#   more to add later... NOT SUPPORTED YET
#   will rely on external implementation of ROS move-it! 


# -------------- navigation ---------------
# Allows movement of a robot base to re-position a camera. 
#   name = "navigation";
#   more to add later... NOT SUPPORTED YET
#   will rely on external implementation of ROS nav_stack 


# -------------- Clipping --------------
# Clips the input pointcloud by the rectangular box bounded by these dimensions. 
#   name = "clip";
#   parameters[0] = X_MIN; 				// Boundaries of the bounding box, from cloud origin
#   parameters[1] = X_MAX;
#   parameters[2] = Y_MIN;
#   parameters[3] = Y_MAX;
#   parameters[4] = Z_MIN;
#   parameters[5] = Z_MAX;


# -------------- Voxelizing --------------
# Re-samples the pointcloud more coarsely; max of one point in each box with the dimensions specified. 
# If only the first leaf size is populated, should use that value for all three. 
#   name = "voxelize";
#   parameters[0] = VOXEL_LEAF_SIZE_X;
#   parameters[1] = VOXEL_LEAF_SIZE_Y;
#   parameters[2] = VOXEL_LEAF_SIZE_Z; 


# -------------- Transformation --------------
# Transforms the pointcloud to be in the specified frame.
#   name = "transform"; 
#   str_parameters[0] = TARGET_FRAME; 			// Frame to which pointcloud is to be transformed


# -------------- Plane Segmentation --------------
# Segments the pointcloud in an attempt to find a plane. 
#   name = "plane_segmentation";
#   parameters[0] = MAX_SEGMENTATION_ITERATIONS; 	// Maximum number of segmentation iterations 
#   parameters[1] = THRESHOLD_DISTANCE; 		// Max distance of a point from plane to be included in plane model


# -------------- Cylinder Segmentation --------------
# Segments the pointcloud in an attempt to find a plane. 
#   name = "cylinder_segmentation";
#   parameters[0] = MAX_SEGMENTATION_ITERATIONS; 	// Maximum number of segmentation iterations 
#   parameters[1] = THRESHOLD_DISTANCE; 		// Max distance of a point from cylinder to be included in cylinder model
