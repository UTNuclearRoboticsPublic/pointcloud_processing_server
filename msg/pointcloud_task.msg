
# ** USE THESE VALUES ** instead of hard-coding ints wherever possible, in case they change
int32 TRANSFORM_TASK 			= 1
int32 CLIPPING_TASK 			= 2
int32 VOXELIZING_TASK 			= 3
int32 PLANE_SEG_TASK 			= 4
int32 CYLINDER_SEG_TASK 		= 5
int32 LINE_SEG_TASK 			= 6
int32 RADIUS_FILTER_TASK 		= 7
int32 STATISTICAL_FILTER_TASK 	= 8


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

#  See github package .readme for a full description of implemented functions and their parameters 

# -------------- Clipping --------------
# Clips the input pointcloud by the rectangular box bounded by these dimensions. 
#   name = "clip";
#   parameters[0] = X_MIN; 				// Boundaries of the bounding box, from cloud origin
#   parameters[1] = X_MAX;
#   parameters[2] = Y_MIN;
#   parameters[3] = Y_MAX;
#   parameters[4] = Z_MIN;
#   parameters[5] = Z_MAX;
#   parameters[6] = X; 				// Pose of the bounding box, relative to cloud frame
#   parameters[7] = Y;
#   parameters[8] = Z;
#   parameters[9] = Roll;
#   parameters[10] = Pitch;
#   parameters[11] = Yaw;

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
