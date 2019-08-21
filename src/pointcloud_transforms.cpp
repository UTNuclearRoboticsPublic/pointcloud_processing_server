
#include "pointcloud_processing_server/pointcloud_transforms.h"


// *** quaternionFromVectors *** 
//   For some reason, the Eigen function fromTwoVectors(), which returns a quaternion rotation between two vectors, seems unstable for vectors that are near-parallel
//   Writing my own implementation that doesn't suck 
//   This finds the normalized quaternion rotation FROM first_vec TO second_vec
Eigen::Quaternion<float> PointcloudUtilities::quaternionFromVectors(Eigen::Vector3f first_vec, Eigen::Vector3f second_vec)
{
	ROS_DEBUG_STREAM("[Rasterizer]   FromTwoVector function inputs - first vector: " << first_vec.x() << " " << first_vec.y() << " " << first_vec.z() << " second vector: "  << second_vec.x() << " " << second_vec.y() << " " << second_vec.z());
	Eigen::Quaternion<float> identity_quat;
	identity_quat.setIdentity();
	// Check whether input vectors are well-conditioned
	if( !std::fpclassify(first_vec.x()) == FP_NORMAL || !std::fpclassify(first_vec.y()) == FP_NORMAL || !std::fpclassify(first_vec.z()) == FP_NORMAL )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, first input vector is poorly conditioned: " << first_vec.x() << " " << first_vec.y() << " " << first_vec.z() << "; returning identity quaternion.");
		return identity_quat;
	}
	if( !std::fpclassify(second_vec.x()) == FP_NORMAL || !std::fpclassify(second_vec.y()) == FP_NORMAL || !std::fpclassify(second_vec.z()) == FP_NORMAL )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, second input vector is poorly conditioned: " << second_vec.x() << " " << second_vec.y() << " " << second_vec.z() << "; returning identity quaternion.");
		return identity_quat;
	}
	// Check whether norms of input vectors are well-conditioned
	float first_vec_norm = first_vec.norm();
	if( !std::fpclassify(first_vec_norm) == FP_NORMAL || first_vec_norm > pow(10,10) )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, norm of first input vector is poorly conditioned: " << first_vec_norm << "; Returning identity quaternion.");
		return identity_quat;
	}
	float second_vec_norm = second_vec.norm();
	if( !std::fpclassify(second_vec_norm) == FP_NORMAL || second_vec_norm > pow(10,10) )
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, norm of second input vector is poorly conditioned: " << second_vec_norm << "; Returning identity quaternion.");
		return identity_quat;
	}

	// Check whether norm of cross product is well-conditioned
	Eigen::Vector3f cross = first_vec.cross(second_vec);
	float norm_of_cross = cross.norm();
	if(fabs(norm_of_cross) > pow(10,30) || fabs(norm_of_cross) < pow(10,-30))
	{
		ROS_WARN_STREAM("[Rasterizer]   During quaternion calculation from two vectors, cross product norm is ill-posed: " << norm_of_cross << "; returning identity quaternion.");
		return identity_quat;
	}
	Eigen::Vector3f unit_normal = cross/norm_of_cross;

	float angle = asin( norm_of_cross/first_vec_norm/second_vec_norm );

	ROS_DEBUG_STREAM("[Rasterizer]   FromTwoVector function outputs - angle: " << angle << " n: " << unit_normal.x() << " " << unit_normal.y() << " " << unit_normal.z() << " norm_of_cross " << norm_of_cross);
	Eigen::Quaternion<float> output( cos(angle/2), sin(angle/2)*unit_normal.x(), sin(angle/2)*unit_normal.y(), sin(angle/2)*unit_normal.z() );

	return output;
}

// *** doTransform ***
//   transforms a pointcloud using an Eigen homogeneous matrix (4x4)
void PointcloudUtilities::doTransform(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &transformed_cloud, Eigen::Matrix4f homogeneous)
{
	// Get Quaternion
	Eigen::Matrix3f rotation;
	rotation << homogeneous(0,0), homogeneous(0,1), homogeneous(0,2),
				homogeneous(1,0), homogeneous(1,1), homogeneous(1,2),
				homogeneous(2,0), homogeneous(2,1), homogeneous(2,2);
	Eigen::Quaternionf quaternion(rotation);
	// Construct Transform Message
	geometry_msgs::TransformStamped transform;
	transform.transform.translation.x = homogeneous(0,3);
	transform.transform.translation.y = homogeneous(1,3);
	transform.transform.translation.z = homogeneous(2,3);
	transform.transform.rotation.z = quaternion.x();
	transform.transform.rotation.y = quaternion.y();
	transform.transform.rotation.z = quaternion.z();
	transform.transform.rotation.w = quaternion.w();
	tf2::doTransform(input_cloud, transformed_cloud, transform);
}

// *** inverseTransform ***
//   transforms a pointcloud using the inverse of an Eigen homogeneous matrix (4x4)
void PointcloudUtilities::inverseTransform(sensor_msgs::PointCloud2 input_cloud, sensor_msgs::PointCloud2 &transformed_cloud, Eigen::Matrix4f homogeneous)
{
	// Get Quaternion
	Eigen::Matrix3f rotation;
	rotation << homogeneous(0,0), homogeneous(0,1), homogeneous(0,2),
				homogeneous(1,0), homogeneous(1,1), homogeneous(1,2),
				homogeneous(2,0), homogeneous(2,1), homogeneous(2,2);
	Eigen::Quaternionf quaternion(rotation);
	// Construct Transform Message
	geometry_msgs::TransformStamped transform;
	transform.transform.translation.x = -homogeneous(0,3);
	transform.transform.translation.y = -homogeneous(1,3);
	transform.transform.translation.z = -homogeneous(2,3);
	transform.transform.rotation.z = quaternion.x();
	transform.transform.rotation.y = quaternion.y();
	transform.transform.rotation.z = quaternion.z();
	transform.transform.rotation.w = -quaternion.w();
	tf2::doTransform(input_cloud, transformed_cloud, transform);
}

// *** translateCloud ***
//   translates a pointcloud using an Eigen Vector of 3 floats
sensor_msgs::PointCloud2 PointcloudUtilities::translateCloud(sensor_msgs::PointCloud2 input_cloud, Eigen::Vector3f translation)
{
	geometry_msgs::TransformStamped transform;
	transform.transform.translation.x = translation(0);
	transform.transform.translation.y = translation(1);
	transform.transform.translation.z = translation(2);

	sensor_msgs::PointCloud2 transformed_cloud;
	tf2::doTransform(input_cloud, transformed_cloud, transform);
	return transformed_cloud;
}

// *** rotateCloud ***
//   rotates a pointcloud using an Eigen Quaternion
sensor_msgs::PointCloud2 PointcloudUtilities::rotateCloud(sensor_msgs::PointCloud2 input_cloud, Eigen::Quaternion<float> quaternion)
{
	geometry_msgs::Quaternion quaternion_msg;
	quaternion_msg.x = float(quaternion.x());
	quaternion_msg.y = float(quaternion.y());
	quaternion_msg.z = float(quaternion.z());
	quaternion_msg.w = float(quaternion.w());
	return rotateCloud(input_cloud, quaternion_msg);
}

// *** rotateCloud ***
//   transforms a pointcloud using a geometry_msgs Quaternion
sensor_msgs::PointCloud2 PointcloudUtilities::rotateCloud(sensor_msgs::PointCloud2 input_cloud, geometry_msgs::Quaternion quaternion)
{
	sensor_msgs::PointCloud2 transformed_cloud;

	geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = input_cloud.header.frame_id;

	transform.transform.rotation = quaternion;
	transform.transform.translation.x = 0.0;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = 0.0;

	tf2::doTransform(input_cloud, transformed_cloud, transform);

	transformed_cloud.header = input_cloud.header;
	return transformed_cloud;
}

// *** doTRANSFORM ***
//   transforms a pointcloud using an Eigen homogeneous matrix (4x4)
sensor_msgs::PointCloud2 PointcloudUtilities::rotatePlaneToXZ(sensor_msgs::PointCloud2 input_plane, std::vector<float> coefficients)
{ 
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	rotatePlaneToXZ(input_plane, coefficients, transform);
}
sensor_msgs::PointCloud2 PointcloudUtilities::rotatePlaneToXZ(sensor_msgs::PointCloud2 input_plane, std::vector<float> coefficients, Eigen::Matrix4f &transform)
{
	// ----------- Finding Rotation ----------- 
	// Output Clouds
	sensor_msgs::PointCloud2 rotated_plane;
	// Rotation --> make normal horizontal
	ROS_INFO_STREAM("coeff size: " << coefficients.size());
	for(int i=0; i<coefficients.size(); i++)
		ROS_INFO_STREAM(i << "th: " << coefficients[i]);
	Eigen::Vector3f input_cloud_norm(coefficients[0], coefficients[1], coefficients[2]);
	Eigen::Vector3f input_cloud_norm_horz(coefficients[0], coefficients[1], 0.0); 
	ROS_DEBUG_STREAM("[Rasterizer] Plane Normal: " << " " << coefficients[0] << " " << coefficients[1] << " " << coefficients[2]);
	//input_cloud_norm_horz.cast<float>();
	Eigen::Quaternion<float> horz_rot;
	horz_rot = quaternionFromVectors(input_cloud_norm, input_cloud_norm_horz);
	ROS_DEBUG_STREAM("[Rasterizer] Plane Rotation to Horz (Quat): " << horz_rot.x() << " " << horz_rot.y() << " " << horz_rot.z() << " " << horz_rot.w());
	// Rotation --> make normal in Y+
	//   Either use positive or negative Y axis, depending on whether plane normal is mostly + or - Y
	//   Otherwise, plane will sometimes get flipped 180 degrees for no reason, since for a plane there's no difference between +/- normal
	//   Would be nice if PCL defaulted to, for example, return the normal which pointed away from the origin, or some other convention,
	//     but as of June 2018 they don't - it's just random (from RANSAC)
	float y_axis_direction;
	if(coefficients[1] > 0)
		y_axis_direction = 1.0;
	else y_axis_direction = -1.0;
	Eigen::Vector3f y_axis(0.0, y_axis_direction, 0.0);
	Eigen::Quaternion<float> onto_y_rot;
	onto_y_rot = quaternionFromVectors(input_cloud_norm_horz, y_axis); 
	ROS_DEBUG_STREAM("[Rasterizer] Plane Rotation to Y (Quat): " << onto_y_rot.x() << " " << onto_y_rot.y() << " " << onto_y_rot.z() << " " << onto_y_rot.w());
	// Build Transform
	Eigen::Quaternion<float> final_rotation = onto_y_rot * horz_rot;
	final_rotation.normalize();
	ROS_DEBUG_STREAM("[Rasterizer] Final Plane Rotation (Quat): " << final_rotation.x() << " " << final_rotation.y() << " " << final_rotation.z() << " " << final_rotation.w());
	// ----------- Performing Rotation -----------
	rotated_plane = rotateCloud(input_plane, final_rotation);  	// transforms input_pc2 into process_message
	rotated_plane.header.stamp = input_plane.header.stamp;
	rotated_plane.header.frame_id = input_plane.header.frame_id;
	ROS_INFO_STREAM("[Rasterizer] Plane cloud rotated. Size is " << rotated_plane.height*rotated_plane.width << "; rotation quaternion coefficients: " << final_rotation.x() << " " << final_rotation.y() << " " << final_rotation.z() << " " << final_rotation.w());

	Eigen::Matrix3f transform_3f = final_rotation.toRotationMatrix();
	transform << 	transform_3f(0,0), transform_3f(0,1), transform_3f(0,2), 0,
					transform_3f(1,0), transform_3f(1,1), transform_3f(1,2), 0,
					transform_3f(2,0), transform_3f(2,1), transform_3f(2,2), 0,
									0, 				   0, 				  0, 0;

	return rotated_plane;
}

// *** minValue ***
//   Finds the MINIMUM value in INPUT_CLOUD for the field FIELD_NAME
float PointcloudUtilities::minValue(sensor_msgs::PointCloud2 input_cloud, char field)
{
	pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
	pcl::fromROSMsg(input_cloud, pcl_cloud);
	float min_value = 1000000;
	if(field == 'x')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].x < min_value)
				min_value = pcl_cloud.points[i].x;
	if(field == 'y')
		for(int i=0; i<pcl_cloud.points.size(); i++)	
			if(pcl_cloud.points[i].y < min_value)
				min_value = pcl_cloud.points[i].y;
	if(field == 'z')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].z < min_value)
				min_value = pcl_cloud.points[i].z;
	if(field == 'i')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].intensity < min_value)
				min_value = pcl_cloud.points[i].intensity;
	return min_value;
}

// *** maxValue ***
// Finds the MAXIMUM value in INPUT_CLOUD for the field FIELD_NAME
float PointcloudUtilities::maxValue(sensor_msgs::PointCloud2 input_cloud, char field)
{
	pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
	pcl::fromROSMsg(input_cloud, pcl_cloud);
	float max_value = -1000000;
	if(field == 'x')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].x > max_value)
				max_value = pcl_cloud.points[i].x;
	if(field == 'y')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].y > max_value)
				max_value = pcl_cloud.points[i].y;
	if(field == 'z')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].z > max_value)
				max_value = pcl_cloud.points[i].z;
	if(field == 'i')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			if(pcl_cloud.points[i].intensity > max_value)
				max_value = pcl_cloud.points[i].intensity;
	return max_value;
}

// *** meanValue ***
// Finds the AVERAGE value in INPUT_CLOUD for the field FIELD_NAME
float PointcloudUtilities::meanValue(sensor_msgs::PointCloud2 input_cloud, char field)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(input_cloud, pcl_cloud);
	float mean_value = 0.0;
	if(field == 'x')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			mean_value += pcl_cloud.points[i].x;
	if(field == 'y')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			mean_value += pcl_cloud.points[i].y;
	if(field == 'z')
		for(int i=0; i<pcl_cloud.points.size(); i++)
			mean_value += pcl_cloud.points[i].z;
	return mean_value / pcl_cloud.points.size();
}

// *** cloudLimits ***
//   Returns a 6-dimensional array of X_MIN, X_MAX, Y_MAX, Y_MIN, Z_MAX, Z_MIN for the cloud INPUT_CLOUD
float * PointcloudUtilities::cloudLimits(sensor_msgs::PointCloud2 input_cloud)
{
	static float limits[6];

	limits[0] = minValue(input_cloud, 'x');
	limits[1] = maxValue(input_cloud, 'x');
	limits[2] = minValue(input_cloud, 'y');
	limits[3] = maxValue(input_cloud, 'y');
	limits[4] = minValue(input_cloud, 'z');
	limits[5] = maxValue(input_cloud, 'z');

	return limits;
}

// *** cloudLimits ***
//   6 return float values are passed by parameter reference 
void PointcloudUtilities::cloudLimits(sensor_msgs::PointCloud2 input_cloud, float* min_x, float* max_x, float* min_y, float* max_y, float* min_z, float* max_z)
{
	*min_x = minValue(input_cloud, 'x');
	*max_x = maxValue(input_cloud, 'x');
	*min_y = minValue(input_cloud, 'y');
	*max_y = maxValue(input_cloud, 'y');
	*min_z = minValue(input_cloud, 'z');
	*max_z = maxValue(input_cloud, 'z');
}

// *** cloudLimits ***
//   6 return float values are passed by parameter reference 
void PointcloudUtilities::cloudLimits(sensor_msgs::PointCloud2 input_cloud, float* min_x, float* max_x, float* min_y, float* max_y, float* min_z, float* max_z, float* min_intensity, float* max_intensity)
{
	*min_x = minValue(input_cloud, 'x');
	*max_x = maxValue(input_cloud, 'x');
	*min_y = minValue(input_cloud, 'y');
	*max_y = maxValue(input_cloud, 'y');
	*min_z = minValue(input_cloud, 'z');
	*max_z = maxValue(input_cloud, 'z');
	*min_intensity = minValue(input_cloud, 'i');
	*max_intensity = maxValue(input_cloud, 'i');
}

// *** translatePlaneToXZ ***
//   Translates a planar pointcloud, assumed to be parallel to XZ, to the XZ plane 
//   Actually, just fixes the centroid of the cloud to be on XZ and the min values in X and Z to be on the origin 
sensor_msgs::PointCloud2 PointcloudUtilities::translatePlaneToXZ(sensor_msgs::PointCloud2 input_cloud)
{ 
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	translatePlaneToXZ(input_cloud, transform);
}
sensor_msgs::PointCloud2 PointcloudUtilities::translatePlaneToXZ(sensor_msgs::PointCloud2 input_cloud, Eigen::Matrix4f &transform)
{
	// ----------- Finding Translation ----------- 
	// Output Clouds
	sensor_msgs::PointCloud2 translated_cloud;
	float min_x = minValue(input_cloud, 'x');
	float max_z = maxValue(input_cloud, 'z');
	float mean_y = meanValue(input_cloud, 'y');
	// Build Transform
	geometry_msgs::TransformStamped cloud_translation;
	cloud_translation.transform.rotation.x = 0;
	cloud_translation.transform.rotation.y = 0;
	cloud_translation.transform.rotation.z = 0;
	cloud_translation.transform.rotation.w = 0;
	cloud_translation.transform.translation.x = -min_x;
	cloud_translation.transform.translation.y = -mean_y;
	cloud_translation.transform.translation.z = -max_z;
	// ----------- Performing Translation -----------
	tf2::doTransform(input_cloud, translated_cloud, cloud_translation);  	// transforms input_pc2 into process_message
	translated_cloud.header.stamp = input_cloud.header.stamp;
	translated_cloud.header.frame_id = input_cloud.header.frame_id;

	transform << 	1, 0, 0, -min_x,
					0, 1, 0, -mean_y,
					0, 0, 1, -max_z,
					0, 0, 0, 1;

	ROS_INFO_STREAM("[Rasterizer] Plane cloud translated. Size is " << translated_cloud.height*translated_cloud.width << "; translation coefficients: " << -min_x << " " << -mean_y << " " << -max_z);
	return translated_cloud;
}


geometry_msgs::Transform PointcloudUtilities::transformMultiplication(geometry_msgs::Transform first_transform, geometry_msgs::Transform second_transform)
{
	// Eigen Quaternion from MSGs
	Eigen::Quaternionf first_quaternion(first_transform.rotation.w,
									    first_transform.rotation.x, 
									    first_transform.rotation.y,
									    first_transform.rotation.z);
	Eigen::Quaternionf second_quaternion(second_transform.rotation.w,
									     second_transform.rotation.x, 
									     second_transform.rotation.y,
									     second_transform.rotation.z);
	Eigen::Matrix3f first_rot_matrix = first_quaternion.toRotationMatrix();
	Eigen::Matrix3f second_rot_matrix = second_quaternion.toRotationMatrix();
	// Eigen Homogeneous Matrices
	Eigen::Matrix4f first_homogeneous, second_homogeneous;
	first_homogeneous <<  first_rot_matrix(0,0),  first_rot_matrix(0,1),  first_rot_matrix(0,2),  first_transform.translation.x,
						  first_rot_matrix(1,0),  first_rot_matrix(1,1),  first_rot_matrix(1,2),  first_transform.translation.y,
						  first_rot_matrix(2,0),  first_rot_matrix(2,1),  first_rot_matrix(2,2),  first_transform.translation.z,
						  0, 					  0, 					  0, 					  1;
	second_homogeneous << second_rot_matrix(0,0), second_rot_matrix(0,1), second_rot_matrix(0,2), second_transform.translation.x,
						  second_rot_matrix(1,0), second_rot_matrix(1,1), second_rot_matrix(1,2), second_transform.translation.y,
						  second_rot_matrix(2,0), second_rot_matrix(2,1), second_rot_matrix(2,2), second_transform.translation.z,
						  0, 					  0, 				      0, 					  1;

 	// Output Eigen
 	Eigen::Matrix4f final_transform = first_homogeneous * second_homogeneous;
 	Eigen::Matrix3f output_rotation;
	output_rotation << final_transform(0,0), final_transform(0,1), final_transform(0,2),
					   final_transform(1,0), final_transform(1,1), final_transform(1,2),
					   final_transform(2,0), final_transform(2,1), final_transform(2,2);
	Eigen::Quaternionf output_quaternion(output_rotation);
	// Ouptut MSGs
	geometry_msgs::Transform output;
	output.translation.x = final_transform(0,3);
	output.translation.y = final_transform(1,3);
	output.translation.z = final_transform(2,3);
	output.rotation.x = output_quaternion.x();
	output.rotation.y = output_quaternion.y();
	output.rotation.z = output_quaternion.z();
	output.rotation.w = output_quaternion.w();

	return output;
}