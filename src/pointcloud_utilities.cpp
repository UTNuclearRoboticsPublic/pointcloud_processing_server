
#include "pointcloud_processing_server/pointcloud_utilities.h"

namespace PointcloudUtilities
{

/* -------------------------------------------------------------------------------------------------------------
                Check Plane Validity 
   -------------------------------------------------------------------------------------------------------------
    Checks whether the coefficients of a plane (Ax+By+Cz+D=0 form) are sufficiently close to a specified target plane
        Checks angle difference between plane orientation and expected orientation
        Checks difference between plane normal distance and expected normal distance
            this is NOT the same as normal distance between the plane and the expected plane
            however if the orientations are close (from the first check) it is an approximation 

   ------------------------------------------------------------------------------------------------------------- */ 
    bool checkPlaneValidity(std::vector<float> exp_coefficients, std::vector<float> coefficients, bool check_orientation, bool check_distance, float angle_threshold, float dist_threshold)
    {
 /*   	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged(); */

        bool good_orientation = true;
        bool good_distance = true;

        // Not sure why, but currently PCL plane segmentation seems to be returning coefficients in the wrong frame? Temp fix:
        coefficients[3] = -coefficients[3];

        // Orientation Check --> ensure wall rotation isn't too far off expected
        if( check_orientation )
        {
            std::vector<float> exp_normal(&exp_coefficients[0],&exp_coefficients[3]);
            std::vector<float> found_normal(&coefficients[0],&coefficients[3]);
            float cross_mag = vectorMagnitude(crossProduct(exp_normal, found_normal));
            // Magnitudes of normal vectors to planes:
            float exp_vector_mag = vectorMagnitude(exp_normal);
            float found_vector_mag = vectorMagnitude(found_normal);
            // theta = asin [ |cross(A,B)| / (|A|*|B|) ]
            float theta = asin ( cross_mag / exp_vector_mag / found_vector_mag ) *180/3.1416;
            //   For positive A, asin(A) will always be positive. A is always positive here because it's the quotient of three Magnitudes!
            //   This means we can do the 'greater than' comparison below safely regardless of angle. 
            //   Note - this means orientation will be 'correct' even if the vector is antiparallel to the expected one 
            //   We correct for this in the distance check below (enforce the distance to be in the right direction)
            //   Note that of course antiparallel normals does NOT defy an orientation constraint for planes - hence not checking here. 
            if ( theta > angle_threshold ) 
            {
            	good_orientation = false;
                ROS_ERROR_STREAM("[PlaneCheck] Bad orientation! Angle offset found to be " << theta << " and threshold is at " << angle_threshold << ".");
            }
        }

        // Distance Check -- Check that wall location isn't too far off expected
        if( check_distance && good_orientation )
        {
            // Because of the way planes are formulated here, we need to scale 'distance' coefficient by magnitude of vector made by first three coefficients.
            // I think in most cases in my usage the vector coefficients form a unit vector already. But still, to be safe!
            float dist_exp = (exp_coefficients[3]) / sqrt( pow(exp_coefficients[0],2) + pow(exp_coefficients[1],2) + pow(exp_coefficients[2],2) );
            float dist_found = (coefficients[3]) / sqrt( pow(coefficients[0],2) + pow(coefficients[1],2) + pow(coefficients[2],2) );

            // Check directionality! Allow for vectors being antiparallel...
            // We only need to do this check if the check_orientation and check_distance requirements are BOTH enforced.
            float directionality = 1.0;
            if (check_orientation)
            {
                std::vector<float> exp_vector(&exp_coefficients[0],&exp_coefficients[3]);
                std::vector<float> found_vector(&coefficients[0],&coefficients[3]);
                // Assuming vectors are roughly parallel (always the case if good_orientation), they are antiparallel if dot(A,B) < 0
                if(dotProduct(exp_vector, found_vector) < 0)
                    directionality = -1,0;
            }
            if ( sqrt(pow(dist_exp-dist_found*directionality,2)) > dist_threshold ) 
            {
            	good_distance = false;
                ROS_ERROR_STREAM("[PlaneCheck] Bad distance! Normal distance found was " << dist_found*directionality << " but expected " << dist_exp << " with threshold difference at " << dist_threshold << ".");
            }
        }
        	
          //return false;
        if(!good_distance || !good_orientation)
        	return false;
        return true; 
    }  

/* -------------------------------------------------------------------------------------------------------------
                Check Cylinder Validity 
   -------------------------------------------------------------------------------------------------------------
    Checks whether the coefficients of a cylinder are sufficiently close to a specified target plane
        coefficients 0 - 2 are the coordinates of some point on the axis
        coefficients 3 - 5 are central axis orientation
        coefficient  6 is the radius
        (cylinder length, ie start/stop point, is not encapsulated)

   ------------------------------------------------------------------------------------------------------------- */ 
    bool checkCylinderValidity(std::vector<float> exp_coefficients, std::vector<float> coefficients, bool check_radius, bool check_orientation, bool check_distance, float radius_threshold, float angle_threshold, float dist_threshold)
    {
    /*    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();  */

        bool good_orientation = true;
        bool good_distance = true;
        bool good_radius = true;

        if( check_radius )
        {
            if ( sqrt(pow(coefficients[6]-exp_coefficients[6],2)) > radius_threshold )
            {
                good_radius = false;
                ROS_ERROR_STREAM("[CylinderCheck] Bad radius! Radius found is " << coefficients[6] << " but expected " << exp_coefficients[6] << " with threshold difference at " << radius_threshold << ".");
            }
        }

        // Orientation Check --> ensure cylinder rotation isn't too far off expected
        if( check_orientation && good_radius )
        {
            float cross_product_x = exp_coefficients[4]*coefficients[5] - exp_coefficients[5]*coefficients[4];
            float cross_product_y = exp_coefficients[5]*coefficients[3] - exp_coefficients[3]*coefficients[5];
            float cross_product_z = exp_coefficients[3]*coefficients[4] - exp_coefficients[4]*coefficients[3];
            float cross_mag = sqrt( pow(cross_product_x,2) + pow(cross_product_y,2) + pow(cross_product_z,2) );
            // Magnitudes of normal vectors to planes:
            float exp_vector_mag = sqrt(pow(exp_coefficients[3],2)+pow(exp_coefficients[4],2)+pow(exp_coefficients[5],2));
            float found_vector_mag = sqrt(pow(coefficients[3],2)+pow(coefficients[4],2)+pow(coefficients[5],2));
            // theta = asin [ cross(A,B) / (|A|*|B|) ]
            float theta = asin ( cross_mag / exp_vector_mag / found_vector_mag ) *180/3.1416;
            //ROS_DEBUG_STREAM("cross_mag: " << cross_mag << " angle_thresh: " << angle_threshold); 
            if ( theta > angle_threshold ) 
            {
                good_orientation = false;
                ROS_ERROR_STREAM("[CylinderCheck] Bad orientation! Angle offset found to be " << theta << " and threshold is at " << angle_threshold << ".");
            }
        }

        if( check_distance && good_radius && good_orientation )
        {
            // Vector perpendicular to both axes:
            std::vector<float> exp_axis(&exp_coefficients[3],&exp_coefficients[6]);
            std::vector<float> found_axis(&coefficients[3],&coefficients[6]);
            std::vector<float> axis_cross = crossProduct(exp_axis, found_axis);

            // Vector which intersects both lines (at the coefficients-specified point)
            std::vector<float> joining_vector;
            for(int i=0; i<3; i++)
                joining_vector.push_back(exp_coefficients[i]-coefficients[i]);

            // Project intersecting vector onto perpendicular vector
            float axis_cross_mag = vectorMagnitude(axis_cross);
            float joining_vector_mag = vectorMagnitude(joining_vector);
            float dot_product = dotProduct(axis_cross, joining_vector);
            std::vector<float> projected = axis_cross;
            projected = scalarMult(projected, dot_product / pow(axis_cross_mag,2));
            float dist = vectorMagnitude(projected);

            if( dist > dist_threshold )
            {
                good_distance = false;
                ROS_ERROR_STREAM("[CylinderCheck] Bad distance! Normal distance found was " << dist << " with threshold difference at " << dist_threshold << ".");
            }
        }
            
        if(!good_distance || !good_orientation || !good_radius)
            return false;
        return true; 
    }  



/* -------------------------------------------------------------------------------------------------------------
                Search For Plane 
   -------------------------------------------------------------------------------------------------------------
    Searches an input PointCloud2 for a list of input primitives with particular coefficients.
        Currently, supports planes and cylinders
        Requires that search order be specified, but checks each primitive of each time against the corresponding outputs...
            IE, if we specify: plane, plane, cylinder, cylinder
            Searches for a plane first, but compares output against both target planes
            Will then search for cylinders only once both planes are found 
            Might try later to make it test other search orders, in the case that it fails to find a primitive 
        Implements a YAML file structure similar to but modified from PCLTaskCreation::processFromYaml

   ------------------------------------------------------------------------------------------------------------- */ 
    pointcloud_processing_server::pointcloud_process searchForPlane(ros::ServiceClient *client, sensor_msgs::PointCloud2 input, std::vector<float> expected_coefficients, bool check_orientation, bool check_distance, float angle_threshold, float dist_threshold, std::string name, int max_segmentation_iterations, float distance_threshold, bool should_publish, std::string topic)
    {
        pointcloud_processing_server::pointcloud_task plane_task;
        plane_task.name = name;
        plane_task.type_ind = 5;
        plane_task.parameters.push_back(max_segmentation_iterations);        
        plane_task.parameters.push_back(distance_threshold);        
        plane_task.should_publish = should_publish;
        plane_task.pub_topic = topic;
        plane_task.remove_cloud = true;

        pointcloud_processing_server::pointcloud_process process;
        process.request.tasks.push_back(plane_task);
        process.request.pointcloud = input;
        process.request.min_cloud_size = 50;

        bool plane_acceptable = false;
        bool process_failed = false;
        
        while ( !plane_acceptable && !process_failed )
        {
            while ( !client->call(process) ) // If we couldn't read output, the service probably isn't up yet --> retry
            {
                ROS_ERROR("[PointcloudUtilities] Service 'pointcloud_service' returned false - may not be up yet. Trying again in 2 seconds...");
                ros::Duration(1).sleep();
            }

            process_failed = process.response.failed;

            if(process_failed)
            {
                ROS_ERROR("[PointcloudUtilities] Plane segmentation process failed during 'searchForPlane' call!");
                break;
            }
            
            if( PointcloudUtilities::checkPlaneValidity(expected_coefficients, process.response.task_results[0].primitive_coefficients, true, true, angle_threshold, dist_threshold) )
            {
                plane_acceptable = true;
                ROS_DEBUG("[PointcloudUtilities] Found an acceptable plane!");
            }
            else
            {
                ROS_DEBUG("[PointcloudUtilities] Found plane is not acceptable. Removing it and trying again...");
                process.request.pointcloud = process.response.task_results[0].remaining_pointcloud;
            }
        }

        if(!process_failed)
            ROS_INFO_STREAM("[PointcloudUtilities] Successfully located an appropriate plane!");

        return process;
    }

    visualization_msgs::Marker makeClippingVisualization(geometry_msgs::Pose clipping_pose, std::vector<float> clipping_box, std::string marker_name, float id, int marker_type)
    {        
        visualization_msgs::Marker marker;
        marker.ns = "basic_shapes";
        marker.id = id;
        marker.type = marker_type;
        marker.action =  visualization_msgs::Marker::ADD;       // the add int is currently 0  
        marker.lifetime = ros::Duration();        // 0 is forever
        marker.scale.x = clipping_box[1]-clipping_box[0];
        marker.scale.y = clipping_box[3]-clipping_box[2];
        marker.scale.z = clipping_box[5]-clipping_box[4];
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.pose = clipping_pose; 
        marker.pose.position.x += clipping_box[0]+marker.scale.x/2;
        marker.pose.position.y += clipping_box[2]+marker.scale.y/2;
        marker.pose.position.z += clipping_box[4]+marker.scale.z/2;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

/*        ROS_DEBUG_STREAM("marker position:    " << marker.pose.position.x << " " << marker.pose.position.y << " " << marker.pose.position.z);
        ROS_DEBUG_STREAM("marker orientation: " << marker.pose.orientation.x << " " << marker.pose.orientation.y << " " << marker.pose.orientation.z << " " << marker.pose.orientation.w);
        ROS_DEBUG_STREAM("marker scale:       " << marker.scale.x << " " << marker.scale.y << " " << marker.scale.z);
*/
        return marker;
    }

/* -------------------------------------------------------------------------------------------------------------
                Clipping Bounds For Plane Cloud
   -------------------------------------------------------------------------------------------------------------
    Finds the pose of a clipping box centered on a given plane (using the plane's coefficients)
        wall_geometry is the list of coefficients for the wall, in the robot frame (Ax + By + Cz + D = 0)
        output is a Pose in /base_link (since this is what the clipping function needs) 
   ------------------------------------------------------------------------------------------------------------- */ 
    geometry_msgs::Pose clippingBoundsPlane(std::vector<float> wall_geometry)
    { 
        geometry_msgs::Pose clipping_box_pose;

        // From coefficients A,B,C,D for the plane, such that Ax + By + Cz + D = 0
        //  1) compose a normal vector <A,B,C> to the plane
        //  2) make it into a unit vector N
        std::vector<float> unit_normal(&wall_geometry[0],&wall_geometry[3]);
        unit_normal = scalarMult(unit_normal,1/vectorMagnitude(unit_normal));

        // Position is set as the point at which the normal vector intersects the plane: N*D
        clipping_box_pose.position.x = unit_normal[0]*wall_geometry[3];
        clipping_box_pose.position.y = unit_normal[1]*wall_geometry[3];
        clipping_box_pose.position.z = unit_normal[2]*wall_geometry[3];

        // Finds a quaternion to rotate the global X-axis onto N:
        clipping_box_pose.orientation = quatFromAxis(unit_normal);

        //ROS_DEBUG_STREAM("unit_normal " << unit_normal[0] << " " << unit_normal[1] << " " << unit_normal[2]);
///return clipping_box_pose;
        // After the above rotation, the X axis will be perpendicular to the plane
        // To fix the object's Y axis as horizontal (parallel to the XY global plane)...

        tf::Quaternion tf_quat(clipping_box_pose.orientation.x, clipping_box_pose.orientation.y, clipping_box_pose.orientation.z, clipping_box_pose.orientation.w);     // tf version of above quaternion

        // Create a vector representing the Y-axis        
        tf::Vector3 y_axis(0,1,0); 
        // Create the Y'-axis resulting from the first rotation found
        tf::Vector3 new_y_axis = tf::quatRotate(tf_quat, y_axis);

        //ROS_DEBUG_STREAM("new_y_axis " << new_y_axis[0] << " " << new_y_axis[1] << " " << new_y_axis[2]);

        // Create a std::vector Z represnting the Z-axis
        std::vector<float> z_axis;
        z_axis.push_back(0.0);
        z_axis.push_back(0.0);
        z_axis.push_back(1.0);
        // Take ( N x Z ), which yields a horizontal vector H within the plane (normal to Z and normal to the plane's normal)
        std::vector<float> horizontal_in_plane = crossProduct(z_axis,unit_normal);

        //ROS_DEBUG_STREAM("horizontal_in_plane " << horizontal_in_plane[0] << " " << horizontal_in_plane[1] << " " << horizontal_in_plane[2]);

        std::vector<float> new_y_axis_std;
        tfToStdVector(new_y_axis, new_y_axis_std);

        //ROS_DEBUG_STREAM("new_y_axis_std " << new_y_axis_std[0] << " " << new_y_axis_std[1] << " " << new_y_axis_std[2]);

        // Find the angle Theta between Y' and H --> this is the angle necessary for the second, correcting rotation 
        std::vector<float> second_rotation_vector = crossProduct(new_y_axis_std,horizontal_in_plane);
        second_rotation_vector = scalarMult(second_rotation_vector,1/vectorMagnitude(second_rotation_vector));
        float second_rotation_angle = angleBetweenVectors(new_y_axis_std,horizontal_in_plane);
        // Build the second quaternion from N and Theta 
        tf::Quaternion second_rotation(sin(second_rotation_angle/2)*second_rotation_vector[0], sin(second_rotation_angle/2)*second_rotation_vector[1], sin(second_rotation_angle/2)*second_rotation_vector[2], cos(second_rotation_angle/2));

        tf::Quaternion full_quat = second_rotation*tf_quat;

       // ROS_DEBUG_STREAM("tf_quat:         " << tf_quat.getAxis().x() << " " << tf_quat.getAxis().y() << " " << tf_quat.getAxis().z() << " " << tf_quat.getW());
      //  ROS_DEBUG_STREAM("second_rotation: " << second_rotation.getAxis().x() << " " << second_rotation.getAxis().y() << " " << second_rotation.getAxis().z() << " " << second_rotation.getW());
      //  ROS_DEBUG_STREAM("full_quat:       " << full_quat.getAxis().x() << " " << full_quat.getAxis().y() << " " << full_quat.getAxis().z() << " " << full_quat.getW());

        quaternionTFToMsg(full_quat, clipping_box_pose.orientation);

        return clipping_box_pose;
    }

/* -------------------------------------------------------------------------------------------------------------
                Clipping Bounds For Cylinder Cloud
   -------------------------------------------------------------------------------------------------------------
    Finds the pose of a clipping box centered on a given cylinder (using the cylinder's coefficients)
        cylinder_geometry is the list of coefficients for the cylinder, in the robot frame
            Coeffs 0-2 are the coordinates of a point on the cylinder axis
            Coeffs 3-5 are the axis definition
            Coeffs 6   is the cylinder radius (not used here)
        outputs a Pose in /base_link (since this is what the clipping function needs) 
   ------------------------------------------------------------------------------------------------------------- */ 
    geometry_msgs::Pose clippingBoundsCylinder(std::vector<float> cylinder_geometry)
    { 
        geometry_msgs::Pose clipping_box_pose;

        clipping_box_pose.position.x = cylinder_geometry[0];
        clipping_box_pose.position.y = cylinder_geometry[1];
        clipping_box_pose.position.z = cylinder_geometry[2];

        std::vector<float> unit_normal(&cylinder_geometry[3],&cylinder_geometry[6]);
        unit_normal = scalarMult(unit_normal,vectorMagnitude(unit_normal));
        //ROS_DEBUG_STREAM("norm: " << unit_normal[0] << " " << unit_normal[1] << " " << unit_normal[2]);

        clipping_box_pose.orientation = quatFromAxis(unit_normal);

        return clipping_box_pose;
    }


    std::vector<float> offsetPlaneCoefficients(std::vector<float> plane_coefficients, std::vector<float> offset)
    {
        std::vector<float> output_coefficients = plane_coefficients;

        // --- Set Translational Offset ---
        //   Make the first three points into a unit normal (changing the fourth point to preserve scale)
        //   Then, fourth point (distance along normal to the plane) can be altered to reflect the translation
        //   Only portion of the translation which matters for the plane is that normal to the plane! 
        // Create a unit normal to the plane
        std::vector<float> normal;
        for(int i=0; i<3; i++)
            normal.push_back(plane_coefficients[i]);
        float normal_mag = vectorMagnitude(normal);
        normal = scalarMult(normal, 1/normal_mag);
        // Alter the plane definition so the first three points are a unit normal, and the last a distance
        for(int i=0; i<3; i++)
            output_coefficients[i] = normal[i];
        output_coefficients[3] = plane_coefficients[3] * normal_mag;
        // Increase the distance to the plane by the projection of the offset onto the plane normal
        std::vector<float> offset_translation;
        for(int i=0; i<3; i++)
            offset_translation.push_back(offset[i]);
        float projected_offset = dotProduct(offset_translation, normal);
        output_coefficients[3] -= projected_offset;


        // --- Set Rotational Offset ---
        //   This part is simpler - just three consecutive rotation matrices multiplied by the first three (axis) values.
        //   Don't need to do anything to modify the normal distance to the plane. 
        std::vector<float> temp_axis;
        // First rotation (about Z)
        temp_axis.push_back(output_coefficients[0]*cos(offset[3]) - output_coefficients[1]*sin(offset[3]));
        temp_axis.push_back(output_coefficients[0]*sin(offset[3]) + output_coefficients[1]*cos(offset[3]));
        temp_axis.push_back(output_coefficients[2]);
        for(int i=0; i<3; i++)
            output_coefficients[i] = temp_axis[i];

        // First rotation (about Y)
        temp_axis[0] = output_coefficients[0]*cos(offset[4]) + output_coefficients[2]*sin(offset[4]);
        temp_axis[1] = output_coefficients[1];
        temp_axis[2] = -output_coefficients[0]*sin(offset[4]) + output_coefficients[2]*cos(offset[4]);
        for(int i=0; i<3; i++)
            output_coefficients[i] = temp_axis[i];

        // First rotation (about X)
        temp_axis[0] = output_coefficients[0];
        temp_axis[1] = output_coefficients[1]*cos(offset[5]) - output_coefficients[2]*sin(offset[5]);
        temp_axis[2] = output_coefficients[1]*sin(offset[5]) + output_coefficients[2]*cos(offset[5]);
        for(int i=0; i<3; i++)
            output_coefficients[i] = temp_axis[i];

        return output_coefficients;
    }


    std::vector<float> offsetCylinderCoefficients(std::vector<float> cylinder_coefficients, std::vector<float> offset)
    {
        std::vector<float> output_coefficients = cylinder_coefficients;

        // --- Set Translational Offset ---
        for(int i=0; i<3; i++)
            output_coefficients[i] -= offset[i];

        // --- Set Rotational Offset ---
        std::vector<float> temp_axis;
        // First rotation (about Z)
        temp_axis.push_back(output_coefficients[3]*cos(offset[3]) - output_coefficients[4]*sin(offset[3]));
        temp_axis.push_back(output_coefficients[3]*sin(offset[3]) + output_coefficients[4]*cos(offset[3]));
        temp_axis.push_back(output_coefficients[5]);
        for(int i=3; i<6; i++)
            output_coefficients[i] = temp_axis[i-3];
        temp_axis[0] = output_coefficients[3]*cos(offset[4]) + output_coefficients[5]*sin(offset[4]);
        temp_axis[1] = output_coefficients[4];
        temp_axis[2] = -output_coefficients[3]*sin(offset[4]) + output_coefficients[5]*cos(offset[4]);
        for(int i=3; i<6; i++)
            output_coefficients[i] = temp_axis[i-3];
        // First rotation (about X)
        temp_axis[0] = output_coefficients[3];
        temp_axis[1] = output_coefficients[4]*cos(offset[5]) - output_coefficients[5]*sin(offset[5]);
        temp_axis[2] = output_coefficients[4]*sin(offset[5]) + output_coefficients[5]*cos(offset[5]);
        for(int i=3; i<6; i++)
            output_coefficients[i] = temp_axis[i-3];

        return output_coefficients;
    }

/* -------------------------------------------------------------------------------------------------------------
                Plane Coefficients From Vector
   -------------------------------------------------------------------------------------------------------------
    Finds the (Ax + By + Cz + D = 0) coefficients for a plane relative to the robot, given a vector describing its geometry 
        robot_pose is the pose of the robot within the /tunnel frame
        wall_geometry is a vector of three 3D points (9 floats) which define the plane in the /tunnel frame
            Usually, probably use two points on the ground (defining a line on the floor) and a point directly above the first (if wall is vertical)    
            However, should be generalizeable to other wall geometries
   ------------------------------------------------------------------------------------------------------------- */ 
    std::vector<float> planeCoefficientsFromPose(std::vector<float> wall_geometry, geometry_msgs::Pose robot_pose)
    { 
        // transform wall_geometry to be relative to robot
        std::vector<float> plane_coefficients = planeCoefficientsFromPoints(wall_geometry);

    }

    std::vector<float> planeCoefficientsFromPoints(std::vector<float> wall_geometry)
    {
        std::vector<float> point_1(&wall_geometry[0],&wall_geometry[3]);
        std::vector<float> point_2(&wall_geometry[3],&wall_geometry[6]);
        std::vector<float> point_3(&wall_geometry[6],&wall_geometry[9]);

        std::vector<float> vector_12 = subtractPoints(point_1, point_2);
        std::vector<float> vector_23 = subtractPoints(point_2, point_3);
        std::vector<float> unit_normal = crossProduct(vector_12, vector_23);
        unit_normal = scalarMult(unit_normal,1/vectorMagnitude(unit_normal));

        std::vector<float> vector_origin;
        for (int i=0; i<3; i++)
            vector_origin.push_back(0);
        std::vector<float> vector_10 = subtractPoints(point_1, vector_origin);
        float normal_dist = vectorMagnitude(projectVector(vector_10,unit_normal));

        std::vector<float> output = unit_normal;
        output.push_back(normal_dist);
        return output;
    }

    std::vector<float> crossProduct(std::vector<float> vector_1, std::vector<float> vector_2)
    {
        std::vector<float> output;
        output.push_back( vector_1[1]*vector_2[2] - vector_1[2]*vector_2[1] );
        output.push_back( vector_1[2]*vector_2[0] - vector_1[0]*vector_2[2] );
        output.push_back( vector_1[0]*vector_2[1] - vector_1[1]*vector_2[0] );
        return output;
    }

    float dotProduct(std::vector<float> vector_1, std::vector<float> vector_2)
    {
        float output = 0;
        if(!(vector_1.size()==vector_2.size()))
            ROS_ERROR_STREAM("[PointcloudUtilities] Subtrated vectors not of the same size.");
        for (int i=0; i<vector_1.size(); i++)
            output += vector_1[i]*vector_2[i];
        return output;
    }

    float vectorMagnitude(std::vector<float> input)
    {
        float magnitude = 0;
        for (int i=0; i<input.size(); i++)
            magnitude += pow(input[i],2);
        return sqrt(magnitude);
    }

    std::vector<float> subtractPoints(std::vector<float> point_1, std::vector<float> point_2)
    {
        std::vector<float> output;
        if(!(point_1.size()==point_2.size()))
            ROS_ERROR_STREAM("[PointcloudUtilities] Subtrated vectors not of the same size.");
        for (int i=0; i<point_1.size(); i++)
            output.push_back(point_1[i]-point_2[i]);
        return output;
    }

    std::vector<float> scalarMult(std::vector<float> input, float scalar)
    {
        for (int i=0; i<input.size(); i++)
            input[i] *= scalar;
        return input;
    }

    std::vector<float> projectVector(std::vector<float> to_project, std::vector<float> projectee)
    {
        float magnitude = dotProduct(to_project, projectee);
        magnitude /= pow(vectorMagnitude(projectee),2);
        std::vector<float> output = scalarMult(projectee,magnitude);
        return output; 
    }

    float angleBetweenVectors(std::vector<float> vector_1, std::vector<float> vector_2)
    {
        float dot_prod = dotProduct(vector_1,vector_2);
        float theta = acos(dot_prod/vectorMagnitude(vector_1)/vectorMagnitude(vector_2));
        return theta;
    }

    float distanceFromPlane(std::vector<float> plane_coefficients, std::vector<float> point)
    {
        float distance = plane_coefficients[0]*point[0] + plane_coefficients[1]*point[1] + plane_coefficients[2]*point[2];
        std::vector<float> normal(&plane_coefficients[0],&plane_coefficients[3]);
        distance /= vectorMagnitude(normal); 
        return distance;
    }

    /* -------------------------------------------------------------------------------------------------------------
                Quaternion From Axis
   -------------------------------------------------------------------------------------------------------------
    Finds the quaternion representing an input Axis's pose, relative to the X Axis
   ------------------------------------------------------------------------------------------------------------- */ 
    geometry_msgs::Quaternion quatFromAxis(std::vector<float> axis)
    {
        geometry_msgs::Quaternion quat_output;      // Output object

        std::vector<float> x_axis;
        x_axis.push_back(1.0);
        x_axis.push_back(0.0);
        x_axis.push_back(0.0);

        std::vector<float> rotation_vector = crossProduct(x_axis,axis);
        float rotation_vector_mag = vectorMagnitude(rotation_vector);
        float rotation_angle;
        if(rotation_vector_mag == 0) 
        {
        // Hard-codes this if input is X Axis - otherwise both the statements within 'else' would divide by zero!
            rotation_angle = 0;
        }
        else
        {
            rotation_vector = scalarMult(rotation_vector,1/rotation_vector_mag);
            rotation_angle = angleBetweenVectors(axis,x_axis);
        }

        //ROS_DEBUG_STREAM("Rot Ang: " << rotation_angle);
       // ROS_DEBUG_STREAM("Rot Vect: " << rotation_vector[0]);
       // ROS_DEBUG_STREAM("Rot Vect: " << rotation_vector[1]);
       // ROS_DEBUG_STREAM("Rot Vect: " << rotation_vector[2]);

        quat_output.x = sin(rotation_angle/2)*rotation_vector[0];
        quat_output.y = sin(rotation_angle/2)*rotation_vector[1];
        quat_output.z = sin(rotation_angle/2)*rotation_vector[2];
        quat_output.w = cos(rotation_angle/2);

        return quat_output;
    }

    void tfToStdVector(tf::Vector3 input, std::vector<float> &output)
    {
        output.clear();
        output.push_back(input[0]);
        output.push_back(input[1]);
        output.push_back(input[2]);
    }

}