
#include "pointcloud_processing_server/pointcloud_subtraction.h"

namespace PointcloudSubtraction
{
// Returns cloud MINUEND, minus the points it shares with SUBTRAHEND
    //   the three boolean parameters can be used to force the cloud to subtract points that DON'T match in the fields referenced
    //   ie, if rgb is FALSE and the others are true, and the clouds are XYZRGBNormal, then only normal and XYZ data will be compared
    sensor_msgs::PointCloud2 subtractClouds(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_intensity, bool check_rgb, bool check_normals)
    {
        // If clouds have different numbers of fields, return the input cloud
        if(minuend.fields.size() != subtrahend.fields.size())
        {
            ROS_WARN_STREAM("[PointcloudUtilities] Cloud subtraction requested, but minuend and subtrahend have a different number of fields, at " << minuend.fields.size() << " and " << subtrahend.fields.size() << ", respectively. Returning minuend as difference...");
            return minuend;
        }

        bool minuend_rgb = false;           // Does the minuend cloud contain RGB data??
        bool minuend_intensity = false;     // Does the minuend cloud contain INTENSITY data? 
        bool minuend_normals = false;       // Does the minuend cloud contain NORMALS data? 
        for(int i=0; i<minuend.fields.size(); i++)
        {
            if(minuend.fields[i].name.compare("rgb") == 0)
                minuend_rgb = true;
            else if(minuend.fields[i].name.compare("intensity") == 0)
                minuend_intensity = true;
            else if(minuend.fields[i].name.compare("normal_x") == 0)        // although clouds with normal data actually devote three fields to it (normal_x, normal_y, normal_z, and curvature)...
                minuend_normals = true;
            else if(minuend.fields[i].name.compare("normal_y") != 0)
                if(minuend.fields[i].name.compare("normal_z") != 0)
                    if(minuend.fields[i].name.compare("curvature") != 0)
                        ROS_WARN_STREAM("[PointcloudUtilities] During cloud subtraction, minuend cloud contains an unexpected field, with name " << minuend.fields[i].name << ". This field will be stripped from the output.");
        }

        bool subtrahend_rgb = false;           // Does the subtrahend cloud contain RGB data??
        bool subtrahend_intensity = false;     // Does the subtrahend cloud contain INTENSITY data? 
        bool subtrahend_normals = false;       // Does the subtrahend cloud contain NORMALS data? 
        for(int i=0; i<subtrahend.fields.size(); i++)
        {
            if(subtrahend.fields[i].name.compare("rgb") == 0)
                subtrahend_rgb = true;
            else if(subtrahend.fields[i].name.compare("intensity") == 0)
                subtrahend_intensity = true;
            else if(subtrahend.fields[i].name.compare("normal_x") == 0)        // although clouds with normal data actually devote three fields to it (normal_x, normal_y, normal_z, and curvature)...
                subtrahend_normals = true;
            else if(subtrahend.fields[i].name.compare("normal_y") != 0)
                if(subtrahend.fields[i].name.compare("normal_z") != 0)
                    if(subtrahend.fields[i].name.compare("curvature") != 0)
                        ROS_WARN_STREAM("[PointcloudUtilities] During cloud subtraction, subtrahend cloud contains an unexpected field, with name " << subtrahend.fields[i].name << ". This field will be stripped from the output.");
        }

        if(check_rgb)
        {
            if( minuend_rgb+subtrahend_rgb )                // At least one cloud contains rgb information
            {
                if( !(minuend_rgb*subtrahend_rgb) )             // BUT * NOT BOTH *! That's a problem
                {
                    ROS_WARN_STREAM("[PointcloudUtilities] Cloud subtraction requested with an rgb check, but only one cloud contains rgb information. Minuend rgb: " << minuend_rgb+0 << " Subtrahend rgb: " << subtrahend_rgb+0 << ". Returning minuend...");
                    return minuend;
                }
            }
            else                                            // Neither cloud has rgb information
                check_rgb = false;
        }
        if(check_intensity)
        {
            if( minuend_intensity+subtrahend_intensity )    // At least one cloud contains intensity information
            {
                if( !(minuend_intensity*subtrahend_intensity) ) // BUT * NOT BOTH *! That's a problem
                {
                    ROS_WARN_STREAM("[PointcloudUtilities] Cloud subtraction requested with an intensity check, but only one cloud contains intensity information. Minuend intensity: " << minuend_intensity+0 << " Subtrahend intensity: " << subtrahend_intensity+0 << ". Returning minuend...");
                    return minuend;
                }
            }
            else                                            // Neither cloud has intensity information
                check_intensity = false;
        }
        if(check_normals)
        {
            if( (minuend_normals+subtrahend_normals) )      // At least one cloud contains normals information
            {
                if( !(minuend_normals*subtrahend_normals))      // BUT * NOT BOTH *! That's a problem
                {
                    ROS_WARN_STREAM("[PointcloudUtilities] Cloud subtraction requested with an normals check, but only one cloud contains intensity information. Minuend normals: " << minuend_normals+0 << " Subtrahend normals: " << subtrahend_normals+0 << ". Returning minuend...");
                    return minuend;                    
                }
            }
            else 
                check_normals = false;                      // Neither cloud has normals information
        }

        // Choose a subtraction function, templated on PointType
        if(minuend_rgb || subtrahend_rgb)
        {
            if(minuend_normals || subtrahend_normals)
                return templatedSubtraction<pcl::PointXYZRGBNormal>(minuend, subtrahend, check_rgb, check_intensity, check_normals);      // XYZ, RGB, Normals
            return templatedSubtraction<pcl::PointXYZRGB>(minuend, subtrahend, check_rgb, check_intensity, check_normals);                 // XYZ, RGB
        }
        if(minuend_intensity || subtrahend_intensity)                                           
        {
            if(minuend_normals || subtrahend_normals)
                return templatedSubtraction<pcl::PointXYZINormal>(minuend, subtrahend, check_rgb, check_intensity, check_normals);        // XYZ, I, Normals
            return templatedSubtraction<pcl::PointXYZI>(minuend, subtrahend, check_rgb, check_intensity, check_normals);                   // XYZ, I
        }
        if(minuend_normals || subtrahend_normals)
            return templatedSubtraction<pcl::PointNormal>(minuend, subtrahend, check_rgb, check_intensity, check_normals);                // XYZ, Normals
        return templatedSubtraction<pcl::PointXYZ>(minuend, subtrahend, check_rgb, check_intensity, check_normals);                         // XYZ
    }

    template<typename PointType> 
    sensor_msgs::PointCloud2 PointcloudSubtractor<PointType>::subtractClouds(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	// PCL cloud types
        PCP minuend_ptr(new PC());
        PCP subtrahend_ptr(new PC());
        PCP difference_ptr(new PC());

        // Convert input from ROS message type to PCL
        pcl::fromROSMsg(minuend, *minuend_ptr);
        pcl::fromROSMsg(subtrahend, *subtrahend_ptr);

        // Search structure for NEAREST NEIGHBOR search (in XYZ-space)
        KDTree kdtree;
        kdtree.setInputCloud(minuend_ptr);
        std::vector<float> nearest_dist_squareds;
        std::vector<int> nearest_indices;

        // List of INDICES from MINUEND that match points in SUBTRAHEND - those to be subtracted
        pcl::PointIndices::Ptr indices_to_subtract(new pcl::PointIndices);

        for(int i=0; i<subtrahend_ptr->points.size(); i++)
            if ( kdtree.nearestKSearch (subtrahend_ptr->points[i], 1, nearest_indices, nearest_dist_squareds) > 0 )
                if ( nearest_dist_squareds[0] == 0 )
                    if (comparePoints(minuend_ptr->points[nearest_indices[0]], subtrahend_ptr->points[i], check_rgb, check_intensity, check_normals) )
                        indices_to_subtract->indices.push_back(nearest_indices[0]);
    	
    	// Subtract the matching indices
    	pcl::ExtractIndices<PointType> extract (true); 	
  		extract.setInputCloud(minuend_ptr);
  		extract.setIndices(indices_to_subtract);
  		extract.setNegative(true); 		// Remove points NOT given by indices
  		// Actually segment out plane pointcloud, and set input to match new, smaller cloud
  		extract.filter(*difference_ptr); 

  		ROS_INFO_STREAM("[PointcloudSubtraction] Subtracted two pointclouds." << 
  				 " Minuend cloud size: " << minuend_ptr->points.size() << 
  				"; Subtrahend cloud size: " << subtrahend_ptr->points.size() << 
  				"; Difference cloud size: " << difference_ptr->points.size());
  		sensor_msgs::PointCloud2 difference;
  		pcl::toROSMsg(*difference_ptr, difference);
  		return difference;
    }

    template<>
    bool PointcloudSubtractor<pcl::PointXYZ>::comparePoints(pcl::PointXYZ minuend_point, pcl::PointXYZ subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	return true;
    }

    template<>
    bool PointcloudSubtractor<pcl::PointXYZI>::comparePoints(pcl::PointXYZI minuend_point, pcl::PointXYZI subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_intensity)
    		if(minuend_point.intensity == subtrahend_point.intensity)
    			return true;
    		else
    			return false;
		return true;
    }

    template<>
    bool PointcloudSubtractor<pcl::PointNormal>::comparePoints(pcl::PointNormal minuend_point, pcl::PointNormal subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_normals)
    	{
    		if(minuend_point.normal_x == subtrahend_point.normal_x)
    			if(minuend_point.normal_y == subtrahend_point.normal_y)
    				if(minuend_point.normal_z == subtrahend_point.normal_z)
    					if(minuend_point.curvature == subtrahend_point.curvature)
    						return true;
			return false;
    	}
		return true;
    }

    template<>
    bool PointcloudSubtractor<pcl::PointXYZINormal>::comparePoints(pcl::PointXYZINormal minuend_point, pcl::PointXYZINormal subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_intensity)
    		if(minuend_point.intensity != subtrahend_point.intensity)
    			return false;
    	if(check_normals)
    	{
    		if(minuend_point.normal_x == subtrahend_point.normal_x)
    			if(minuend_point.normal_y == subtrahend_point.normal_y)
    				if(minuend_point.normal_z == subtrahend_point.normal_z)
    					if(minuend_point.curvature == subtrahend_point.curvature)
    						return true;
			return false;
    	}
		return true;	
    }

    template<>
    bool PointcloudSubtractor<pcl::PointXYZRGB>::comparePoints(pcl::PointXYZRGB minuend_point, pcl::PointXYZRGB subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_rgb)
    	{
    		if(minuend_point.r == subtrahend_point.r)
    			if(minuend_point.g == subtrahend_point.g)
    				if(minuend_point.b == subtrahend_point.b)
    					return true;
			return false;
    	}
		return true;
    }

    template<>
    bool PointcloudSubtractor<pcl::PointXYZRGBNormal>::comparePoints(pcl::PointXYZRGBNormal minuend_point, pcl::PointXYZRGBNormal subtrahend_point, bool check_rgb, bool check_intensity, bool check_normals)
    {
    	if(check_rgb)
    	{
    		if(minuend_point.r != subtrahend_point.r)
    			return false;
    		if(minuend_point.g != subtrahend_point.g)
    			return false;
			if(minuend_point.b != subtrahend_point.b)
				return false;
    	}
    	if(check_normals)
    	{
    		if(minuend_point.normal_x == subtrahend_point.normal_x)
    			if(minuend_point.normal_y == subtrahend_point.normal_y)
    				if(minuend_point.normal_z == subtrahend_point.normal_z)
    					if(minuend_point.curvature == subtrahend_point.curvature)
    						return true;
			return false;
    	}
		return true;
    }

    template<typename PointType>
    sensor_msgs::PointCloud2 templatedSubtraction(sensor_msgs::PointCloud2 minuend, sensor_msgs::PointCloud2 subtrahend, bool check_intensity, bool check_rgb, bool check_normals)
    {
        PointcloudSubtractor<PointType> subtractor;
        //return subtractor.subtractClouds(minuend, subtrahend, check_rgb, check_intensity, check_normals);
    }

} // namespace PointcloudSubtraction