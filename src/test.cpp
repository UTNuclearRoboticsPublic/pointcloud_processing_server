
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>

#include <pcl/features/normal_3d.h>

#include <pcl/features/fpfh.h>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <geometry_msgs/Transform.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "wall_change_tester");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    // ----------------------------- Pull in data -----------------------------
    std::string bag_name_dense = "/home/conor/ros_data/Fake_Walls/Segmented/screwy/1dps.bag";
    std::string bag_topic_dense = "/target_wall";
    ROS_INFO_STREAM("[Tester] Loading clouds from bag files, using bag name: " << bag_name_dense << " and topic name: " << bag_topic_dense << ".");
    sensor_msgs::PointCloud2 input_msg_dense;
	// Open Bag
    rosbag::Bag input_bag_dense; 
    input_bag_dense.open(bag_name_dense, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics_dense;
    topics_dense.push_back(bag_topic_dense);
    rosbag::View view_dense(input_bag_dense, rosbag::TopicQuery(topics_dense));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view_dense)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            input_msg_dense = *cloud_ptr;
        else
          ROS_ERROR_STREAM("[Tester] Cloud caught for first cloud is null...");
    }
    input_bag_dense.close();
    ROS_INFO_STREAM("[Tester] First cloud size: " << input_msg_dense.height*input_msg_dense.width);


    // ----------------------------- Pull in data -----------------------------
    std::string bag_name_sparse = "/home/conor/ros_data/Fake_Walls/Segmented/screwy/5dps.bag";
    std::string bag_topic_sparse = "/target_wall";
    ROS_INFO_STREAM("[Tester] Loading clouds from bag files, using bag name: " << bag_name_sparse << " and topic name: " << bag_topic_sparse << ".");
    sensor_msgs::PointCloud2 input_msg_sparse;
	// Open Bag
    rosbag::Bag input_bag_sparse; 
    input_bag_sparse.open(bag_name_sparse, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics_sparse;
    topics_sparse.push_back(bag_topic_sparse);
    rosbag::View view_sparse(input_bag_sparse, rosbag::TopicQuery(topics_sparse));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view_sparse)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            input_msg_sparse = *cloud_ptr;
        else
          ROS_ERROR_STREAM("[Tester] Cloud caught for first cloud is null...");
    }
    input_bag_sparse.close();
    ROS_INFO_STREAM("[Tester] First cloud size: " << input_msg_sparse.height*input_msg_sparse.width);




    // ----------------------------- Compute correspondences -----------------------------
    // --------- Inputs ---------
    ROS_INFO_STREAM("[Tester] Converting messages to PCL clouds");
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_dense(new pcl::PointCloud<pcl::PointXYZ>);
   	pcl::fromROSMsg(input_msg_dense, *input_cloud_dense);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_sparse(new pcl::PointCloud<pcl::PointXYZ>);
   	pcl::fromROSMsg(input_msg_sparse, *input_cloud_sparse);

   	ROS_INFO_STREAM("[Tester] Input cloud sizes: " << input_cloud_dense->points.size() << " and " << input_cloud_sparse->points.size());

   //	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

   	// Remove NaNs from cloud (otherwise norm calc may fail)
	std::vector<int> index_source;
	pcl::removeNaNFromPointCloud(*input_cloud_dense, *input_cloud_dense, index_source);
	pcl::removeNaNFromPointCloud(*input_cloud_sparse, *input_cloud_sparse, index_source);
   	ROS_INFO_STREAM("[Tester] Input cloud sizes: " << input_cloud_dense->points.size() << " and " << input_cloud_sparse->points.size());

   	// --------- Normals ---------
   	ROS_INFO_STREAM("[Tester] Generating Normal clouds");
   	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_dense(new pcl::PointCloud<pcl::Normal>);
   	pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_sparse(new pcl::PointCloud<pcl::Normal>);
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
	//norm_est.setSearchMethod(tree);
	//norm_est.setKSearch(12);
	//norm_est.setInputCloud(input_cloud_dense);
	//norm_est.compute(*normals_cloud_dense);
	//norm_est.setInputCloud(input_cloud_sparse);
	//norm_est.compute(*normals_cloud_sparse);
   	ros::Duration(2.0).sleep();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0; i<100; i++)
		for(int j=0; j<100; j++)
		{
			pcl::PointXYZ point;
			point.x = i/100;
			point.y = j/100;
			point.z = 0;
			cloud->points.push_back(point);
		}

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);
	ROS_INFO_STREAM("florp");
	ros::Duration(2.0).sleep();

   	// --------- FPFH ---------
   	ROS_INFO_STREAM("[Tester] Creating FPFH clouds");
   	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
   	fpfh.setSearchMethod(tree);
   	fpfh.setRadiusSearch(0.03);
   	// Dense
   	fpfh.setInputCloud(input_cloud_dense);
   	fpfh.setInputNormals(normals_cloud_dense);
   	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_dense(new pcl::PointCloud<pcl::FPFHSignature33>());
   	fpfh.compute(*fpfh_dense);
   	// Sparse
	fpfh.setInputCloud(input_cloud_sparse);
   	//fpfh.setInputNormals(normals_cloud_sparse);
   	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_sparse(new pcl::PointCloud<pcl::FPFHSignature33>());
   	fpfh.compute(*fpfh_sparse);

   	// --------- Correspondences ---------
   	ROS_INFO_STREAM("[Tester] Generating correspondences");
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr;
    corr.setInputSource(fpfh_dense);
    corr.setInputTarget(fpfh_sparse);
    pcl::Correspondences correspondences;
    pcl::Correspondences correspondences_reciprocal;
    corr.determineCorrespondences(correspondences);
    corr.determineReciprocalCorrespondences(correspondences_reciprocal);

    ROS_INFO_STREAM("[Tester] Donezos");

}