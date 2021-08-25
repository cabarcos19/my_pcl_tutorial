#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <pcl/common/io.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/Point.h>
#include <pcl/common/common.h>
#include <std_msgs/Float64.h>
#include <my_pcl_tutorial/ClusterArray.h>
ros::Publisher pub;
ros::Publisher pub_one_cluster;
ros::Publisher pub_ransac;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::fromROSMsg(*input,*cloud_pass);
        
    	//create filtered pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
       
    	//Passthough filter
   	
        pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloud_pass);
        pass.setFilterFieldName ("z");
   	pass.setFilterLimits (0.75,1000);
   	pass.filter (*cloud_filtered);
   
        
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    	// perform ransac planar filtration
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg1;
	// Optional
	seg1.setOptimizeCoefficients (true);
	// Mandatory
	seg1.setModelType (pcl::SACMODEL_PLANE);
	seg1.setMethodType (pcl::SAC_RANSAC);
	seg1.setDistanceThreshold (0.01);

	seg1.setInputCloud (cloud_filtered);
	seg1.segment (*inliers, *coefficients);


	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_ransac);
        
        sensor_msgs::PointCloud2 output1;
        pcl::PCLPointCloud2 outputPCL1;


        pcl::toPCLPointCloud2( *cloud_ransac ,outputPCL1);
        pcl_conversions::fromPCL(outputPCL1, output1);
        pub_ransac.publish(output1);

        
	// Perform downsampling
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampling(new pcl::PointCloud<pcl::PointXYZ>);
	sor.setInputCloud (cloud_ransac);
	sor.setLeafSize (0.05, 0.05, 0.05);
	sor.filter (*cloud_downsampling);
        
    	// perform euclidean cluster segmentation

	// Create the KdTree object for the search method of the extraction
	
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_downsampling);

	// create the extraction object for the clusters
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	// specify euclidean cluster parameters
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_downsampling);
	// exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
	ec.extract (cluster_indices);
	
	//iterate over found clusters
        my_pcl_tutorial::ClusterArray CloudClusters;
	sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 outputPCL;
	std::vector<sensor_msgs::PointCloud2> cluster_vec;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    	{
    		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    		//iterate over the points of the i cluster
    		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    		{
    			cloud_cluster->points.push_back (cloud_downsampling->points[*pit]);	
    		}

		pcl::toPCLPointCloud2( *cloud_cluster ,outputPCL);
                pcl_conversions::fromPCL(outputPCL, output);
                CloudClusters.clusters.push_back(output);
                output.header.frame_id = input->header.frame_id;
                pub_one_cluster.publish(output);		

	}
        pub.publish(CloudClusters);
      
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
        pub_one_cluster =  nh.advertise<sensor_msgs::PointCloud2> ("/camera_objects", 1);
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<my_pcl_tutorial::ClusterArray> ("/array_clusters", 1);
	pub_ransac = nh.advertise<sensor_msgs::PointCloud2> ("/ransac_objects", 1);
	// Spin
	ros::spin ();
}
