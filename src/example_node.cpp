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
	seg1.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg1.setMethodType (pcl::SAC_RANSAC);
	seg1.setDistanceThreshold (0.01);

        Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
	seg1.setAxis(axis);
	seg1.setEpsAngle(  30.0f * (3.14/180.0f) ); 

	seg1.setInputCloud (cloud_filtered);
	seg1.segment (*inliers, *coefficients);


	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	extract.setInputCloud (cloud_filtered);
	extract.setIndices (inliers);
	extract.setNegative (false);
	extract.filter (*cloud_ransac);
        
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 outputPCL;


        pcl::toPCLPointCloud2( *cloud_ransac ,outputPCL);
        pcl_conversions::fromPCL(outputPCL, output);
        pub_ransac.publish(output);
}

int
main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "my_pcl_tutorial");
	ros::NodeHandle nh;
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
	// Create a ROS publisher for the output point cloud
	pub_ransac = nh.advertise<sensor_msgs::PointCloud2> ("/ransac_objects", 1);
	// Spin
	ros::spin ();
}
