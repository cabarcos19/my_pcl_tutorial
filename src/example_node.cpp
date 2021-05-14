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
ros::Publisher pub;


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
   	pass.setFilterFieldName ("y");
   	pass.setFilterLimits (-1000, 0);
   	pass.filter (*cloud_filtered);
   
    /*
    //RANSAC computation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> inliers;
	//created RandomSampleConsensus object
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
	model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_filtered));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.01);
	ransac.computeModel();
    ransac.getInliers(inliers);
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud (*cloud_filtered, inliers, *cloud_ransac);
    */
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ransac(new pcl::PointCloud<pcl::PointXYZ>);
    // perform ransac planar filtration to remove table top
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


   
	// Perform downsampling
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampling(new pcl::PointCloud<pcl::PointXYZ>);
	sor.setInputCloud (cloud_ransac);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (*cloud_downsampling);
   
    // perform euclidean cluster segmentation to seporate individual objects

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
	
	int j = 0;
	double cluster_separation = 0;
	double centroids_x [cluster_indices.size()];
	double maxPt_vector[cluster_indices.size()]; 
	double minPt_vector[cluster_indices.size()];
	
	//iterate over found clusters
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    	//iterate over the points of the i cluster
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    	{
    		cloud_cluster->points.push_back (cloud_downsampling->points[*pit]);	
    	}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
			
		geometry_msgs::Point centroid;
		centroid.x = 0;
		centroid.y = 0;
		centroid.z = 0;
		
		
		pcl::PointCloud<pcl::PointXYZ>::iterator i;
		
		for (i = cloud_cluster->points.begin(); i < cloud_cluster->points.end(); i++)
		{
			centroid.x += i->x;
			centroid.y += i->y;
			centroid.z += i->z;
		}

		centroid.x = centroid.x / cloud_cluster->size();
		centroid.y = centroid.y / cloud_cluster->size();
		centroid.z = centroid.z / cloud_cluster->size();
		
		//double boundingRadius = 0;
		//pcl::PointCloud<pcl::PointXYZ>::iterator iter;
		//for ( iter = cloud_cluster->points.begin(); iter != cloud_cluster->points.end(); ++iter)
		//{
      		//pcl::PointXYZ currentPoint = *iter;

      		// Measure distance
      		//double radius = sqrt(pow(currentPoint.x - centroid.x, 2) + pow(currentPoint.y - centroid.y, 2) + pow(currentPoint.z - centroid.z, 2));
      		//if (radius > boundingRadius)
      		//{
        		//boundingRadius = radius;
     		//}
    	//}
		
		centroids_x[j] = centroid.x;	
		pcl::PointXYZ minPt,maxPt;
		
		pcl::getMinMax3D(*cloud_cluster,minPt,maxPt);
		
		minPt_vector[j] = minPt.x;
		maxPt_vector[j] = maxPt.x;
		
		
		//std::cout << "Centroid for clustor " << j << ": [" << centroid.x << ", " << centroid.y << ", " << centroid.z << "]" << std::endl;
		//std::cout << "Radious for clustor " << j << ": " << boundingRadius << std::endl;
		//std::cout << "Ancho del clúster " << j << ": " << ancho << std::endl;
		j++;
	}
	
	for(int i = 0;i < j; i++)
	{

		//std::cout << "cluster min and max" << maxPt_vector[i] << minPt_vector[i] << std::endl;
		std::cout << "distance: " << std::abs(minPt_vector[i]) + std::abs(maxPt_vector[i+1]) << std::endl;
		
		//cluster_separation = std::abs(centroids_x[i]) + std::abs(centroids_x[i+1]);
		//std::cout << "Separation between cluster " << i << " and " << i+1 << " : " << cluster_separation << " cm." << std::endl;
		//}
	}
   
	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*cloud_downsampling,output);
    
	// Convert to ROS data type

	// Publish the data
	pub.publish (output);
	
	

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
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);

	// Spin
	ros::spin ();
}
