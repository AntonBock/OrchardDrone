#include <iostream>
#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/surface/mls.h>

//Self made .msg files
#include "pcl_communication/volumetricData.h"
#include "pcl_communication/volumetricDataArray.h"


class remove_planes {
  public:
   remove_planes();
   void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);;

  private:
   ros::NodeHandle nh;
   ros::Publisher pub;
   ros::Subscriber sub;
};

//Subscriber and publisher declares as public in the remove_planes class.
remove_planes::remove_planes(){
  //sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &remove_planes::callback, this); //Note that this topic will only by activated with the filter command: "filters:=pointcloud"
  sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_pcd", 10, &remove_planes::callback, this);
  pub = nh.advertise<pcl_communication::volumetricDataArray>("/volumetric_data", 1); //We define this ourselves and can call it whatever
}

//Callback function which reads input from "/camera/depth/color/points" topic and outputs on "/volumetric_data" topic.
void remove_planes::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::cout<<"\n START   START   START   START   START"<<std::endl;

  //Conversion from Point Cloud (ROS type) to Point Cloud (PCL type)
  std::cout<<"\n DEBUG 1: Point Clound conversion"<<std::endl;
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*msg,*cloud2);

  //Remove NaN points from Point Cloud
  std::cout<<"\n DEBUG 2: Remove NaN points"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_NaN (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (*cloud2, *cloud_NaN);
  std::cout << "cloud_NaN has " << cloud_NaN->points.size() << " data points." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_long (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices_NaN;
  pcl::removeNaNFromPointCloud(*cloud_NaN, *cloud_long, indices_NaN);
  std::cout << "cloud_long has " << cloud_long -> points.size () << " data points." << std::endl;


  //PCL code
  std::cout<<"\n DEBUG 3: START MAIN.CPP"<<std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
	std::cout<<"Voxel filtering"<<std::endl;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud_long);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud_voxel);
  std::cout << "PointCloud after downsampling has: " << cloud_voxel->size ()  << " data points." << std::endl;

  // Create the filtering object: remove noise from the dataset
	std::cout<<"Statistical outlier removal"<<std::endl;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZ>);
  sor.setInputCloud (cloud_voxel);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_sor);
	std::cout << "PointCloud after outlier removal has: " << cloud_sor->size ()  << " data points." << std::endl;

  // Create a KD-Tree for efficient search within the pointcloud
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	std::cout<<"Normal smoothing"<<std::endl;
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> normals;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);

  // Set parameters and apply
  mls.setInputCloud (cloud_sor);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.05);
  mls.process (normals);

  // Reconstruct
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_normals->resize(normals.size());

	for (size_t i = 0; i < normals.points.size(); ++i) {
		  cloud_normals->points[i].x=normals.points[i].x; //error
		  cloud_normals->points[i].y=normals.points[i].y; //error
		  cloud_normals->points[i].z=normals.points[i].z; //error
	}
	std::cout << "PointCloud Normals has: " << cloud_normals->size ()  << " data points." << std::endl;

	// Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.25);

	pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
	int i=0, nr_points = (int) cloud_normals->size ();
  while (cloud_normals->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_normals);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_normals);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*plane);
    *cloud_normals = *plane;
  }

	// Creating the KdTree object for the search method of the extraction
	std::cout << "Extracting clusters" << std::endl;
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_normals);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_normals);
  ec.extract (cluster_indices);


  int j = 0;
  int8_t treeID = 1;
  pcl_communication::volumetricData volumetric_data; //E.g. the info stored in each entry of the array "msg"
  pcl_communication::volumetricDataArray volumetric_msg; //E.g. an array of info
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
   	std::cout << "For loop" << std::endl;
 		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_normals)[*pit]);
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    j++;

    //Volumetric data
    std::cout<<"\n DEBUG 4: Calculating volumetric data"<<std::endl;
		pcl::PointXYZ minPt, maxPt;
  	pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);

		float tree_height = maxPt.y - minPt.y+0.25;
		std::cout << "Height of the tree: " << tree_height << std::endl;
		float tree_width = maxPt.x - minPt.x;
		std::cout << "Width of the tree: " << tree_width << std::endl;

		float pi = 3.14159;
		float tree_volume = pi*(tree_width/2)*(tree_width/2)*tree_height;
		std::cout << "Cylindrical volume of the tree: " << tree_volume << std::endl;

    volumetric_data.tree_height = tree_height;
    volumetric_data.tree_width = tree_width;
    volumetric_data.tree_volume = tree_volume;
    volumetric_data.treeID = treeID;
    treeID = treeID + 1;
    volumetric_msg.volumetricData.push_back(volumetric_data);

  }

  std::cout<<"\n DEBUG 5: Publishing volumetric data!"<<std::endl;
  pub.publish(volumetric_msg);
}


int main (int argc, char** argv) {
  ros::init(argc, argv, "pcl");

  while(ros::ok){
      remove_planes remove_planes_object;
      ros::spin();
  }

return 0;
}
