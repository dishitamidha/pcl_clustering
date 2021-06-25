#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

class clustering{
    private:
    ros::Subscriber sub;
    ros::Publisher pub;

    public:
    clustering(ros::NodeHandle *nh){
        sub = nh->subscribe("/kinect/depth/points", 1, &clustering::callback, this);
        pub = nh->advertise<sensor_msgs::PointCloud2> ("/pcl_clustering", 1);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input);

    };

void clustering::callback(const sensor_msgs::PointCloud2ConstPtr& input)
{   
  // Defining containers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);	

  //Converting to a form that is processible by PCL
  pcl::fromROSMsg(*input, *cloud);
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Voxel Grid filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.015f, 0.015f, 0.015f);
  sor.filter (*voxel_cloud);

  //Ground Plane removal using Ransac Segmentation
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.03);
  seg.setMaxIterations (1000);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
  seg.setAxis(axis);
  seg.setEpsAngle(0.523599);
  seg.setInputCloud (voxel_cloud);
  seg.segment (*inliers, *coefficients);
  
  // Extracting indices
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (voxel_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_f);
    
  // KD Tree for searching
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_f);

  // Using Euclidean Cluster Extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.07); 
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (200000000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_f);
  ec.extract (cluster_indices);

  cloud_cluster->header.frame_id = cloud_f->header.frame_id;

  //counting numbr of clusters
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    j++;
  }
  
  //generating random r g b values
  int* colors = new int[j * 3 + 1];
  std::cout <<"Number of clusters currently: "<< j << std::endl;
  srand(j);
  for(int i = 0; i < j * 3 ; i++){
      // srand(i);
      colors[i] = rand()%255;
  }
  
  int k = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
      cloud_f->points[*pit].r = colors[k * 3];
      cloud_f->points[*pit].g = colors[k * 3 + 1];
      cloud_f->points[*pit].b = colors[k * 3 + 2];
      cloud_cluster->push_back ((*cloud_f)[*pit]);
      }
    k++;
  }
  pub.publish(cloud_cluster);
  delete [] colors;
}

int main(int argc, char** argv){
    // initialising node
    ros::init(argc, argv, "pcl_cluster");
    ros::NodeHandle nh;
    clustering obj =  clustering(&nh);
    ros::spin();
}