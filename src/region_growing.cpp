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
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/search.h>


ros::Publisher pub; 
int random_number(){
  srand((unsigned) time(0));
  int random_number;
 
  random_number = rand() % 255;
  
  return random_number;
  
  
}

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
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



  // Extracting indices
  seg.setInputCloud (voxel_cloud);
  seg.segment (*inliers, *coefficients);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (voxel_cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_f);
    
  
  // KD Tree for searching
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
//   tree->setInputCloud (cloud_f);
  // Normal Estimation
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_f);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  // Region Growing
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud_f);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (5 / 180.0 * M_PI);
  reg.setCurvatureThreshold (3.5);

  std::vector <pcl::PointIndices> cluster_indices;
  reg.extract (cluster_indices);



  // Clustering
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
  
    //   if (j==0){
    //   cloud_f->points[*pit].r = 9;
    //   cloud_f->points[*pit].g = 176;
    //   cloud_f->points[*pit].b = 173;
    // }

    //  else if(j==1){
    //   cloud_f->points[*pit].r = 158;
    //   cloud_f->points[*pit].g = 102;
    //   cloud_f->points[*pit].b = 242;
    // }

    //    else if(j==2){
    //   cloud_f->points[*pit].r = 100;
    //   cloud_f->points[*pit].g = 102;
    //   cloud_f->points[*pit].b = 242;
    // }

    //   else if(j==3){
    //   cloud_f->points[*pit].r = 206;
    //   cloud_f->points[*pit].g = 237;
    //   cloud_f->points[*pit].b = 52;
    // }
    //   else if(j==4){
    //   cloud_f->points[*pit].r = 252;
    //   cloud_f->points[*pit].g = 132;
    //   cloud_f->points[*pit].b = 40;

    // }

    // else if(j==5){
    //   cloud_f->points[*pit].r = 0;
    //   cloud_f->points[*pit].g = 100;
    //   cloud_f->points[*pit].b = 180;

    // }
    cloud_f->points[*pit].r = 255 - j * 20;
    cloud_f->points[*pit].g = 0 + j * 20;
    cloud_f->points[*pit].b = 255 - j * 30;

    cloud_cluster->push_back ((*cloud_f)[*pit]);

    }

   j++;
  }


  std::cout<<j;
  cloud_cluster->header.frame_id = cloud->header.frame_id;
  pub.publish(cloud_cluster);
  
}







    





int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_segment");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_seg_extracted", 1);
    // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_segmented", 1);

    ros::spin();
}