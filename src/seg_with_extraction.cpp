#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void segmentation_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_blob (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    // pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Convert to the templated PointCloud
    pcl::fromROSMsg (*input, *cloud_blob);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (0.02f, 0.02f, 0.02f);
    sor.filter (*cloud_filtered);

    // pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->size () > 0.7 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
    pub.publish (*cloud_filtered);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "segmentation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", 1, segmentation_cb);
  
    pub = n.advertise<sensor_msgs::PointCloud2> ("/pcl_new", 1);

    ros::spin();
}