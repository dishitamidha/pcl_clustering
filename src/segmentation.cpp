#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

ros::Publisher pub; 
// ros::Publisher pub1; 

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{   
    // Defining containers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    //Converting to a form that is processible by PCL
    pcl::fromROSMsg(*input, *cloud);
    // pcl::fromROSMsg(*input, *segmented_cloud);
    // pub.publish (*cloud);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    

    //Ground Plane removal using Ransac Segmentation
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
    }


    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
    for (const auto& idx: inliers->indices){
        std::cerr << idx << "    " << cloud->points[idx].x << " "
                                << cloud->points[idx].y << " "
                                << cloud->points[idx].z << std::endl;}
    
    // Publishing the cloud
    pub.publish (*cloud);
    //  pub1.publish (*segmentedcloud);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_segment");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_normal", 1);
    // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_segmented", 1);

    ros::spin();
}



