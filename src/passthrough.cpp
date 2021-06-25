#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub; 

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{   
    // Defining containers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    //Converting to a form that is processible by pcl
    pcl::fromROSMsg(*input, *cloud);

    // Passthrough filtering
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*passthrough_cloud);

    // Publishing the cloud
    pub.publish (*passthrough_cloud);


}


int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_passthrough");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl_passthrough", 1);

    ros::spin();
}

