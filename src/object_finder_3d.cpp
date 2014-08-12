#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

using namespace std;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    //Convert msg to a pointcloud

    // Members: float x, y, z; uint32_t rgba
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    //Convert pointcloud to 2D image (qworg does this, I feel iffy about using it)
    //Color segment the 2D image using OpenCV
    //alternatively we could color segment it using PCL? Must investigate
    //http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
                        boost::shared_ptr<pcl::search::Search
                        <pcl::PointXYZRGB> >
                        (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setIndices(indices);

    //TODO: Parameterize
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
 
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

    //Get all points in the segmentation in cloud_iterator
    //pcl::compute3DCentroid(&cloud_iterator, &centroid)

    //axis calculation:
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_finder_3d");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000,
                                      callback);
    //ros::Publisher centroid_pub = 

    ros::spin();

    return 0;
}
