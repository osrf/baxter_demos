#ifndef BAXTER_DEMOS_CLOUD_SEGMENTER_H_
#define BAXTER_DEMOS_CLOUD_SEGMENTER_H_

#include <iostream>
#include <exception>
#include <cmath>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

class CloudSegmenter {
private:
    int radius;
    int filter_min;
    int filter_max;
    int distance_threshold;
    int point_color_threshold;
    int region_color_threshold;
    int min_cluster_size;

    bool has_desired_color;
    bool has_cloud;

    string frame_id;
    pcl::PointRGB desired_color;
    
    pcl::IndicesPtr indices;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pose_pub;

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;

    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_ptrs;
    vector<geometry_msgs::Pose> object_poses;
    tf::TransformListener tf_listener;

public:

    pcl::visualization::CloudViewer cloud_viewer;
    
    bool hasCloud();
    
    bool hasColor();
    static bool isPointWithinDesiredRange(const pcl::PointRGB input_pt,
                               const pcl::PointRGB desired_pt, int radius){

    CloudSegmenter() : cloud_viewer("Cloud viewer"), indices( new vector<int>());
    //CloudSegmenter();
    void publish_poses();
    //remember to shift-click!
    void getClickedPoint(const pcl::visualization::PointPickingEvent& event,
                         void* args);
    void renderCloud();
    void renderClusters();
    pcl::PointRGB getCloudColorAt(int x, int y);
    pcl::PointRGB getCloudColorAt(int n);
   
    void segmentation();
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

#endif
