#include "ros/ros.h"
#include "CloudSegmenter.h"

using namespace baxter_demos;

int main(int argc, char** argv){
    ros::init(argc, argv, "object_finder_3d");
    CloudSegmenter cs;
    pcl::visualization::CloudViewer cloud_viewer("Cloud viewer");
    cloud_viewer.registerPointPickingCallback(
    &CloudSegmenter::getClickedPoint, cs, (void*) NULL );
    ros::Rate loop_rate(100);
    //event loop
    while(ros::ok() && !cloud_viewer.wasStopped()){
        ros::spinOnce();
        loop_rate.sleep();
        if(cs.hasColor()){
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = cs.getClusteredCloudPtr();
            cloud_viewer.showCloud(cloud);
        } else if(cs.hasCloud()){
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = cs.getCloudPtr();
            cloud_viewer.showCloud(cloud);
        }
    }
    return 0;
}
