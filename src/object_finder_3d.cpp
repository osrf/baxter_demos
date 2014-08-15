#include "ros/ros.h"
#include "CloudSegmenter.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "object_finder_3d");

    CloudSegmenter::CloudSegmenter cs;

    ros::Rate loop_rate(100);
    //event loop
    while(ros::ok() && !cs.cloud_viewer.wasStopped()){
        ros::spinOnce();
        loop_rate.sleep();
        if(cs.hasColor()){
            cs.renderClusters();
        } else if(cs.hasCloud()){
            cs.renderCloud();
        }
        cs.publish_poses();
    }

    return 0;
}
