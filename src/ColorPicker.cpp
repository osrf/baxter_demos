
#include <pcl/visualization/pcl_visualizer.h>
#include "ros/ros.h"

class ColorPicker{
private:
    ros::Subscriber cloud_sub;
    ros::Publisher pub;

public:
    ColorPicker(){
        cloud_sub = ros::subscribe("/camera/depth_registered/points", 1000,
                                   &ColorPicker::callback, this);
        pub = ros::advertise<Point>("", 1000);
    }
}
