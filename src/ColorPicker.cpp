#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include "geometry_msgs/Point.h"
#include "ros/ros.h"

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/recognition/color_gradient_dot_modality.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/mutex.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointColorCloud;
using namespace std;

class ColorPicker{
private:
    ros::NodeHandle n;
    ros::Subscriber cloud_sub;
    ros::Subscriber color_sub;
    ros::Publisher pub;
    bool has_cloud;
    bool has_desired_color;
    bool segmented;
    PointColorCloud::Ptr cloud;
    PointColorCloud::Ptr segmented_cloud;
    pcl::IndicesPtr indices;
    pcl::PointRGB desired_color;

public:
    boost::mutex cloud_mutex;

    bool hasCloud(){
        return has_cloud;
    }

    bool hasDesiredColor(){
        return has_desired_color;
    }

    bool wasSegmented(){
        return segmented;
    }

    PointColorCloud::Ptr getCloud(){
        return cloud;
    }

    PointColorCloud::Ptr getSegmentedCloud(){
        return segmented_cloud;
    }

    ColorPicker(){
        cloud_sub = n.subscribe("/camera/depth_registered/points", 1000,
                                   &ColorPicker::callback, this);
        color_sub = n.subscribe("/object_tracker/segmented_cloud", 1000,
                                   &ColorPicker::segmented_callback, this);

        pub = n.advertise<geometry_msgs::Point>("/object_tracker/picked_color", 1000);
        has_cloud = false;
        segmented = false;
        has_desired_color = false;
        cloud_mutex.unlock();
        cloud = PointColorCloud::Ptr( new PointColorCloud());
        segmented_cloud = PointColorCloud::Ptr( new PointColorCloud());
    }

    void segmented_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
 
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*msg, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *segmented_cloud);
       
        segmented = true;
    }

    
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        //Update the class cloud ptr
        indices = pcl::IndicesPtr( new vector<int>() );

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*msg, pcl_pc);
        cloud_mutex.lock();
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);
        pcl::removeNaNFromPointCloud(*cloud, *cloud, *indices);
        has_cloud = true;
        cloud_mutex.unlock();

        if(has_desired_color){
            geometry_msgs::Point pub_msg;
            pub_msg.x = (int) desired_color.r;
            pub_msg.y = (int) desired_color.g;
            pub_msg.z = (int) desired_color.b;
            pub.publish(pub_msg);
        }
    }

    pcl::PointRGB getCloudColorAt(int x, int y){
        pcl::PointXYZRGB cur = cloud->at(x, y);
        return pcl::PointRGB(cur.b, cur.g, cur.r);
    }

    pcl::PointRGB getCloudColorAt(size_t n){
        pcl::PointXYZRGB cur = cloud->at(n);
        return pcl::PointRGB(cur.b, cur.g, cur.r);
    }

    //remember to shift-click!
    void getClickedPoint(const pcl::visualization::PointPickingEvent& event,
                         void* args){
        /*int n = event.getPointIndex();
        if (n == -1){
            cout << "Got no point" << endl;
            return;
        }

        desired_color = getCloudColorAt((size_t) n);*/
        
        pcl::search::KdTree<pcl::PointXYZRGB> search;

        search.setInputCloud(cloud);
        pcl::PointXYZRGB picked_pt;
        event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);

        vector<float> distances(1);
        vector<int> search_indices(1);
        search.nearestKSearch (picked_pt, 1, search_indices, distances);
        desired_color = getCloudColorAt( (size_t) search_indices[0]);

        cout << "Desired color: " << (int) desired_color.r << ", " <<
                (int) desired_color.g << ", " << (int) desired_color.b << endl;
        has_desired_color = true;
    }
};

int main(int argc, char** argv){

    ros::init(argc, argv, "color_picker");

    ColorPicker color_picker;
    pcl::visualization::CloudViewer cloud_viewer("Cloud viewer");
    cloud_viewer.registerPointPickingCallback(
                &ColorPicker::getClickedPoint, color_picker, (void*) NULL );

    ros::Rate loop_rate(100);

    while(ros::ok && !cloud_viewer.wasStopped()){
        ros::spinOnce();
        loop_rate.sleep();
        if(color_picker.hasCloud() && color_picker.getCloud() != NULL){
            break;
        }
    }

    color_picker.cloud_mutex.lock();
    PointColorCloud::Ptr cloud = color_picker.getCloud(); 
    
    color_picker.cloud_mutex.unlock();

    while(ros::ok && !cloud_viewer.wasStopped()){
        ros::spinOnce();
        if(color_picker.wasSegmented()){
            cloud_viewer.showCloud(color_picker.getSegmentedCloud());
        } else {
            cloud_viewer.showCloud(cloud);
        }
        loop_rate.sleep();
    }
}
