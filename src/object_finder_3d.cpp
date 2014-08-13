#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

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

#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std; //I don't like using multiple namespaces okaaay

bool isPointWithinDesiredRange(const pcl::PointRGB input_pt, const pcl::PointRGB desired_pt, int radius){
    // unsigned int subtraction--overflow
    pcl::PointXYZRGB input_xyz(input_pt.r, input_pt.g, input_pt.b), desired_xyz(desired_pt.r, desired_pt.g, desired_pt.b);
    pcl::PointXYZHSV input_hsv, desired_hsv;
    pcl::PointXYZRGBtoXYZHSV(input_xyz, input_hsv);
    pcl::PointXYZRGBtoXYZHSV(desired_xyz, desired_hsv);
    
    if ( abs((int)input_hsv.h - (int)desired_hsv.h) < radius &&
         abs((int)input_hsv.s - (int)desired_hsv.s) < radius && abs((int)input_hsv.v - (int)desired_hsv.v) < radius){
        return true;
    }
    return false;
}

class CloudSegmenter{
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
    pcl::PointRGB desired_color;
    
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pose_pub;

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
    vector<geometry_msgs::Pose> object_poses;
    tf::TransformListener tf_listener;

public:

    pcl::visualization::CloudViewer cloud_viewer;
    //pcl::visualization::CloudViewer cluster_viewer;
    
    bool hasCloud(){
        return has_cloud;
    }
    
    bool hasColor(){
        return has_desired_color;
    }

    CloudSegmenter() : cloud_viewer("Cloud viewer"){
        string folder_name;
        if(!n.getParam("/object_tracker/config_folder", folder_name)){
            // probably freak out here
        }
        //YAML::Node doc = YAML::LoadFile(folder_name);
        ifstream fin(folder_name.c_str());
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);

        doc["radius"] >> radius;
        doc["filter_min"] >> filter_min;
        doc["filter_max"] >> filter_max;
        doc["distance_threshold"] >> distance_threshold;
        doc["point_color_threshold"] >> point_color_threshold;
        doc["region_color_threshold"] >> region_color_threshold;
        doc["min_cluster_size"] >> min_cluster_size;

        has_desired_color = false;
        has_cloud = false;
        //cloud_viewer.registerMouseCallback(&CloudSegmenter::getClickedPoint, *this, (void*) NULL);
        cloud_viewer.registerPointPickingCallback(&CloudSegmenter::getClickedPoint, *this, (void*) NULL );

        sub = n.subscribe("/camera/depth_registered/points", 500,
                                          &CloudSegmenter::callback, this);
        //TODO: add args for topic name
        pose_pub = n.advertise<geometry_msgs::PoseArray>("/object_tracker/right/goal_poses", 1000);

        
    }
    void publish_poses(){
        geometry_msgs::PoseArray msg;
        msg.poses = object_poses; //Does this assignment work? We'll see!!
        pose_pub.publish(msg);
    }

    //remember to shift-click!
    void getClickedPoint(const pcl::visualization::PointPickingEvent& event, void* args){

        //pcl::PointXYZ clicked_point;
        //event.getPoint(clicked_point.x, clicked_point.y, clicked_point.z);
        //cout << "Got point: " << clicked_point.x << ", " << clicked_point.y << ", " << clicked_point.z <<endl;
        //desired_color = getCloudColorAt(clicked_point.x, clicked_point.y);

        int n = event.getPointIndex();
        if (n == -1){
            cout << "Got no point" << endl;
            return;
        }

        desired_color = getCloudColorAt(n);
        cout << "Desired color: " << (int) desired_color.r << ", " << (int) desired_color.g << ", " << (int) desired_color.b << endl;
        has_desired_color = true;
        segmentation();
    }

    void renderCloud(){
        cloud_viewer.showCloud(cloud); //Is this okay in this callback...?
    }

    void renderClusters(){
        cloud_viewer.showCloud(colored_cloud);
    }

    pcl::PointRGB getCloudColorAt(int x, int y){
        pcl::PointXYZRGB cur = cloud->at(x, y); //Indexing might be dubious
        return pcl::PointRGB(cur.r, cur.g, cur.b);
    }
    pcl::PointRGB getCloudColorAt(int n){
        pcl::PointXYZRGB cur = cloud->at(n); //Indexing might be dubious
        return pcl::PointRGB(cur.r, cur.g, cur.b);
    }

   
    void segmentation(){
        if (!has_desired_color){
            return;
        }

        /* Segmentation code from:
           http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php*/
        pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
                            boost::shared_ptr<pcl::search::Search
                            <pcl::PointXYZRGB> >
                            (new pcl::search::KdTree<pcl::PointXYZRGB>);

        pcl::IndicesPtr indices (new vector <int>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 4.0);
        pass.filter (*indices);
        //TODO: yaml these parameters 

        reg.setInputCloud (cloud);
        reg.setIndices (indices);
        reg.setSearchMethod (tree);
        reg.setDistanceThreshold (10);
        reg.setPointColorThreshold (6);
        reg.setRegionColorThreshold (5);
        reg.setMinClusterSize (600);
     
        vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
     
        colored_cloud = reg.getColoredCloud ();

        vector<pcl::PointIndices> blob_clusters;
        // Select the correct color clouds from the segmentation
        for (int i = 0; i < clusters.size(); i++){
            pcl::PointIndices cluster = clusters[i];

            // Get a representative color in the cluster

            float n = cluster.indices.size();
            //Hurr indexing through all these points is not so good
            int r = 0; int g = 0; int b = 0;
            for (int j = 0; j < n; j++){
                pcl::PointRGB color = getCloudColorAt(cluster.indices[j]);
                r += (int) color.r;
                g += (int) color.g;
                b += (int) color.b;
            }
            r /= n; g /= n; b /= n;
            cout << "Average color: " << r << ", " << g << ", " << b << endl;
            pcl::PointRGB avg((unsigned char) r, (unsigned char) g, (unsigned char) b);
            //avg = getCloudColorAt(cluster.indices[0]);

            // Check if avg is within the clicked color
            //TODO: get desired_color from click and radius from param
            if (isPointWithinDesiredRange(avg, desired_color, radius)){
                blob_clusters.push_back(clusters[i]);
            }
        }

        object_poses = vector<geometry_msgs::Pose>();
        for (int i = 0; i < blob_clusters.size(); i++){
            // Get all points in the segmentation in cloud_iterator
            const pcl::PointCloud<pcl::PointXYZRGB> cloud_subset = pcl::PointCloud<pcl::PointXYZRGB>(*cloud, blob_clusters[i].indices);
            Eigen::Vector4f centroid;

            //TODO: transform to base frame

            //this centroid will be a BIT off because the shape is only 3 faces of a cube
            pcl::compute3DCentroid(cloud_subset, centroid);
            geometry_msgs::Point position;
            position.x = centroid(0); position.y = centroid(1); position.z = centroid(2);

            //make this smart at some point
            geometry_msgs::Quaternion orientation;
            orientation.x = 0;
            orientation.y = 0;
            orientation.z = 0;
            orientation.w = 1;
            geometry_msgs::PoseStamped pose_in;
            pose_in.pose.position = position; pose_in.pose.orientation = orientation;
            pose_in.header.frame_id = "/camera_depth_optical_frame";
            pose_in.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped pose_out;
            tf_listener.transformPose("/base", pose_in, pose_out);
            //Now transform the pose to the base frame
            object_poses.push_back(pose_out.pose);
            
            // TODO: Pose calculation for cube
            // Get surface normals for cloud_subset
            // Cluster into 3 surfaces
            // Discard the one most parallel to Z, randomly choose the other
        }

    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // Members: float x, y, z; uint32_t rgba
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*msg, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *new_cloud);
        cloud = new_cloud;
        has_cloud = true;
    }
};

int main(int argc, char** argv){
    

    ros::init(argc, argv, "object_finder_3d");

    CloudSegmenter cs;

    ros::Rate loop_rate(100);
    //event loop
    while(ros::ok() && !cs.cloud_viewer.wasStopped()){
        ros::spinOnce();
        loop_rate.sleep();
        if(cs.hasColor()){
            cs.renderClusters();
        } else if(cs.hasCloud()){
            cs.renderCloud(); //only do this once?
        }
        cs.publish_poses();
    }

    return 0;
}
