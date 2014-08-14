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

bool isPointWithinDesiredRange(const pcl::PointRGB input_pt,
                               const pcl::PointRGB desired_pt, int radius){
    pcl::PointXYZRGB input_xyz(input_pt.r, input_pt.g, input_pt.b),
                        desired_xyz(desired_pt.r, desired_pt.g, desired_pt.b);
    pcl::PointXYZHSV input_hsv, desired_hsv;
    pcl::PointXYZRGBtoXYZHSV(input_xyz, input_hsv);
    pcl::PointXYZRGBtoXYZHSV(desired_xyz, desired_hsv);
    
    if ( abs(input_hsv.h - desired_hsv.h) < radius &&
         abs(input_hsv.s - desired_hsv.s) < radius &&
         abs(input_hsv.v - desired_hsv.v) < radius){
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
    
    pcl::IndicesPtr indices;
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
    
    bool hasCloud(){
        return has_cloud;
    }
    
    bool hasColor(){
        return has_desired_color;
    }

    CloudSegmenter() : cloud_viewer("Cloud viewer"), indices( new vector<int>()){
        map<string, int> params;
        if(!n.getParam("/object_finder_params", params)){
            throw runtime_error("Couldn't find parameters\n");
        }
        radius = params["radius"];
        filter_min = params["filter_min"];
        filter_max = params["filter_max"];
        distance_threshold = params["distance_threshold"];
        point_color_threshold = params["point_color_threshold"];
        region_color_threshold = params["region_color_threshold"];
        min_cluster_size = params["min_cluster_size"];

        has_desired_color = false;
        has_cloud = false;
        cloud_viewer.registerPointPickingCallback(
                    &CloudSegmenter::getClickedPoint, *this, (void*) NULL );

        sub = n.subscribe("/camera/depth_registered/points", 500,
                                          &CloudSegmenter::callback, this);

        //TODO: add args for topic name
        pose_pub = n.advertise<geometry_msgs::PoseArray>(
                            "/object_tracker/right/goal_poses", 1000);
    }

    void publish_poses(){
        geometry_msgs::PoseArray msg;
        msg.poses = object_poses;
        pose_pub.publish(msg);
    }

    //remember to shift-click!
    void getClickedPoint(const pcl::visualization::PointPickingEvent& event,
                         void* args){
        int n = event.getPointIndex();
        if (n == -1){
            cout << "Got no point" << endl;
            return;
        }

        desired_color = getCloudColorAt(n);
        cout << "Desired color: " << (int) desired_color.r << ", " <<
                (int) desired_color.g << ", " << (int) desired_color.b << endl;
        has_desired_color = true;
        segmentation();
    }

    void renderCloud(){
        cloud_viewer.showCloud(cloud);
    }

    void renderClusters(){
        cloud_viewer.showCloud(colored_cloud);
    }

    pcl::PointRGB getCloudColorAt(int x, int y){
        pcl::PointXYZRGB cur = cloud->at(x, y);
        return pcl::PointRGB(cur.b, cur.g, cur.r);
    }
    pcl::PointRGB getCloudColorAt(int n){
        pcl::PointXYZRGB cur = cloud->at(n);
        return pcl::PointRGB(cur.b, cur.g, cur.r);
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

        reg.setInputCloud (cloud);
        reg.setIndices (indices);
        reg.setSearchMethod (tree);
        reg.setDistanceThreshold (distance_threshold);
        reg.setPointColorThreshold (point_color_threshold);
        reg.setRegionColorThreshold (region_color_threshold);
        reg.setMinClusterSize (min_cluster_size);
     
        vector <pcl::PointIndices> clusters;
        reg.extract (clusters);
     
        colored_cloud = reg.getColoredCloud ();

        vector<pcl::PointIndices> blob_clusters;
        // Select the correct color clouds from the segmentation
        // TODO: this is a major bottleneck. improve performance
        for (int i = 0; i < clusters.size(); i++){
            pcl::PointIndices cluster = clusters[i];

            // Get a representative color in the cluster

            float n = cluster.indices.size();
            float r = 0; float g = 0; float b = 0;
            for (int j = 0; j < n; j++){
                pcl::PointRGB color = getCloudColorAt(cluster.indices[j]);
                r += (float) color.r; //float typecasts suck
                g += (float) color.g;
                b += (float) color.b;
            }
            r /= n; g /= n; b /= n;
            cout << "Average color: " << r << ", " << g << ", " << b << endl;
            pcl::PointRGB avg((unsigned char) b, (unsigned char) g, (unsigned char) r);

            // Check if avg is within the clicked color
            if (isPointWithinDesiredRange(avg, desired_color, radius)){
                blob_clusters.push_back(clusters[i]);
            }
        }

        object_poses = vector<geometry_msgs::Pose>();
        for (int i = 0; i < blob_clusters.size(); i++){
            // Get all points in the segmentation in cloud_iterator
            const pcl::PointCloud<pcl::PointXYZRGB> cloud_subset =
                pcl::PointCloud<pcl::PointXYZRGB>(*cloud, blob_clusters[i].indices);
            //Eigen::Vector4f centroid;
            Eigen::Vector3f centroid;

            //TODO: transform to base frame

            //get the moment of inertia
            pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> inertia;
            inertia.setInputCloud(cloud_subset);
            inertia.compute();

            //this centroid will be off because we get only 3 faces of a cube
            inertia.getMassCenter(centroid);
            geometry_msgs::Point position;
            position.x = centroid(0);
            position.y = centroid(1);
            position.z = centroid(2);

            vector<float> eigenvals(3);
            vector<Eigen::Vector3f> eigenvecs(3);
            //Order is major, middle, minor
            if(!inertia.getEigenVectors(eigenvals[0], eigenvals[1], eigenvals[2]) ||
               !inertia.getEigenValues(eigenvecs[0], eigenvecs[1], eigenvecs[2]) ){
                cout << "Couldn't get valid eigenvectors/values for moment of\
                         inertia calculation" <<endl;
                return;
            }

            const Eigen::Vector3f[3] basevecs = [ Eigen::Vector3f(1, 0, 0),
                        Eigen::Vector3f(0, 1, 0 ), Eigen::Vector3f(0, 0, 1)];
            //Get orientation from eigenvectors/values
            /*float[3] angles;
            Eigen::Matrix3f R = Matrix3d::Identity();
            for (int j = 0; j < 3; j++){
                Eigen::Vector3f p = eigenvecs[j];
                Eigen::Vector3f q = basevecs[j];
                //get angle between p and a base vec
                angle = acos( p.normalized().dot( q.normalized() ) )
                R = R.dot(Eigen::aa(angle, q));
            }*/

            Eigen::Quaternion q = Eigen::Quaternion.Identity();
            for (int j = 0; j < 3; j++){
                Eigen::Quaternion p;
                q*=p.fromTwoVectors(basevecs[j], eigenvecs[j]);
            }

            geometry_msgs::Quaternion orientation;
            orientation.x = q.x;
            orientation.y = q.y;
            orientation.z = q.z;
            orientation.w = q.w;

            geometry_msgs::Pose pose;
            pose.position = position; pose.orientation = orientation;
            object_poses.push_back(pose);

        }

    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

        //Transform the point cloud msg to the base frame

        sensor_msgs::PointCloud2 msg_out;
        tf_listener.transformPointCloud("/base", msg, msg_out);
 

        // Members: float x, y, z; uint32_t rgba
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr new_cloud
                                    (new pcl::PointCloud <pcl::PointXYZRGB>);

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*msg_out, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *new_cloud);
        indices = pcl::IndicesPtr( new vector<int>() );
        pcl::removeNaNFromPointCloud(*new_cloud, *new_cloud, *indices);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (new_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (filter_min, filter_max);
        pass.filter (*indices);


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
            cs.renderCloud();
        }
        cs.publish_poses();
    }

    return 0;
}
