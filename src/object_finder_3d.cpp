#include "ros/ros.h"
#include "CloudSegmenter.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "object_finder_3d");
    bool has_cloud = false;
    bool has_color = false;

    CloudSegmenter cs;

    //pcl::visualization::CloudViewer cloud_viewer("Cloud viewer"); 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer( new pcl::visualization::PCLVisualizer("Cloud viewer"));
    cloud_viewer->setBackgroundColor (0, 0, 0);
   //cloud_viewer->addCoordinateSystem(1.0);
    cloud_viewer->initCameraParameters();
    cloud_viewer->registerPointPickingCallback(
                &CloudSegmenter::getClickedPoint, cs, (void*) NULL );
    cloud_viewer->addText("No valid color selected yet.", 10, 10, 1.0, 1.0, 1.0, "color_info");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2;
    ros::Rate loop_rate(100);
    //event loop
    while(ros::ok() && !cloud_viewer->wasStopped()){
        ros::spinOnce();
        cloud_viewer->spinOnce(100);
        //loop_rate.sleep();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        if(cs.wasSegmented()){
            char color_info[128];
            Eigen::Vector3i color = cs.getDesiredColor();
            sprintf(color_info, "Color selected: %d, %d, %d", color[0], color[1], color[2]);
            cloud_viewer->updateText(color_info, 10, 10, 1.0, 1.0, 1.0, "color_info");

            if (!has_color){
                cloud_viewer->removePointCloud("Input cloud");
                rgb2 = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cs.getClusteredCloudPtr());
                cloud_viewer->addPointCloud<pcl::PointXYZRGB>(cs.getClusteredCloudPtr(), rgb, "Cluster cloud");
                has_color = true;
            } else {
                cloud_viewer->updatePointCloud(cs.getClusteredCloudPtr(), "Cluster cloud");
            }

        } else if(cs.hasCloud() && !has_cloud){
            rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cs.getCloudPtr());
            cloud_viewer->addPointCloud<pcl::PointXYZRGB>(cs.getCloudPtr(), rgb, "Input cloud");

            has_cloud = true;
 
        } else if (cs.hasCloud()){
            cloud_viewer->updatePointCloud(cs.getCloudPtr(), "Input cloud");
        }
        cs.publish_poses();
    }

    return 0;
}
