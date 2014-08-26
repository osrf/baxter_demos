#ifndef BAXTER_DEMOS_CLOUD_SEGMENTER_CPP_
#define BAXTER_DEMOS_CLOUD_SEGMENTER_CPP_

#include "CloudSegmenter.h"

#include <pluginlib/class_list_macros.h>

namespace baxter_demos{

PLUGINLIB_DECLARE_CLASS(baxter_demos, CloudSegmenter, baxter_demos::CloudSegmenter, nodelet::Nodelet)

bool CloudSegmenter::isPointWithinDesiredRange(const pcl::PointRGB input_pt,
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

bool CloudSegmenter:: hasCloud(){
    return has_cloud;
}

bool CloudSegmenter:: hasColor(){
    return has_desired_color;
}

bool CloudSegmenter::wasSegmented(){
    return segmented;
}

Eigen::Vector3i CloudSegmenter::getDesiredColor(){
    return Eigen::Vector3i((int) desired_color.r, (int) desired_color.g,
                           (int) desired_color.b);
}

void CloudSegmenter::visualize_old(){
    ros::Rate loop_rate(100);
    pcl::visualization::CloudViewer cloud_viewer("Cloud viewer");
    cloud_viewer.registerPointPickingCallback(&CloudSegmenter::getClickedPoint, *this, (void*) NULL );

    //event loop
    while(ros::ok() && !cloud_viewer.wasStopped()){
        ros::spinOnce();
        loop_rate.sleep();
        //Lock
        //cloud_mutex.lock();
        if(hasColor()){
            //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = getClusteredCloudPtr();
            cloud_viewer.showCloud(colored_cloud);
        } else if(has_cloud){
            //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = getCloudPtr();
            cloud_viewer.showCloud(cloud);
        }
        //Unlock
        //cloud_mutex.unlock();
    }
}

void CloudSegmenter::visualize(){
    cout << "Entering visualization thread" << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer( new pcl::visualization::PCLVisualizer("Cloud viewer"));
    //pcl::visualization::PCLVisualizer cloud_viewer("Cloud viewer");
    cloud_viewer->setBackgroundColor (0, 0, 0);
    cloud_viewer->addCoordinateSystem(1.0);
    cloud_viewer->registerPointPickingCallback(
                &CloudSegmenter::getClickedPoint, *this, (void*) NULL );
    cloud_viewer->addCoordinateSystem(1.0);
    cloud_viewer->initCameraParameters();
    //cloud_viewer->addText("No valid color selected yet.", 10, 10, 1.0, 1.0, 1.0, "color_info");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2;
    ros::Rate loop_rate(100);
    bool displayed_cloud = false;
    bool displayed_color = false;
    //event loop
    PointColorCloud vis;
    PointColorCloud::Ptr vis_cloud;
    while(ros::ok() && !cloud_viewer->wasStopped()){
        loop_rate.sleep();
        if(has_cloud){
            cloud_mutex.lock();
            vis = PointColorCloud(*cloud);
            vis_cloud = vis.makeShared();
            cloud_mutex.unlock();
        }
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        if(wasSegmented()){
            char color_info[128];
            Eigen::Vector3i color = getDesiredColor();
            sprintf(color_info, "Color selected: %d, %d, %d", color[0], color[1], color[2]);
            //cloud_viewer->updateText(color_info, 10, 10, 1.0, 1.0, 1.0, "color_info");

            if (!displayed_color){
                cloud_viewer->removePointCloud("Input cloud");
                rgb2 = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(colored_cloud);
                cloud_viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, rgb2, "Cluster cloud");
                cloud_viewer->initCameraParameters();
                displayed_color = true;
            } else {
                cloud_viewer->updatePointCloud(colored_cloud, "Cluster cloud");
            }

        } else if(has_cloud && !displayed_cloud){
            if (cloud == NULL){
                throw ros::Exception("cloud was null in visualizer");
            }
            cout << "Point cloud test point: " << vis_cloud->at(0) << endl;
            rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(vis_cloud);
            cout << "Adding point cloud" << endl;
            cloud_viewer->addPointCloud<pcl::PointXYZRGB>(vis_cloud, rgb, "Input cloud");
            cout << "Added point cloud" << endl;

            displayed_cloud = true;
 
        } else if (has_cloud){
            cloud_viewer->updatePointCloud(cloud, "Input cloud");
        }
        cloud_viewer->spinOnce();
        //cout << "unlocking" << endl;
    }
}

CloudSegmenter::CloudSegmenter() : has_cloud(false), has_desired_color(false), segmented(false)  {
}

void CloudSegmenter::onInit(){
    //load params from yaml

    //Create visualization thread
    n = getNodeHandle();

    n.getParam("radius", radius);
    n.getParam("filter_min", filter_min);
    n.getParam("filter_max", filter_max);
    n.getParam("distance_threshold", distance_threshold);
    n.getParam("point_color_threshold", point_color_threshold);
    n.getParam("region_color_threshold", region_color_threshold);
    n.getParam("min_cluster_size", min_cluster_size);

    string object_side_str;
    string exclusion_padding_str;
    n.getParam("object_height", object_side_str);
    n.getParam("exclusion_padding", exclusion_padding_str);
    object_side = atof(object_side_str.c_str());
    exclusion_padding = atof(exclusion_padding_str.c_str());

    n.getParam("sample_size", sample_size);

    has_desired_color = false;
    has_cloud = false;
    segmented = false;

    cloud_mutex.unlock();

    visualizer = new boost::thread( boost::bind( &CloudSegmenter::visualize, this) );
    //visualizer->detach();
    //boost::thread visualizer = boost::thread(boost::bind( &CloudSegmenter::visualize, this));

    cloud_sub = n.subscribe("/camera/depth_registered/points", 100,
                                      &CloudSegmenter::points_callback, this);
    
    object_pub = n.advertise<CollisionObjectArray>(
                        "/object_tracker/collision_objects", 100);

    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/modified_points", 200);

    object_sequence = 0;
    cout << "finished initialization" << endl;
}

Eigen::Vector3f positionToVector(geometry_msgs::Point p){
    return Eigen::Vector3f(p.x, p.y, p.z);
}

//Compare goal poses: by greater magnitude

class pose_compare{
    public:
    bool operator()(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b){

        return positionToVector(pose_a.position).norm() <
               positionToVector(pose_b.position).norm();
    }
};


// Construct a vector of MoveIt CollisionObjects for the new poses that came in
// Match the IDs of the old poses with the new ones and construct new poses if necessary
void CloudSegmenter::match_prev_cur_poses(vector<geometry_msgs::Pose> cur_poses,
                            vector<moveit_msgs::CollisionObject>& next_objs,
                            vector<moveit_msgs::CollisionObject>& remove_objs ){


    int prev_len = prev_objs.size();
    int cur_len = cur_poses.size();
    set<geometry_msgs::Pose, pose_compare> added_poses;

    if (prev_len != 0){
        //Map the ID of the CollisionObject from the previous frame to the best match pose
        //from the current frame
        IDPoseMap prev_to_cur_map; //Previous objects are mapped to their closest current pose
        IDObjectMap id_to_obj_map;

        for(int j = 0; j < prev_len; j++){
            id_to_obj_map[prev_objs[j].id] = prev_objs[j];
        }

        for(int i = 0; i < cur_len; i++){
            geometry_msgs::Pose cur_pose = cur_poses[i];
            Eigen::Vector3f cur_point = positionToVector( cur_pose.position );
            float min_d = 1000; //things probably won't get bigger than this?
            //geometry_msgs::Pose argmin;
            moveit_msgs::CollisionObject argmin;
            for(int j = 0; j < prev_len; j++){
                geometry_msgs::Pose prev_pose = prev_objs[j].primitive_poses[0];
                Eigen::Vector3f prev_point = positionToVector( prev_pose.position );
                float d = (prev_point - cur_point).norm();
                if(d < min_d){
                    min_d = d;
                    argmin = prev_objs[j];
                }
            }

            //Enforce uniqueness
            string id = argmin.id;
            IDPoseMap::iterator it = prev_to_cur_map.find( id ); 
            if( it != prev_to_cur_map.end()){ //If argmin is already matched to a pose
                geometry_msgs::Pose old_pose = prev_to_cur_map[id];
                float old_min_d = ( positionToVector( old_pose.position ) - cur_point).norm();
                if ( old_min_d < min_d ){
                    //Don't add it to the map because the old match is better than this one
                    continue;
                }
            }
            prev_to_cur_map[id] = cur_pose;
            added_poses.insert(cur_pose);
        }
        next_objs.clear();
        remove_objs.clear();
        for(IDPoseMap::iterator it = prev_to_cur_map.begin();
                it != prev_to_cur_map.end(); it++ ){
            geometry_msgs::Pose cur_pose = it->second;
            string id = it->first;
            moveit_msgs::CollisionObject cur_obj = id_to_obj_map[id];
            cur_obj.primitive_poses[0] = cur_pose;
            cur_obj.operation = moveit_msgs::CollisionObject::MOVE;
            cur_obj.header.stamp = ros::Time::now();
            next_objs.push_back(cur_obj);
        }

        //For all the poses that did not make it into the new scene, remove them
        for(int i = 0; i < prev_objs.size(); i ++){
            if(prev_to_cur_map.find(prev_objs[i].id) == prev_to_cur_map.end()){
                moveit_msgs::CollisionObject remove_obj = prev_objs[i];
                remove_obj.operation = moveit_msgs::CollisionObject::REMOVE;
                remove_obj.header.stamp = ros::Time::now();
                remove_objs.push_back(remove_obj);
            }
        }
    }

    // For all the current poses that were not matched with a previous pose, add a new CollisionObject

    for(int j = 0; j < cur_poses.size(); j++){
        if(added_poses.find(cur_poses[j]) == added_poses.end()){

            moveit_msgs::CollisionObject new_obj; 
            char id[16];
            sprintf(id, "goal_block_%d", object_sequence);
            new_obj.id=id;
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = object_side;
            primitive.dimensions[1] = object_side;
            primitive.dimensions[2] = object_side;
            new_obj.primitives.push_back(primitive);
            new_obj.primitive_poses.push_back(cur_poses[j]);
            new_obj.operation = moveit_msgs::CollisionObject::ADD;

            new_obj.header.frame_id = frame_id;
            new_obj.header.stamp = ros::Time::now();

            object_sequence++;

            next_objs.push_back(new_obj);
        }
    }
}

void CloudSegmenter:: publish_poses(){
    //geometry_msgs::PoseArray msg;
    //msg.poses = cur_poses;
    CollisionObjectArray msg;
    msg.objects = cur_objs;
    if(cur_objs.empty()){
        cout << "Oops, no objects found!" << endl;
    }
    object_pub.publish(msg);

    if(has_cloud){
        cout << "publishing modified points" << endl;
        cloud_msg.header.frame_id = frame_id;
        cloud_pub.publish(cloud_msg);
    }
}

//remember to shift-click!
void CloudSegmenter:: getClickedPoint(
                const pcl::visualization::PointPickingEvent& event,
                void* args){
    /*int n = event.getPointIndex();
    if (n == -1){
        cout << "Got no point" << endl;
        return;
    }

    desired_color = getCloudColorAt((size_t) n);*/
    
    pcl::search::KdTree<pcl::PointXYZRGB> search;

    search.setInputCloud (cloud);
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

PointColorCloud::ConstPtr CloudSegmenter::getCloudPtr(){
    return cloud;
}

PointColorCloud::ConstPtr CloudSegmenter::getClusteredCloudPtr(){
    return colored_cloud;
}


pcl::PointRGB CloudSegmenter:: getCloudColorAt(int x, int y){
    pcl::PointXYZRGB cur = cloud->at(x, y);
    return pcl::PointRGB(cur.b, cur.g, cur.r);
}

pcl::PointRGB CloudSegmenter:: getCloudColorAt(size_t n){
    pcl::PointXYZRGB cur = cloud->at(n);
    return pcl::PointRGB(cur.b, cur.g, cur.r);
}

OrientedBoundingBox getOBBForCloud(PointColorCloud::Ptr cloud_ptr){

    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> inertia;

    inertia.setInputCloud(cloud_ptr);

    //get the moment of inertia
    inertia.compute();

    //this centroid will be a bit off because we get only 2-3 faces of a cube
    //Get the oriented bounding box around this cluster
    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    inertia.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    inertia.getAABB(min_point_AABB, max_point_AABB);
    return OrientedBoundingBox(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB, min_point_AABB, max_point_AABB);

}

void CloudSegmenter::mergeCollidingBoxes(){
    bool collides = true;
    while(collides){
        collides = false;
        for (int i = 0; i < cloud_ptrs.size(); i++){
            for (int j = 0; j < cloud_ptrs.size(); j++){
                if (cloud_ptrs[i] == cloud_ptrs[j]){
                    continue;
                }
                if (cloud_boxes[cloud_ptrs[i]].collides_with(cloud_boxes[cloud_ptrs[j]]) ){
                    collides=true;
                    //cloud_ptrs[i] = PointColorCloud::Ptr( *cloud_ptrs[i] + *cloud_ptrs[j]));
                    PointColorCloud newcloud = *cloud_ptrs[i] + *cloud_ptrs[j];
                    
                    PointColorCloud::Ptr newptr = newcloud.makeShared();
                    //cloud_ptrs[i] = newptr;
                    //cloud_ptrs[j] = newptr;
                    cloud_boxes.erase(cloud_ptrs[j]);
                    cloud_ptrs.erase(cloud_ptrs.begin()+j);

                    cloud_boxes.erase(cloud_ptrs[i]);
                    cloud_ptrs.erase(cloud_ptrs.begin()+i);

                    //push_back new box
                    cloud_ptrs.push_back(newptr);
                    cloud_boxes[newptr] = getOBBForCloud(cloud_ptrs.back());
                    break;
                }
            }
            if(collides) break;
        }
    }
}

void CloudSegmenter:: segmentation(){

    cout << "Starting segmentation" << endl;
    /* Segmentation code from:
       http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php*/

    //Segmentation is a major bottleneck
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

    //cloud_mutex.lock(); 
    colored_cloud = reg.getColoredCloud();
    //cloud_mutex.unlock(); 

    cloud_ptrs.clear();

    cloud_boxes.clear();
    // Select the correct color clouds from the segmentation
    
    PointColorCloud obstacle_points;

    for (int i = 0; i < clusters.size(); i++){
        pcl::PointIndices cluster = clusters[i];

        // Get a representative color in the cluster
        const int n = cluster.indices.size();
        const int sample_inc = n/sample_size;
            
        pcl::CentroidPoint<pcl::PointXYZRGB> rgb_centroid;
        for (int j = 0; j < n; j+=sample_inc){
            rgb_centroid.add(cloud->at(cluster.indices[j]));
            
        }
        pcl::PointXYZRGB avg_xyz;
        rgb_centroid.get(avg_xyz);
        pcl::PointRGB avg(avg_xyz.b, avg_xyz.g, avg_xyz.r);

        cout << "Average color: " << (int) avg.r << ", " << (int) avg.g <<
                ", " << (int) avg.b << endl;

        // Check if avg is within the clicked color
        PointColorCloud cloud_subset = PointColorCloud(*cloud, cluster.indices);
        PointColorCloud::Ptr cloud_ptr = cloud_subset.makeShared();
        if (isPointWithinDesiredRange(avg, desired_color, radius)){
            cloud_ptrs.push_back(cloud_ptr);
        } else {
            //Accumulate the clusters that are not candidate goal objects
            obstacle_points += *cloud_ptr;
        }
    }

    obstacle_cloud = obstacle_points.makeShared();
    pcl::toROSMsg(*obstacle_cloud, cloud_msg);

    cout << "Clusters found: " << cloud_ptrs.size() << endl;
    if(cloud_ptrs.empty()){
        has_desired_color = false;
        return;
    }

    segmented = true;

    vector<geometry_msgs::Pose> cur_poses;
    vector<OrientedBoundingBox> OBBs;
    for (int i = 0; i < cloud_ptrs.size(); i++){
        OBBs.push_back(getOBBForCloud(cloud_ptrs[i]));
        //cout << "OBB position: " << OBBs.back().get_position() << endl;
        OBBs.back().set_sides( object_side, object_side, object_side);
        cloud_boxes.insert(CloudPtrBoxPair(cloud_ptrs[i], OBBs.back()));
    }

    //Combine poses with intersecting bounding boxes
    mergeCollidingBoxes();

    //For each OBB, extract the pose

    for(CloudPtrBoxMap::iterator it = cloud_boxes.begin(); it != cloud_boxes.end(); it++){
        OrientedBoundingBox box = (OrientedBoundingBox) it->second;
        Eigen::Vector3f position_OBB = box.get_position();
        Eigen::Matrix3f rotational_matrix_OBB = box.get_rotational_matrix();
        
        geometry_msgs::Point position;
        position.x = position_OBB[0];
        position.y = position_OBB[1];
        position.z = position_OBB[2];

        Eigen::Quaternionf q(rotational_matrix_OBB);

        geometry_msgs::Quaternion orientation;
        orientation.w = q.w();
        orientation.x = q.x();
        orientation.y = q.y();
        orientation.z = q.z();

        geometry_msgs::PoseStamped pose_in;
        pose_in.pose.position = position; pose_in.pose.orientation = orientation;
        //cout << "Pose in: " << pose_in.pose << endl;
        pose_in.header.frame_id = frame_id;
        geometry_msgs::PoseStamped pose_out;
        pose_in.header.stamp = ros::Time::now();
        tf_listener.transformPose("/base", pose_in, pose_out);
        //cout << "Pose out: " << pose_out.pose << endl;
        cur_poses.push_back(pose_out.pose);
    }

    vector<moveit_msgs::CollisionObject> next_objs;
    match_prev_cur_poses(cur_poses, next_objs, cur_objs);
    for(int i = 0; i < next_objs.size(); i++){
        cur_objs.push_back(next_objs[i]);
    }
    
    prev_objs = next_objs;

}

void CloudSegmenter::points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    //cout << "got points" << endl;
    frame_id = msg->header.frame_id;
    // Members: float x, y, z; uint32_t rgba
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr new_cloud
                                (new pcl::PointCloud <pcl::PointXYZRGB>);

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *new_cloud);
    indices = pcl::IndicesPtr( new vector<int>() );

    //cout << "Locking in points callback" << endl;
    cloud_mutex.lock();
    pcl::removeNaNFromPointCloud(*new_cloud, *new_cloud, *indices);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (new_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (filter_min, filter_max);
    pass.filter (*indices);

    cloud = new_cloud;
    //cout << "Unlocking in points callback" << endl;
    cloud_mutex.unlock();

    if(!has_cloud){
        cloud_msg = sensor_msgs::PointCloud2(*msg);
    }

    has_cloud = true;

    if(has_desired_color){
        cloud_mutex.lock();
        cout << "segmenting" << endl;
        segmentation();
        cloud_mutex.unlock();
        publish_poses();
    }
}

//just a wrapper to make things more readable
void addComparison(pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond,
                   const char* channel, pcl::ComparisonOps::CompareOp op,
                   float value){
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>(channel, op, value)) );
}

void CloudSegmenter::exclude_object(const geometry_msgs::Pose object, const PointColorCloud::ConstPtr src_cloud, PointColorCloud::Ptr dst_cloud ){

    //Assume the object has object_height dimensions
    //Remove the part of the pointcloud containing the goal object (with a bit of padding)
    const float side = object_side + exclusion_padding*2;

    const float inner_side = object_side - exclusion_padding*2;
    const float outer_side = object_side + exclusion_padding*2;
    
    //Transform to the object frame (so that the center of the object is the origin)
    Eigen::Vector3f position( object.position.x, object.position.y, object.position.y ); //???
    Eigen::Quaternionf orientation(object.orientation.w, object.orientation.x,
                                   object.orientation.y, object.orientation.z);
    Eigen::Isometry3f object_camera_tf; 
    object_camera_tf.translate(position);
    object_camera_tf.rotate(orientation);

    Eigen::Matrix4f object_camera = object_camera_tf.matrix();
    Eigen::Matrix4f camera_object = object_camera.inverse();
    pcl::transformPointCloud(*src_cloud, *dst_cloud, camera_object);

    //apply condition to exclude the cube
    //From http://pointclouds.org/documentation/tutorials/conditional_removal.php
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond(
                                    new pcl::ConditionAnd<pcl::PointXYZRGB>);
    addComparison(range_cond, "x", pcl::ComparisonOps::GT, -side/2);
    addComparison(range_cond, "x", pcl::ComparisonOps::LT,  side/2);
    addComparison(range_cond, "y", pcl::ComparisonOps::GT, -side/2);
    addComparison(range_cond, "y", pcl::ComparisonOps::LT,  side/2);
    addComparison(range_cond, "z", pcl::ComparisonOps::GT, -side/2);
    addComparison(range_cond, "z", pcl::ComparisonOps::LT,  side/2);
    
    /*addComparison(range_cond, "x", pcl::ComparisonOps::GT, -outer_side/2);
    addComparison(range_cond, "x", pcl::ComparisonOps::LT,  outer_side/2);
    addComparison(range_cond, "y", pcl::ComparisonOps::GT, -outer_side/2);
    addComparison(range_cond, "y", pcl::ComparisonOps::LT,  outer_side/2);
    addComparison(range_cond, "z", pcl::ComparisonOps::GT, -outer_side/2);
    addComparison(range_cond, "z", pcl::ComparisonOps::LT,  outer_side/2);

    addComparison(range_cond, "x", pcl::ComparisonOps::LT, -inner_side/2);
    addComparison(range_cond, "x", pcl::ComparisonOps::GT,  inner_side/2);
    addComparison(range_cond, "y", pcl::ComparisonOps::LT, -inner_side/2);
    addComparison(range_cond, "y", pcl::ComparisonOps::GT,  inner_side/2);
    addComparison(range_cond, "z", pcl::ComparisonOps::LT, -inner_side/2);
    addComparison(range_cond, "z", pcl::ComparisonOps::GT,  inner_side/2);*/


    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond); 
    condrem.setInputCloud(dst_cloud);
    condrem.filter(*dst_cloud);

    //transform back
    pcl::transformPointCloud(*dst_cloud, *dst_cloud, object_camera);

}

void CloudSegmenter::exclude_all_objects(vector<geometry_msgs::Pose> cur_poses){
    PointColorCloud transform_cloud_obj(*cloud);
    PointColorCloud::Ptr transform_cloud = transform_cloud_obj.makeShared();
    

    for(int i = 0; i < cur_poses.size(); i++){

        exclude_object(cur_poses[i], transform_cloud, transform_cloud);
    }

    pcl::toROSMsg(*transform_cloud, cloud_msg);
    obstacle_cloud = transform_cloud;
}

void CloudSegmenter::goal_callback(const geometry_msgs::Pose msg){

    PointColorCloud::Ptr transform_cloud(new PointColorCloud);
    
    exclude_object(msg, cloud, transform_cloud);

    //publish it to topic /modified_points
    pcl::toROSMsg(*transform_cloud, cloud_msg);

    obstacle_cloud = transform_cloud;
}
}
#endif
