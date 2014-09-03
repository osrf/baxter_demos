#ifndef BAXTER_DEMOS_CLOUD_SEGMENTER_CPP_
#define BAXTER_DEMOS_CLOUD_SEGMENTER_CPP_

#include "CloudSegmenter.h"

#include <pluginlib/class_list_macros.h>

#include "pcl_ros/transforms.h"

namespace baxter_demos{

PLUGINLIB_DECLARE_CLASS(baxter_demos, CloudSegmenter, baxter_demos::CloudSegmenter, nodelet::Nodelet)


bool pose_compare::operator()(geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b){

    return positionToVector(pose_a.position).norm() <
           positionToVector(pose_b.position).norm();
}

bool CloudSegmenter::isPointWithinDesiredRange(const pcl::PointRGB input_pt,
                               const pcl::PointRGB desired_pt, int radius){
    pcl::PointXYZRGB input_xyz(input_pt.r, input_pt.g, input_pt.b),
                        desired_xyz(desired_pt.r, desired_pt.g, desired_pt.b);
    pcl::PointXYZHSV input_hsv, desired_hsv;
    pcl::PointXYZRGBtoXYZHSV(input_xyz, input_hsv);
    pcl::PointXYZRGBtoXYZHSV(desired_xyz, desired_hsv);
    
    if ( abs(((int) input_hsv.h) - ((int) desired_hsv.h)) < radius && //this is not lisp
         abs(((int) input_hsv.s) - ((int) desired_hsv.s)) < radius &&
         abs(((int) input_hsv.v) - ((int) desired_hsv.v)) < radius){
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

CloudSegmenter::CloudSegmenter() : has_cloud(false), has_desired_color(false), segmented(false)  {
    cloud = PointColorCloud::Ptr(new PointColorCloud);
}

void CloudSegmenter::updateParams(){
    //load params from yaml
    double object_height;

    n.getParam("radius", radius);
    n.getParam("filter_min", filter_min);
    n.getParam("filter_max", filter_max);
    n.getParam("distance_threshold", distance_threshold);
    n.getParam("point_color_threshold", point_color_threshold);
    n.getParam("region_color_threshold", region_color_threshold);
    n.getParam("min_cluster_size", min_cluster_size);
    n.getParam("max_cluster_size", max_cluster_size);
    double l;
    n.getParam("leaf_size", l);
    leaf_size = (float) l;
    n.getParam("exclusion_padding", exclusion_padding);
    n.getParam("tolerance", tolerance);
    n.getParam("object_height", object_height);
    n.getParam("min_neighbors", min_neighbors);
    n.getParam("outlier_radius", outlier_radius);
    
    n.getParam("sample_size", sample_size);

    object_side =(float) (object_height + exclusion_padding);

}

void CloudSegmenter::onInit(){
    n = getNodeHandle();
    updateParams();

    has_desired_color = false;
    has_cloud = false;
    segmented = false;
    published_goals = false;

    cloud_sub = n.subscribe("/camera/depth_registered/points", 100,
                                      &CloudSegmenter::points_callback, this);

    color_sub = n.subscribe("/object_tracker/picked_color", 1000,
                                      &CloudSegmenter::color_callback, this);
    
    object_pub = n.advertise<CollisionObjectArray>(
                        "/object_tracker/collision_objects", 100);
    //object_pub = n.advertise<moveit_msgs::CollisionObject>("/collision_object", 100);
    goal_pub = n.advertise<geometry_msgs::PoseArray>("/object_tracker/right/goal_poses", 100);

    //cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/modified_points", 200);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/object_tracker/segmented_cloud", 1000);

    object_sequence = 0;
    cout << "finished initialization" << endl;
}

Eigen::Vector3f positionToVector(geometry_msgs::Point p){
    return Eigen::Vector3f(p.x, p.y, p.z);
}


moveit_msgs::CollisionObject CloudSegmenter::constructCollisionObject(geometry_msgs::Pose pose){
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
    new_obj.primitive_poses.push_back(pose);

    new_obj.operation = moveit_msgs::CollisionObject::ADD;

    new_obj.header.frame_id = "/base";
    new_obj.header.stamp = ros::Time::now();

    object_sequence++;
    return new_obj;
}

void CloudSegmenter::match_objects(vector<geometry_msgs::Pose> cur_poses){
//this matching between frames business is super buggy
    prev_diffs = cur_diffs;
    cur_diffs.clear();

    const float delta_max = 0.09;

    //Match each pose in cur_poses to a unique object in all_objects

    IDPoseMap matched_objects; //Key on old object IDs
    for(int i = 0 ; i < cur_poses.size(); i++){
        Eigen::Vector3f new_point = positionToVector(cur_poses[i].position);

        for(IDObjectMap::iterator it = all_objects.begin();
                                  it != all_objects.end(); it++){
            moveit_msgs::CollisionObject old_object = it->second;
            Eigen::Vector3f old_point = positionToVector(it->second.primitive_poses[0].position);
            float dist_i = (old_point-new_point).norm();
            if( dist_i < delta_max){
                //match them
                if(matched_objects.find(old_object.id) == matched_objects.end() ){
                    matched_objects[old_object.id] = cur_poses[i];
                } else {
                    Eigen::Vector3f map_point = positionToVector(matched_objects[old_object.id].position);
                    float dist_j = (old_point - map_point).norm();
                    if( dist_i < dist_j ){
                        matched_objects[old_object.id] = cur_poses[i]; }
                }
            }
        }
    }

    //Now, check the matched poses

    for(int i = 0; i < cur_poses.size(); i++){
        geometry_msgs::Pose cur_pose = cur_poses[i];
        //PoseIDMap::iterator it = matched_poses.find(cur_pose);
        IDPoseMap::iterator it;
        for(it = matched_objects.begin();
            it != matched_objects.end(); it++){
            if(pose_compare::position_equal(it->second, cur_pose) ){
                //For all the poses that are in matched_objects:
                //If they were in prev_diffs with the ops ADD or MOVE,
                //add them to cur_diffs with the op MOVE
                //else, add them with the op ADD
                moveit_msgs::CollisionObject matched_obj = all_objects[it->first];
                matched_obj.primitive_poses[0] = cur_pose;
                if(prev_diffs.find(matched_obj.id) != prev_diffs.end() &&
                   prev_diffs[matched_obj.id].operation !=
                                    moveit_msgs::CollisionObject::REMOVE ){
                    matched_obj.operation = moveit_msgs::CollisionObject::MOVE;
                } /*else {
                    matched_obj.operation = moveit_msgs::CollisionObject::ADD;
                }*/ //XXX test
                cur_diffs[matched_obj.id] = matched_obj;
            }
            break;
        }
        if(it == matched_objects.end()){
            //Make new objects for the poses that are not keys in matched_objects
            //and add them to cur_diffs with op ADD

            moveit_msgs::CollisionObject new_obj = constructCollisionObject(cur_pose);
            cur_diffs[new_obj.id] = new_obj;
        } 
    }
    
    //For all poses in prev_diffs that are not in matched_objects
    //and do not have operation REMOVE:
    //add them to cur_diffs with the operation REMOVE

    for(IDObjectMap::iterator it = prev_diffs.begin(); it != prev_diffs.end(); it++){
        if(matched_objects.find(it->first) == matched_objects.end() &&
           it->second.operation != moveit_msgs::CollisionObject::REMOVE){
            moveit_msgs::CollisionObject remove_obj = it->second;
            remove_obj.operation = moveit_msgs::CollisionObject::REMOVE;
            cur_diffs[remove_obj.id] = remove_obj;
        }
    }

    //Now update each object's state in all_objects
    for(IDObjectMap::iterator it = cur_diffs.begin(); it != cur_diffs.begin(); it++){
        all_objects[it->first] = it->second;
    }
}


// Construct a vector of MoveIt CollisionObjects for the new poses that came in
// Match the IDs of the old poses with the new ones and construct new poses if necessary
/*void CloudSegmenter::match_prev_cur_poses(vector<geometry_msgs::Pose> cur_poses,
                            vector<moveit_msgs::CollisionObject>& next_objs,
                            vector<moveit_msgs::CollisionObject>& remove_objs ){

    int prev_len = prev_diffs.size();
    int cur_len = cur_poses.size();
    set<geometry_msgs::Pose, pose_compare> added_poses;

    if (prev_len != 0){
        return;
        //Map the ID of the CollisionObject from the previous frame to the best match pose
        //from the current frame
        IDPoseMap prev_to_cur_map; //Previous objects are mapped to their closest current pose
        IDObjectMap id_to_obj_map;

        for(int j = 0; j < prev_len; j++){
            id_to_obj_map[prev_diffs[j].id] = prev_diffs[j];
        }

        for(int i = 0; i < cur_len; i++){
            geometry_msgs::Pose cur_pose = cur_poses[i];
            Eigen::Vector3f cur_point = positionToVector( cur_pose.position );
            float min_d = 1000; //things probably won't get bigger than this?
            moveit_msgs::CollisionObject argmin;
            for(int j = 0; j < prev_len; j++){
                geometry_msgs::Pose prev_pose = prev_diffs[j].primitive_poses[0];
                Eigen::Vector3f prev_point = positionToVector( prev_pose.position );
                float d = (prev_point - cur_point).norm();
                if(d < min_d){
                    min_d = d;
                    argmin = prev_diffs[j];
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
            cur_obj.header.frame_id = "/base";
            next_objs.push_back(cur_obj);
        }

        //For all the poses that did not make it into the new scene, remove them
        for(int i = 0; i < prev_diffs.size(); i ++){
            if(prev_to_cur_map.find(prev_diffs[i].id) == prev_to_cur_map.end()){
                moveit_msgs::CollisionObject remove_obj = prev_diffs[i];
                remove_obj.operation = moveit_msgs::CollisionObject::REMOVE;
                remove_obj.header.stamp = ros::Time::now();
                remove_obj.header.frame_id = "/base";
                remove_objs.push_back(remove_obj);
            }
        }
    }

    // For all the current poses that were not matched with a previous pose,
    // add a new CollisionObject

    for(int j = 0; j < cur_poses.size(); j++){
        //if(added_poses.find(cur_poses[j]) == added_poses.end()){

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

            new_obj.header.frame_id = "/base";
            new_obj.header.stamp = ros::Time::now();

            object_sequence++;

            next_objs.push_back(new_obj);
        //}
    }
}*/

void CloudSegmenter:: publish_poses(){
    //geometry_msgs::PoseArray msg;
    //msg.poses = cur_poses;
    if(!published_goals){
        CollisionObjectArray msg;
        vector<moveit_msgs::CollisionObject> cur_diffs_vec;
        for(IDObjectMap::iterator it = cur_diffs.begin(); it != cur_diffs.end(); it++){
            cur_diffs_vec.push_back(it->second);
        }
        msg.objects = cur_diffs_vec;
        if(cur_diffs_vec.empty()){
            cout << "Oops, no objects found!" << endl;
        }
        /*for(int i = 0; i < cur_diffs_vec.size(); i++){
            cur_diffs_vec[i].header.stamp = ros::Time::now();
            object_pub.publish(cur_diffs_vec[i]);
        }*/
        
        object_pub.publish(msg);

        published_goals=true;
    }

    geometry_msgs::PoseArray pose_msg;
    pose_msg.poses = goal_poses;
    goal_pub.publish(pose_msg);
    if(segmented){
        //tf_listener.waitForTransform(frame_id, "/base", cloud_msg.header.stamp, ros::Duration(4.0));
        //pcl_ros::transformPointCloud("/base", cloud_msg, cloud_msg, tf_listener);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
    }

    /*if(has_cloud){
        if(!goal_poses.empty()){
            exclude_all_objects(goal_poses);
        }
        cout << "publishing modified points" << endl;
        cloud_msg.header.frame_id = frame_id;
        tf_listener.waitForTransform(frame_id, "/base", cloud_msg.header.stamp, ros::Duration(4.0));
        pcl_ros::transformPointCloud("/base", cloud_msg, cloud_msg, tf_listener);
        //cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
    }*/
}

PointColorCloud::ConstPtr CloudSegmenter::getCloudPtr(){
    return cloud;
}

PointColorCloud::ConstPtr CloudSegmenter::getClusteredCloudPtr(){
    return colored_cloud;
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
                    PointColorCloud newcloud = *cloud_ptrs[i] + *cloud_ptrs[j];
                    
                    PointColorCloud::Ptr newptr = newcloud.makeShared();
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

    /* Segmentation code from:
       http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php*/

    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
                        boost::shared_ptr<pcl::search::Search
                        <pcl::PointXYZRGB> >
                        (new pcl::search::KdTree<pcl::PointXYZRGB>);

    cloud_ptrs.clear();
    cloud_boxes.clear();
    vector <pcl::PointIndices> clusters;
    PointColorCloud obstacle_points;
    //PointColorCloud goal_obj_points;
    
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (distance_threshold);
    reg.setPointColorThreshold (point_color_threshold);
    reg.setRegionColorThreshold (region_color_threshold);
    reg.setMinClusterSize (min_cluster_size);
    reg.setMaxClusterSize (max_cluster_size);

    reg.extract (clusters);
   
    //cloud_mutex.lock(); 
    colored_cloud = PointColorCloud(*reg.getColoredCloud()).makeShared();
    //cloud_mutex.unlock(); 

    // Select the correct color clouds from the segmentation
    
    cout << "Finished segmentation, starting clustering" << endl;
    set<int> goal_indices;
    for (int i = 0; i < clusters.size(); i++){
        pcl::PointIndices cluster = clusters[i];

        // Get a representative color in the cluster
        const int n = cluster.indices.size();
        const int sample_inc = n/sample_size;
            
        pcl::CentroidPoint<pcl::PointXYZRGB> rgb_centroid;
        for (int j = 0; j < n; j++){
            rgb_centroid.add(cloud->at(cluster.indices[j]));
        }
        pcl::PointXYZRGB avg_xyz;
        rgb_centroid.get(avg_xyz);
        pcl::PointRGB avg(avg_xyz.b, avg_xyz.g, avg_xyz.r);
        /*float n_f = cluster.indices.size();
        float r = 0; float g = 0; float b = 0;
        // TODO: this is a major bottleneck. improve performance
        for (int j = 0; j < n; j++){
            pcl::PointRGB color = getCloudColorAt(cluster.indices[j]);
            r += (float) color.r; //float typecasts suck
            g += (float) color.g;
            b += (float) color.b;
        }
        r /= n_f; g /= n_f; b /= n_f;
        cout << "Average color: " << r << ", " << g << ", " << b << endl;
        pcl::PointRGB avg((unsigned char) b, (unsigned char) g, (unsigned char) r);*/

        cout << "Average color: " << (int) avg.r << ", " << (int) avg.g <<
                ", " << (int) avg.b << endl;

        // Check if avg is within the clicked color
        PointColorCloud cloud_subset = PointColorCloud(*cloud, cluster.indices);
        PointColorCloud::Ptr cloud_ptr = cloud_subset.makeShared();
        if (isPointWithinDesiredRange(avg, desired_color, radius)){
            cloud_ptrs.push_back(cloud_ptr);
            for(int j = 0; j < n; j++){
                goal_indices.insert(cluster.indices[j]);
            }
        }
    }
    /*for(int i = 0; i < (*indices).size(); i++){
        if(goal_indices.find(indices->at(i)) == goal_indices.end() ){
            obstacle_points.push_back( cloud->at(indices->at(i)));
        }
    }
    cout<< "Modified point cloud has " << obstacle_points.size() << " points" << endl;

    obstacle_cloud = obstacle_points.makeShared();*/
    pcl::toROSMsg(*colored_cloud, cloud_msg);

    cout << "Clusters found: " << cloud_ptrs.size() << endl;
    if(cloud_ptrs.empty()){
        return;
    }

    segmented = true;

    vector<geometry_msgs::Pose> cur_poses;
    vector<OrientedBoundingBox> OBBs;
    for (int i = 0; i < cloud_ptrs.size(); i++){
        OBBs.push_back(getOBBForCloud(cloud_ptrs[i]));
        //cout << "OBB position: " << OBBs.back().get_position() << endl;
        //OBBs.back().set_sides( object_side, object_side, object_side);
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
        //pose_in.header.stamp = ros::Time::now();
        tf_listener.waitForTransform(frame_id, "base", pose_in.header.stamp, ros::Duration(4.0));
        tf_listener.transformPose("/base", pose_in, pose_out);
        //cout << "Pose out: " << pose_out.pose << endl;
        pose_out.header.frame_id = "/base";
        cur_poses.push_back(pose_out.pose);
    }

    cout << "Found " << cur_poses.size() << " non-colliding boxes" << endl;
    //match_objects(cur_poses);
    if(!published_goals){
        match_objects(cur_poses);
    }
    goal_poses = cur_poses;

}

void CloudSegmenter::points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    updateParams();
    //cout << "got points" << endl;
    frame_id = msg->header.frame_id;
    // Members: float x, y, z; uint32_t rgba
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);
    indices = pcl::IndicesPtr( new vector<int>() );

    //cout << "Locking in points callback" << endl;
    //cloud_mutex.lock();
    pcl::removeNaNFromPointCloud(*cloud, *cloud, *indices);

    pcl::VoxelGrid<pcl::PointXYZRGB> sampler;
    sampler.setInputCloud(cloud);
    sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    sampler.filter(*cloud);

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> noise_filter;
    noise_filter.setInputCloud(cloud);
    noise_filter.setRadiusSearch(outlier_radius);
    noise_filter.setMinNeighborsInRadius(min_neighbors);
    noise_filter.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (filter_min, filter_max);
    pass.filter (*indices);


    //cout << "Unlocking in points callback" << endl;
    //cloud_mutex.unlock();

    if(!has_cloud){
        cloud_msg = sensor_msgs::PointCloud2(*msg);
    }

    has_cloud = true;

    if(has_desired_color){
        cout << "Segmenting for desired color: " << (int) desired_color.r <<
                ", " << (int) desired_color.g << ", " << (int) desired_color.b << endl;

        //cloud_mutex.lock();
        segmentation();
        //cloud_mutex.unlock();
        publish_poses();
    }
}

void CloudSegmenter::color_callback(const geometry_msgs::Point msg){
    desired_color = pcl::PointRGB(msg.z, msg.y, msg.x); //bgr!
    has_desired_color = true;

}

//just a wrapper to make things more readable
void addComparison(pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond,
                   const char* channel, pcl::ComparisonOps::CompareOp op,
                   float value){
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZRGB>(channel, op, value)) );
}


//not in use
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
    object_camera_tf.setIdentity(); 
    object_camera_tf = object_camera_tf.translate(position);
    object_camera_tf = object_camera_tf.rotate(orientation);

    Eigen::Matrix4f object_camera = object_camera_tf.matrix();
    Eigen::Matrix4f camera_object = object_camera.inverse();

    pcl::transformPointCloud(*src_cloud, *dst_cloud, camera_object);

    //apply condition to exclude the cube
    //From http://pointclouds.org/documentation/tutorials/conditional_removal.php
    pcl::ConditionOr<pcl::PointXYZRGB>::Ptr range_cond(
                                    new pcl::ConditionOr<pcl::PointXYZRGB>);

    addComparison(range_cond, "x", pcl::ComparisonOps::LT, -side/2.0);
    addComparison(range_cond, "x", pcl::ComparisonOps::GT,  side/2.0);
    addComparison(range_cond, "y", pcl::ComparisonOps::LT, -side/2.0);
    addComparison(range_cond, "y", pcl::ComparisonOps::GT,  side/2.0);
    addComparison(range_cond, "z", pcl::ComparisonOps::LT, -side/2.0);
    addComparison(range_cond, "z", pcl::ComparisonOps::GT,  side/2.0);
                                                       
    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
    condrem.setCondition(range_cond); 
    condrem.setInputCloud(dst_cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*dst_cloud);

    //transform back
    pcl::transformPointCloud(*dst_cloud, *dst_cloud, object_camera);

}

void CloudSegmenter::exclude_all_objects(vector<geometry_msgs::Pose> cur_poses){


    for(int i = 0; i < cur_poses.size(); i++){

        exclude_object(cur_poses[i], obstacle_cloud, obstacle_cloud);
    }

    pcl::toROSMsg(*obstacle_cloud, cloud_msg);
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
