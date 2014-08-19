#ifndef ORIENTED_BOUNDING_BOX_H
#define ORIENTED_BOUNDING_BOX_H

#include <Eigen/Eigen>
#include <pcl/point_types.h>

//TODO:

class OrientedBoundingBox {
    Eigen::Vector3f min_point_OBB;
    Eigen::Vector3f max_point_OBB;
    Eigen::Vector3f position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    //keeping these around for kicks
    Eigen::Vector3f min_point_AABB;
    Eigen::Vector3f max_point_AABB;
    public:
    OrientedBoundingBox() {}

    OrientedBoundingBox(Eigen::Vector3f min_point_O,
                        Eigen::Vector3f max_point_O,
                        Eigen::Vector3f position_O,
                        Eigen::Matrix3f rotational_matrix_O,
                        Eigen::Vector3f min_point_AA,
                        Eigen::Vector3f max_point_AA ){
        min_point_OBB = min_point_O;
        max_point_OBB = max_point_O;
        position_OBB = position_O;
        rotational_matrix_O = rotational_matrix_O;
        min_point_AABB = min_point_AA;
        max_point_AABB = max_point_AA;
    }

    OrientedBoundingBox(pcl::PointXYZRGB min_point_O,
                        pcl::PointXYZRGB max_point_O,
                        pcl::PointXYZRGB position_O,
                        Eigen::Matrix3f rotational_matrix_O,
                        pcl::PointXYZRGB min_point_AA,
                        pcl::PointXYZRGB max_point_AA){
        min_point_OBB = Eigen::Vector3f(min_point_O.x, min_point_O.y, min_point_O.z);
        max_point_OBB = Eigen::Vector3f(max_point_O.x, max_point_O.y, max_point_O.z);
        position_OBB  = Eigen::Vector3f(position_O.x, position_O.y, position_O.z);
        rotational_matrix_OBB = rotational_matrix_O;
        min_point_AABB = Eigen::Vector3f(min_point_AA.x, min_point_AA.y, min_point_AA.z);
        max_point_AABB = Eigen::Vector3f(max_point_AA.x, max_point_AA.y, max_point_AA.z);
 
    }

    Eigen::Vector3f get_min_point(){
        return min_point_OBB;
    }
    Eigen::Vector3f get_max_point(){
        return max_point_OBB;
    }
    Eigen::Vector3f get_position(){
        return position_OBB;
    }

    pcl::PointXYZRGB get_min_point_XYZ(){
        return pcl::PointXYZRGB(min_point_OBB[0], min_point_OBB[1], min_point_OBB[2]);
    }
    pcl::PointXYZRGB get_max_point_XYZ(){
        return pcl::PointXYZRGB(max_point_OBB[0], max_point_OBB[1], max_point_OBB[2]);
    }
    pcl::PointXYZRGB get_position_XYZ(){
        return pcl::PointXYZRGB(position_OBB[0], position_OBB[1], position_OBB[2]);
    }

    Eigen::Matrix3f get_rotational_matrix(){
        return rotational_matrix_OBB;
    }

    void set_min_point(Eigen::Vector3f min_point_O){
        min_point_OBB = min_point_O;
    }
    void set_max_point(Eigen::Vector3f max_point_O){
        max_point_OBB = max_point_O;
    }
    void set_position(Eigen::Vector3f position_O){
        position_OBB = position_O;
    }
    void set_rotational_matrix(Eigen::Matrix3f rotational_matrix_O){
        rotational_matrix_OBB = rotational_matrix_O;
    }

    Eigen::Vector3f get_sides(){
        //Calculate the length of a side based on min_point, max_point and rotation
        Eigen::Matrix3f rot_inv = rotational_matrix_OBB.inverse();

        //First rotate min_point and max_point
        Eigen::Vector3f min_point = rot_inv * min_point;
        Eigen::Vector3f max_point = rot_inv * max_point;

        //Translate by the inverse of the new min_point
        return max_point - min_point;

    }

    void set_sides(const float x, const float y, const float z){
        min_point_OBB = Eigen::Vector3f(-x/2.0, -y/2.0, -z/2.0);
        max_point_OBB = Eigen::Vector3f(x/2.0, y/2.0, z/2.0);
        
        min_point_OBB = rotational_matrix_OBB * min_point_OBB;
        max_point_OBB = rotational_matrix_OBB * max_point_OBB;
        min_point_OBB += position_OBB;
        max_point_OBB += position_OBB;

    }

//lazy AABB collision detection

    bool collides_with(OrientedBoundingBox input_box){
       //Returns true if this object collides with input_box, false otherwise
       //Lazy AABB implementation at the moment!
       return max_point_AABB[0] > input_box.min_point_AABB[0] &&
              min_point_AABB[0] < input_box.max_point_AABB[0] &&
              max_point_AABB[1] > input_box.min_point_AABB[1] &&
              min_point_AABB[1] < input_box.max_point_AABB[1] &&
              max_point_AABB[2] > input_box.min_point_AABB[2] &&
              min_point_AABB[2] < input_box.max_point_AABB[2];

    }


};

#endif
