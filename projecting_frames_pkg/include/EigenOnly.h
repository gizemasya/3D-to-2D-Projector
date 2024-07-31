#pragma once
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

class EigenOnly{
    public:
        EigenOnly();

        void createFrame();
        void getUnitVectors();

        void controlLoop();
    private:
        ros::NodeHandle nh_; 

        ros::Subscriber image_subscriber_;
        ros::Subscriber cam_info_subscriber_;
        void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg);
        void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_msg);

        Eigen::Matrix<double, 3, 4> projection_matrix_;
        double fx_;

        Eigen::Vector2d direction_of_camera_;
        double fov_horizantal_;

        tf::TransformBroadcaster tf_broadcaster_;
        tf::Transform create_temp_transorm_;

        tf::TransformListener tf_listener_;
        tf::StampedTransform map_to_dash_transform_;

        const tf::Quaternion rotation_of_temp_in_map_;
        const tf::Point origin_of_temp_in_map_;
        tf::Point i_of_temp_in_map_;
        tf::Point j_of_temp_in_map_;
        tf::Point k_of_temp_in_map_;

        tf::Point origin_of_temp_in_dash_;
        tf::Point i_of_temp_in_dash_;
        tf::Point j_of_temp_in_dash_;
        tf::Point k_of_temp_in_dash_;

        Eigen::Vector3d origin_projected_in_plane;
        Eigen::Vector3d i_projected_in_plane;
        Eigen::Vector3d j_projected_in_plane;
        Eigen::Vector3d k_projected_in_plane;

        cv::Mat image_;
        std::string frame_id_;
};