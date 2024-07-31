#pragma once
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

class Pinhole{
    public:
        Pinhole();

        void createFrame();
        void getUnitVectors();

        void controlLoop();
    private:
        ros::NodeHandle nh_; 

        ros::Subscriber image_subscriber_; 
        ros::Subscriber cam_info_subscriber_;
        void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg);
        void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_msg);

        image_geometry::PinholeCameraModel cam_model_;
        
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

        cv::Point2d cv_origin_projected_2d_;
        cv::Point2d cv_i_projected_2d_;
        cv::Point2d cv_j_projected_2d_;
        cv::Point2d cv_k_projected_2d_;

        cv::Mat image_;
        std::string frame_id_;
};