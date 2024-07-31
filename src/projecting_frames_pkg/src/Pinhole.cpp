#include "../include/Pinhole.h"

Pinhole::Pinhole(): nh_("eigen_node"), origin_of_temp_in_map_(70.5, -135.5 , 4.5), rotation_of_temp_in_map_(0, 0, -0.1736482, 0.9848078), direction_of_camera_(0, 1){
    image_subscriber_ = nh_.subscribe("/dash_center/image_raw", 10, &Pinhole::imageCallback, this);
    cam_info_subscriber_ = nh_.subscribe("/dash_center/camera_info", 10, &Pinhole::camInfoCallback, this);

    cv::namedWindow("Video"); 

    createFrame();
    getUnitVectors();
};

// image message callback
void Pinhole::imageCallback(const sensor_msgs::Image::ConstPtr& img_msg){
    try{
        image_ = cv_bridge::toCvShare(img_msg, "bgr8")->image; // converting ros image message to cv image
    }
    
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to 'bgr8'");
    }  
}

// camera info message callback
void Pinhole::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_msg){
    cam_model_.fromCameraInfo(cam_msg);
    fov_horizantal_ = cam_model_.fovX(); // getting horizontal fov from cam_model_

    frame_id_ = cam_msg->header.frame_id;

    cam_info_subscriber_.shutdown(); // unsubscribing after getting the first message
}

// function to create temporary frame
void Pinhole::createFrame(){ 
    create_temp_transorm_.setOrigin(origin_of_temp_in_map_);
    create_temp_transorm_.setRotation(rotation_of_temp_in_map_);

    tf_broadcaster_.sendTransform(tf::StampedTransform(create_temp_transorm_, ros::Time::now(), "map", "temp_frame")); // sending transformation between map and temporary frame
}

// function to get unit vectors of temporary frame relative to map frame
void Pinhole::getUnitVectors(){
    const tf::Matrix3x3 rotation_matrix(rotation_of_temp_in_map_);

    i_of_temp_in_map_ = rotation_matrix.getRow(0) + origin_of_temp_in_map_;
    j_of_temp_in_map_ = rotation_matrix.getRow(1) + origin_of_temp_in_map_;
    k_of_temp_in_map_ = rotation_matrix.getRow(2) + origin_of_temp_in_map_;
}

// the main function which does both extrinsic and intrinsic transformation
void Pinhole::controlLoop(){
    if (image_.empty()) {
        ROS_ERROR("Image is empty. Cannot display.");
        return;
    }
    
    try{
        tf_listener_.lookupTransform(frame_id_, "map", ros::Time(0), map_to_dash_transform_); // getting the transformation from map to dash center
    }

    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1).sleep();
        return;
    } 

    // applying extrinsic transformation by multiplying positions in map with the transformation matrix. the resulting vector is the positions relative to dash center frame
    origin_of_temp_in_dash_ = map_to_dash_transform_ * origin_of_temp_in_map_;
    i_of_temp_in_dash_ = map_to_dash_transform_ * i_of_temp_in_map_;
    j_of_temp_in_dash_ = map_to_dash_transform_ * j_of_temp_in_map_;
    k_of_temp_in_dash_ = map_to_dash_transform_ * k_of_temp_in_map_;

    Eigen::Vector2d origin_of_temp_in_dash_eigen(origin_of_temp_in_dash_.x(), origin_of_temp_in_dash_.z()); // the line which starts from the origin of the camera and ends in the origin of the point in 3D.

    // calculating the angle between optical axis and the origin_of_temp_in_dash_eigen
    double angle = std::atan2((origin_of_temp_in_dash_eigen.y() * direction_of_camera_.x()) - (origin_of_temp_in_dash_eigen.x() * direction_of_camera_.y()), // using cross() returns 0, so I sticked with calculating like this.
                              origin_of_temp_in_dash_eigen.dot(direction_of_camera_));
    
    if (std::abs(angle) <= fov_horizantal_/2) { // checking if the angle is bigger than half of the fov, if so, then the point will not projected.
        const cv::Point3d cv_origin_in_dash_3d(origin_of_temp_in_dash_.x(), origin_of_temp_in_dash_.y(), origin_of_temp_in_dash_.z());
        const cv::Point3d cv_i_in_dash_3d(i_of_temp_in_dash_.x(), i_of_temp_in_dash_.y(), i_of_temp_in_dash_.z());
        const cv::Point3d cv_j_in_dash_3d(j_of_temp_in_dash_.x(), j_of_temp_in_dash_.y(), j_of_temp_in_dash_.z());
        const cv::Point3d cv_k_in_dash_3d(k_of_temp_in_dash_.x(), k_of_temp_in_dash_.y(), k_of_temp_in_dash_.z());

        // applying intrinsic transformation by using the project3dToPixel() function. the resulting vector is the 2D positions in image plane.
        cv_origin_projected_2d_ = cam_model_.project3dToPixel(cv_origin_in_dash_3d);
        cv_i_projected_2d_ = cam_model_.project3dToPixel(cv_i_in_dash_3d);
        cv_j_projected_2d_ = cam_model_.project3dToPixel(cv_j_in_dash_3d); 
        cv_k_projected_2d_ = cam_model_.project3dToPixel(cv_k_in_dash_3d); 

        // drawing the projected points to the image
        cv::arrowedLine(image_, cv_origin_projected_2d_, cv_i_projected_2d_, CV_RGB(0, 255, 0), 2);
        cv::arrowedLine(image_, cv_origin_projected_2d_, cv_j_projected_2d_, CV_RGB(0, 0, 255), 2);
        cv::arrowedLine(image_, cv_origin_projected_2d_, cv_k_projected_2d_, CV_RGB(255, 0, 0), 2);
        cv::putText(image_, "/temp_frame", cv::Point(cv_origin_projected_2d_.x + 15, cv_origin_projected_2d_.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
    } 
    
    cv::imshow("Video", image_);
    cv::waitKey(1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pinhole");

    Pinhole pinhole_object;

    ros::Rate rate(10);

    while (ros::ok()){
        pinhole_object.controlLoop();
         
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
