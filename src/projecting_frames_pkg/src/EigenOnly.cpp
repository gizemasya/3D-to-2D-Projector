#include "../include/EigenOnly.h"

EigenOnly::EigenOnly(): nh_("eigen_node"), origin_of_temp_in_map_(70.5, -135.5 , 4.5), rotation_of_temp_in_map_(0, 0, -0.1736482, 0.9848078), direction_of_camera_(0, 1){
    image_subscriber_ = nh_.subscribe("/dash_center/image_raw", 10, &EigenOnly::imageCallback, this);
    cam_info_subscriber_ = nh_.subscribe("/dash_center/camera_info", 10, &EigenOnly::camInfoCallback, this);

    cv::namedWindow("Video"); 

    createFrame();
    getUnitVectors(); // eng yorumlar ekle.
};

void EigenOnly::imageCallback(const sensor_msgs::Image::ConstPtr& img_msg){
    try{
        image_ = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    }
    
    catch(cv_bridge::Exception& ex){
        ROS_ERROR("Could not convert to 'bgr8'");
    }  
}

void EigenOnly::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_msg){
    projection_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(cam_msg->P.data());
    fx_ = projection_matrix_(0, 0);
    fov_horizantal_ = 2 * (std::atan(cam_msg->width / fx_ * 2));

    frame_id_ = cam_msg->header.frame_id;

    cam_info_subscriber_.shutdown();
}

void EigenOnly::createFrame(){ 
    create_temp_transorm_.setOrigin(origin_of_temp_in_map_);
    create_temp_transorm_.setRotation(rotation_of_temp_in_map_);

    tf_broadcaster_.sendTransform(tf::StampedTransform(create_temp_transorm_, ros::Time::now(), "map", "temp_frame"));
}

void EigenOnly::getUnitVectors(){
    const tf::Matrix3x3 rotation_matrix(rotation_of_temp_in_map_);

    i_of_temp_in_map_ = rotation_matrix.getRow(0) + origin_of_temp_in_map_;
    j_of_temp_in_map_ = rotation_matrix.getRow(1) + origin_of_temp_in_map_;
    k_of_temp_in_map_ = rotation_matrix.getRow(2) + origin_of_temp_in_map_;
}

void EigenOnly::controlLoop(){
    if (image_.empty()){
        ROS_ERROR("Image is empty. Cannot display.");
        return;
    }

    try{
        tf_listener_.lookupTransform(frame_id_, "map", ros::Time(0), map_to_dash_transform_);
    }

    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1).sleep();
        return;
    } 

    origin_of_temp_in_dash_ = map_to_dash_transform_ * origin_of_temp_in_map_;
    i_of_temp_in_dash_ = map_to_dash_transform_ * i_of_temp_in_map_;
    j_of_temp_in_dash_ = map_to_dash_transform_ * j_of_temp_in_map_;
    k_of_temp_in_dash_ = map_to_dash_transform_ * k_of_temp_in_map_; 

    Eigen::Vector2d origin_of_temp_in_dash_eigen(origin_of_temp_in_dash_.x(), origin_of_temp_in_dash_.z());

    double angle = std::atan2((origin_of_temp_in_dash_eigen.y() * direction_of_camera_.x()) - (origin_of_temp_in_dash_eigen.x() * direction_of_camera_.y()), //cross yapınca "0" geliyor, determinant fonksiyonu ile yaparsam da uzuyor
                              origin_of_temp_in_dash_eigen.dot(direction_of_camera_));

    if (std::abs(angle) <= fov_horizantal_/2) {
        const Eigen::Vector4d origin_in_dash_4d(origin_of_temp_in_dash_.x(), origin_of_temp_in_dash_.y(), origin_of_temp_in_dash_.z(), 1.0);
        const Eigen::Vector4d i_in_dash_4d(i_of_temp_in_dash_.x(), i_of_temp_in_dash_.y(), i_of_temp_in_dash_.z(), 1.0);
        const Eigen::Vector4d j_in_dash_4d(j_of_temp_in_dash_.x(), j_of_temp_in_dash_.y(), j_of_temp_in_dash_.z(), 1.0);
        const Eigen::Vector4d k_in_dash_4d(k_of_temp_in_dash_.x(), k_of_temp_in_dash_.y(), k_of_temp_in_dash_.z(), 1.0);

        origin_projected_in_plane = projection_matrix_ * origin_in_dash_4d; 
        i_projected_in_plane = projection_matrix_ * i_in_dash_4d;
        j_projected_in_plane = projection_matrix_ * j_in_dash_4d;
        k_projected_in_plane = projection_matrix_ * k_in_dash_4d;

        origin_projected_in_plane /= origin_projected_in_plane.z();
        i_projected_in_plane /= i_projected_in_plane.z();
        j_projected_in_plane /= j_projected_in_plane.z();
        k_projected_in_plane /= k_projected_in_plane.z(); 

        cv::Point2d cv_origin_projected(origin_projected_in_plane.x(), origin_projected_in_plane.y()); 
        cv::Point2d cv_i_projected(i_projected_in_plane.x(), i_projected_in_plane.y());
        cv::Point2d cv_j_projected(j_projected_in_plane.x(), j_projected_in_plane.y());
        cv::Point2d cv_k_projected(k_projected_in_plane.x(), k_projected_in_plane.y());

        cv::arrowedLine(image_, cv_origin_projected, cv_i_projected, CV_RGB(0, 255, 0), 2); 
        cv::arrowedLine(image_, cv_origin_projected, cv_j_projected, CV_RGB(0, 0, 255), 2);
        cv::arrowedLine(image_, cv_origin_projected, cv_k_projected, CV_RGB(255, 0, 0), 2);
        cv::putText(image_, "/temp_frame", cv::Point(cv_origin_projected.x + 15, cv_origin_projected.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255));
    }
    
    cv::imshow("Video", image_);
    cv::waitKey(1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "eigen");

    EigenOnly eigen_object;

    ros::Rate rate(10);

    while (ros::ok()){
        eigen_object.controlLoop();
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}