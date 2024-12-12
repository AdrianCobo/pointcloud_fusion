#ifndef POINTCLOUD_FUSSION__POINTCLOUD_FUSSION_HPP_
#define POINTCLOUD_FUSSION__POINTCLOUD_FUSSION_HPP_

#include <opencv2/core.hpp>
#include <pcl/PCLPointCloud2.h>

#include "rclcpp/rclcpp.hpp"

// Revisar las necesarias
// #include <image_geometry/pinhole_camera_model.h>
// #include <math.h>
// #include <stdlib.h>
// #include <time.h>

// #include <Eigen/Dense>
// #include <chrono>
// #include <cmath>
// #include <functional>
// #include <image_transport/image_transport.hpp>
// #include <iostream>
// #include <memory>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/opencv.hpp>
// #include <string>

// #include "cv_bridge/cv_bridge.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "tf2/exceptions.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"

const N_CAMERAS = 4

namespace pointcloud_fussion
{

using namespace std::chrono_literals;

class PointcloudFusion : public rclcpp::Node
{
public:
    PointcloudFusion();

private:
    cv::Mat depth_imgs_[N_CAMERAS];
    pcl::Pointcloud pcl_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr[N_CAMERAS] depth_subs_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr[N_CAMERAS] intrinsic_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    image_geometry::PinholeCameraModel intrinsic_cameras_matrix_[N_CAMERAS];
    geometry_msgs::msg::TransformStamped cameras2basefootprint_[N_CAMERAS];
    const std::string targets_frame_[N_CAMERAS] = ['c1', 'c2', 'c3', 'c4'];
    const std::string targets_frame_[N_CAMERAS] = ['c1_info', 'c2_info', 'c3_info', 'c4_info'];

    void on_timer();
    void distance_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;
    void intrinsic_params_callback(const sensor_msgs::msg::CameraInfo msg) const;
    cv::Mat proyect_2d_to_3d(cv::Mat input);

    rclcpp::TimerBase::SharedPtr timer_;
    tf2::BufferCore tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
    // void control_cycle();

    // static const int FORWARD = 0;
    // static const int BACK = 1;
    // static const int TURN = 2;
    // static const int STOP = 3;
    // int state_;
    // rclcpp::Time state_ts_;

    // void go_state(int new_state);
    // bool check_forward_2_back();
    // bool check_forward_2_stop();
    // bool check_back_2_turn();
    // bool check_turn_2_forward();
    // bool check_stop_2_forward();

    // const rclcpp::Duration TURNING_TIME {2s};
    // const rclcpp::Duration BACKING_TIME {2s};
    // const rclcpp::Duration SCAN_TIMEOUT {1s};

    // static constexpr float SPEED_LINEAR = 0.3f;
    // static constexpr float SPEED_ANGULAR = 0.3f;
    // static constexpr float OBSTACLE_DISTANCE = 1.0f;

    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;

    // sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

} // namespace pointcloud_fussion

#endif // POINTCLOUD_FUSSION__POINTCLOUD_FUSSION_HPP_