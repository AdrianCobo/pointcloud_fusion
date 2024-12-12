/*
# Copyright (c) 2024 Adrián Cobo Merino
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include "pointcloud_fusion/PointcloudFusion.hpp"

using namespace std::chrono_literals;

namespace pointcloud_fussion
{

cv::Mat image_processing(const cv::Mat in_image); // borrar

  PointcloudFusion()
  : Node("opencv_subscriber"), tf_buffer_(), tf_listener_(tf_buffer_)
  {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    for(int i = 0; i < N_CAMERAS; i++){
      depth_subs_[i] = this->create_subscription<sensor_msgs::msg::Image>(
      targets_frame_[i], qos,
      std::bind(
        &PointcloudFusion::distance_image_callback, this, std::placeholders::_1));
    }

    for(int i = 0; i < N_CAMERAS; i++){
      intrinsic_subs_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      targets_frame_[i], qos,
      std::bind(
        &PointcloudFusion::intrinsic_params_callback, this, std::placeholders::_1));
    }

    // Call on_timer function every 500ms in order to get our tf
    timer_ = create_wall_timer(500ms, std::bind(&PointcloudFusion::on_timer, this));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_fusion", qos);
  }

  void on_timer()
  {
    try {
      camera2basefootprint = tf_buffer_.lookupTransform(
        "head_front_camera_depth_optical_frame", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Obstacle transform not found: %s", ex.what());
      return;
    }
  }

// TODO: ¿como saber que camara es para guardarla correctamente?
void distance_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  // Convertir los datos de profundidad a un objeto Mat de OpenCV
  cv_bridge::CvImagePtr depth_image_ptr =
    cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

  depth_image = depth_image_ptr->image;
}

// TODO: ¿como saber que camara es para guardarla correctamente?
void intrinsic_params_callback(const sensor_msgs::msg::CameraInfo msg) const
{
  intrinsic_camera_matrix = image_geometry::PinholeCameraModel();
  intrinsic_camera_matrix.fromCameraInfo(msg);
}

// needs to change and return a pointcloud
cv::Mat proyect_2d_to_3d(cv::Mat input)
{
  double fx, fy, cx, cy, x_3d, y_3d, z_3d, d;
  int px, py;

  cv::Mat intrinsic_marix = (cv::Mat)intrinsic_camera_matrix.intrinsicMatrix();

  fx = intrinsic_marix.at<double>(0, 0);
  fy = intrinsic_marix.at<double>(1, 1);
  cx = intrinsic_marix.at<double>(0, 2);
  cy = intrinsic_marix.at<double>(1, 2);

  for (size_t i = 0; i < Points.size(); i++) {
    px = Points[i].x;
    py = Points[i].y;
    d = depth_image.at<float>(py, px);

    x_3d = ((double)px - cx) * d / fx;
    y_3d = ((double)py - cy) * d / fy;
    z_3d = d;

    std::string coordinates = "[";
    coordinates.append(
      std::to_string(x_3d) + " " + std::to_string(y_3d) + " " + std::to_string(z_3d) + "]");

    cv::circle(input, Points[i], 2, WHITE);
    cv::putText(
      input, coordinates, cv::Point(px + 10, py), cv::FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2);
  }

  return input;
}

cv::Mat image_processing(const cv::Mat in_image)
{
  cv::Mat noise_filtered_img, detected_circuit_lines, proyection_lines_printed,
    depth_image_normalized, out_image;

  if (!initialized_img) {
    initialize_user_interface();
  }

  // refresh the global paramiters
  CAMERA_MODE = cv::getTrackbarPos(trackbar_1_text, p5_window_name);
  TRACKBAR_ITERATIONS = cv::getTrackbarPos(trackbar_2_text, p5_window_name);
  TRACKBAR_DISTANCE = cv::getTrackbarPos(trackbar_3_text, p5_window_name);

  cv::medianBlur(in_image, noise_filtered_img, 3);

  switch (CAMERA_MODE) {
    case OPTION_1:
      detected_circuit_lines = detect_circuit_lines(noise_filtered_img);
      proyection_lines_printed = print_3d_to_2d_proyection_lines(detected_circuit_lines);
      out_image = print_3d_to_2d_proyection_points(proyection_lines_printed);
      break;

    case OPTION_2:
      depth_image_normalized = get_normaliced_depth_img();
      out_image = print_3d_to_2d_proyection_points(depth_image_normalized);
      break;

    case CLEAN:
    default:
      out_image = in_image;
      Points.clear();
      break;
  }

  // show resultant image at window P3
  cv::imshow(p5_window_name, out_image);
  cv::waitKey(100);
  return out_image;
}

} // namespace pointcloud_fussion
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudFusion>());
  rclcpp::shutdown();
  return 0;
}