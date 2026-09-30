#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "MvCameraControl.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode();
  
private:
  static void ImageCallback(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
  void* camera_handle_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult on_parameter_update(const std::vector<rclcpp::Parameter> & parameters);
  std::string image_topic_;
  std::chrono::steady_clock::time_point last_time_;
  int frame_count_ = 0;
  std::chrono::steady_clock::time_point last_frame_time_;
  // 断线重连检测函数
  void check_camera_status();
  bool connect_camera();
  // 用于重连的定时器
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  // 保存当前生效的参数（用于重连后恢复）
  double current_exposure_time_ = 5000.0;
  double current_gain_ = 0.0;
  double current_frame_rate_ = 30.0;
  // TODO(student): Design the interfaces and resource ownership required by
  // your implementation. No SDK handles or camera operations are provided.
};
  
}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
