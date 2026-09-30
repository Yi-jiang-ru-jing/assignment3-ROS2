#include "hikrobot_camera/camera_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "MvCameraControl.h"
#include <arpa/inet.h>

namespace hikrobot_camera
{

bool CameraNode::connect_camera(){
  std::string camera_ip = this->get_parameter("camera_ip").as_string();
  std::string serial_number = this->get_parameter("serial_number").as_string();
  
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);

  if(nRet != MV_OK || stDeviceList.nDeviceNum == 0){
    RCLCPP_ERROR(this->get_logger(), "无设备");
    return false;
  }
  bool found = false;
  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
    // 注意：这里需要根据设备类型（GigE/USB）去获取序列号并比较
    // 找到后创建句柄并打开设备：
    //pDeviceTarget = pDeviceInfo;
    std::string current_sn;
    std::string current_ip;

    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        // 网口相机获取序列号
        current_sn = (char*)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber;
        // 网口相机获取 IP（MVS 里的 nCurrentIp 是个 32 位整数，你可以转成字符串比较，或者直接比较无符号整数）
        unsigned int current_ip_int = pDeviceInfo->SpecialInfo.stGigEInfo.nCurrentIp;
        // 此处省略 IP 字符串转换代码，为简化，假设先只比对序列号
    } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        // USB 相机获取序列号
        current_sn = (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
    }

    bool sn_match = (!serial_number.empty() && current_sn == serial_number);
    // 是否匹配：
    bool ip_match = (!camera_ip.empty() && current_ip == camera_ip);
    
    if (sn_match || ip_match) {
        RCLCPP_INFO(this->get_logger(), "找到匹配相机! 序列号: %s", current_sn.c_str());
    
      nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
      if (nRet != MV_OK){
        RCLCPP_ERROR(this->get_logger(), "句柄失败");
        continue;
      }
      nRet = MV_CC_OpenDevice(camera_handle_);
      if (nRet != MV_OK){
        RCLCPP_ERROR(this->get_logger(), "打开失败");
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
        continue;
      }
      found = true;
      RCLCPP_INFO(this->get_logger(), "成功打开");
      break;
    }
  }

  if(!found){
    RCLCPP_ERROR(this->get_logger(), "连接失败");
    return false;
  }

  nRet = MV_CC_RegisterImageCallBackEx(camera_handle_, ImageCallback, this);
  if (nRet != MV_OK) {
      RCLCPP_ERROR(this->get_logger(), "注册图像回调失败!");
      return false;
  }

  MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", current_exposure_time_);
  MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0);
  MV_CC_SetFloatValue(camera_handle_, "Gain", current_gain_);
  MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", current_frame_rate_);

  nRet = MV_CC_StartGrabbing(camera_handle_);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "开始取流失败!");
    return false;
  }

  return true;
}


CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_WARN(
    get_logger(),
    "Training scaffold only: camera connection, image publishing, and camera "
    "parameter control are NOT implemented.");

  // TODO(student): Implement the requirements in docs/assignment.md.
  // - Device selection and connection.
  // - Image acquisition and sensor_msgs/msg/Image publishing.
  // - Camera parameter inspection and updates.
  // - Disconnection recovery and resource cleanup.

  this->declare_parameter<std::string>("camera_ip", "");
  this->declare_parameter<std::string>("serial_number", "");
  this->declare_parameter<double>("exposure_time", 5000.0);
  this->declare_parameter<double>("gain", 0.0);
  this->declare_parameter<double>("frame_rate", 30.0);
  this->declare_parameter<std::string>("image_topic", "/image_raw");

  // 1. 获取参数
  std::string camera_ip = this->get_parameter("camera_ip").as_string();
  std::string serial_number = this->get_parameter("serial_number").as_string();
  current_exposure_time_ = this->get_parameter("exposure_time").as_double();
  current_gain_ = this->get_parameter("gain").as_double();
  current_frame_rate_ = this->get_parameter("frame_rate").as_double();
  std::string image_topic = this->get_parameter("image_topic").as_string();
  image_topic_ = this->get_parameter("image_topic").as_string();
  reconnect_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&CameraNode::check_camera_status, this));
  // 2. 根据参数初始化 MVS SDK 并连接相机

  int nRet = MV_CC_Initialize();
  if (nRet != MV_OK){
    RCLCPP_ERROR(this->get_logger(), "初始化失败");
    return;
  }
  
  if (!connect_camera()) {
    RCLCPP_ERROR(this->get_logger(), "连接失败");
    return;
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

  RCLCPP_INFO(this->get_logger(), "相机节点启动成功，正在发布图像到: %s", image_topic_.c_str());
}


  // 静态回调函数实现
  void CameraNode::ImageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
  {
    CameraNode* node = static_cast<CameraNode*>(pUser);
    // 统计实际帧率
    node->frame_count_++;
    node->last_frame_time_ = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - node->last_time_).count();
    
    if (duration >= 1000) { 
      double fps = node->frame_count_ * 1000.0 / duration;
      RCLCPP_INFO(node->get_logger(), "实际接收帧率: %.2f FPS", fps);
      node->frame_count_ = 0;
      node->last_time_ = now;
    }
    // 创建 ROS 2 图像消息
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = node->now();
    msg->header.frame_id = "camera_optical_frame"; // 可以后续改成参数
    msg->height = pFrameInfo->nHeight;
    msg->width = pFrameInfo->nWidth;
    msg->step = pFrameInfo->nWidth; // 假设 Mono8 格式，每行字节数等于宽度
    msg->encoding = "mono8";        // 虚拟相机默认通常是 mono8
    msg->is_bigendian = false;
    if (pFrameInfo->enPixelType == PixelType_Gvsp_Mono8) {
      msg->step = pFrameInfo->nWidth;
      msg->encoding = "mono8";
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_RGB8_Packed) {
      msg->step = pFrameInfo->nWidth * 3;
      msg->encoding = "rgb8";
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_BayerRG8) {
      msg->step = pFrameInfo->nWidth;
      msg->encoding = "bayer_rggb8";
    } else if (pFrameInfo->enPixelType == PixelType_Gvsp_YUV422_YUYV_Packed){
      msg->step = pFrameInfo->nWidth * 2;
      msg->encoding = "yuv422_yuy2";
    } else {
      RCLCPP_WARN(node->get_logger(), "暂不支持的像素格式: 0x%x", (unsigned int)pFrameInfo->enPixelType);
      return;
    }

    msg->data.resize(msg->step * msg->height);
    
    // 拷贝图像数据
    memcpy(msg->data.data(), pData, msg->data.size());

    // 发布消息
    node->image_pub_->publish(std::move(msg));
  }


  // 曝光
  rcl_interfaces::msg::SetParametersResult CameraNode::on_parameter_update(const std::vector<rclcpp::Parameter> & parameters){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      // 1. 处理曝光时间
      if (param.get_name() == "exposure_time") {
        double value = param.as_double();  
        int nRet = MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0);
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "关闭自动曝光失败!";
          return result;
        }
          // 设置曝光
        nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", value);
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "设置曝光时间失败! 错误码: " + std::to_string(nRet);
          return result;
        }
        if (nRet == MV_OK) {
          current_exposure_time_ = value; 
        }
      }
      // 2. 处理增益
      else if (param.get_name() == "gain") {
        double value = param.as_double();
        MV_CC_SetEnumValue(camera_handle_, "GainAuto", 0);
            
        int nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", value);
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "设置增益失败! 错误码: " + std::to_string(nRet);
          return result;
        }
        if (nRet == MV_OK) {
          current_gain_ = value; 
        }
      }
      // 3. 处理帧率
      else if (param.get_name() == "frame_rate") {
        double value = param.as_double();
        int nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", value);
        if (nRet != MV_OK) {
          result.successful = false;
          result.reason = "设置帧率失败!";
          return result;
        }
        if (nRet == MV_OK) {
          current_frame_rate_ = value; 
        }
      }
    }

    return result;
  } 


// 析构函数：释放资源
  CameraNode::~CameraNode()
  {
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(camera_handle_);
    }
    MV_CC_Finalize();
  }


  // 断线重联函数
  void CameraNode::check_camera_status()
  {
    if (camera_handle_ == nullptr) return;
    // 检查是否掉线
    //int nRet = MV_CC_GetIntValue(camera_handle_, "Width", nullptr); // 简单测试
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_frame_time_).count();
    if (elapsed >= 3) {
      RCLCPP_WARN(this->get_logger(), "相机掉线，尝试重连...");
      // 释放资源
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(camera_handle_);
      camera_handle_ = nullptr;

      // 重连
      if (connect_camera()) {
        RCLCPP_ERROR(this->get_logger(), "重连成功");
      }else{
        RCLCPP_ERROR(this->get_logger(), "重连失败");
      }
    }
  }
}  // namespace hikrobot_camera
