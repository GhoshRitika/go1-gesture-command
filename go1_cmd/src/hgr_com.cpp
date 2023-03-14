#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <stdio.h>
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

enum class HGRCode : uint8_t
{
     no_data = 99,
     open = 0
    ,close = 1
    ,one = 2
    ,PointerIndex = 3
    ,PointerMiddle = 4
    ,Grasp3 = 5
    ,Grasp4 = 6
};

class HGRCom : public rclcpp::Node
{
  public:
    HGRCom()
    : Node("hgr_com")
    {
      //Publishers
      ang_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("Joint_angles", 10);

      // hgr_topic subscriber
      hgr_sub_ = create_subscription<std_msgs::msg::Int32>(
                  "/hgr_topic",
                  10, std::bind(&HGRCom::hgr_callback, this, std::placeholders::_1));
      
      // To check if previous gesture has been execute
      // result_sub_ = create_subscription<std_msgs::msg::Int32>(
      //             "/moving",
      //             10, std::bind(&HGRCom::moving_callback, this, std::placeholders::_1));

      // timer
      timer_ = create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&HGRCom::timer_callback, this));
        // previously 5ms
      
      angles.data = {0.0, 0.0, 0.0, 0.0, 0.5236, 0.3665, 0.8901, 0.4887, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      RCLCPP_INFO_STREAM(get_logger(), "Waiting...");
    }

  private:

    /// @brief Continuously running timer callback for sending commands to the Go1.
    void timer_callback()
    {
      if (hgr_code != hgr_code_prev)    // if the Go1 is laying down, need to stand up before doing anything else
      {
        switch (hgr_code)
        {
          case HGRCode::no_data:  // Home position
          {
            angles.data = {0.0, 0.0, 0.0, 0.0, 0.5236, 0.3665, 0.8901, 0.4887, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::open:   // open palm
          {
            angles.data = {0.0, 0.0697, 0.1396, 0.0697, 0.4014, 0.1745, 0.6632, 0.7853, 0.0, 0.0698, 0.1396, 0.0698, 0.0, 0.0698, 0.1396, 0.0698};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::close: // envelope grasp
          {
            angles.data = {0.1221, 1.2042, 0.9948, 1.3962, 0.5934, 0.2967, 0.9075, 0.7853, -0.0174, 1.2566, 1.4137, 0.8203, 0.1047, 1.2740, 1.3962, 1.0122};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::one: // index pointer
          {
            angles.data = {0.0, 0.0174, 0.0349, -0.0174, 0.9250, 0.3316, 1.0821, 1.3264, -0.0174, 1.2566, 1.4137, 0.8203, 0.1047, 1.2740, 0.1396, 1.0122};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::PointerIndex:     // pinch with index finger
          {
            angles.data = {0.4712, 0.8028, 1.2566, 0.7679, 0.9075, 0.1396, 0.3490, 1.0472, 0.0, 0.0174, 0.0349, 0.2617, 0.0, 0.0698, 0.0698, 0.1221};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::PointerMiddle:  // pinch wiith middle finger
          {
            angles.data = {0.0, 0.0698, 0.0698, 0.1221, 1.2915, 0.0698, 0.4188, 1.2391, 0.4712, 0.8028, 1.2566, 0.7504, 0.0, 0.0698, 0.0698, 0.1221};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::Grasp3:    // 3 finger grasp
          {
            angles.data = {0.1047, 0.8028, 1.2566, 0.8028, 1.3962, 0.05235, 0.3490, 1.2566, 0.1047, 0.8028, 1.2566, 0.7504, 0.0, 0.0698, 0.0698, 0.1221};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::Grasp4:  // 4 finger grasp
          {
            angles.data = {0.0349, 0.7679, 1.3962, 1.0646, 1.2740, 0.4712, 0.4886, 1.4660, 0.1396, 0.7853, 1.3439, 0.9599, 0.3141, 0.8028, 1.2566, 1.1170};
            ang_pub_->publish(angles);
            break;
          }
        }
        hgr_code_prev = hgr_code;
      }
    }

    /// @brief Subscription callback to receive hand gesture recognition data.
    /// @param msg - HGR data
    void hgr_callback(const std_msgs::msg::Int32 & msg)
    {
        if (msg.data == -1)
        {
          hgr_code = HGRCode::no_data;
        }
        else 
        {
          hgr_code = static_cast<HGRCode>(msg.data);
        }
    }

    /// @brief 
    /// @param msg - 
    // void moving_callback(const std_msgs::msg::Int32 & msg)
    // {
    //     if (msg.data == 1)
    //     {
    //       moving_flag = 1;
    //     }
    //     else 
    //     {
    //       moving_flag = 0;
    //     }
    // }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ang_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr hgr_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr result_sub_;

    int moving_flag = 0;
    std_msgs::msg::Float64MultiArray angles;
    HGRCode hgr_code = HGRCode::no_data;
    HGRCode hgr_code_prev = HGRCode::no_data;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HGRCom>());
  rclcpp::shutdown();
  return 0;
}
