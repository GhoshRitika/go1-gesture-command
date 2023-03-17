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


      // timer
      timer_ = create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&HGRCom::timer_callback, this));
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
            // angles.data = {-0.106249, 0.481451, 0.309162, 0.468936, 1.085573, 0.575274, 0.247294, 1.005864, -0.005503, 0.375024, 0.255549, 1.372544, 0.111842, 0.550420, 0.298777, 1.497434};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::close: // envelope grasp
          {
            // angles.data = {0.1221, 1.2042, 0.9948, 1.3962, 0.5934, 0.2967, 0.9075, 0.7853, -0.0174, 1.2566, 1.4137, 0.8203, 0.1047, 1.2740, 1.3962, 1.0122};
            angles.data = {-0.014025, 1.271354, 1.225908, 1.218718, 0.382924, 0.605542, 0.565687, 1.438052, -0.014025, 1.271354, 1.225908, 1.218718, -0.014025, 1.571354, 1.225908, 1.218718 };
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::one: // index pointer
          {
            // angles.data = {0.122173, 0.994838, 1.15192, -0.0174, 0.9250, 0.3316, 1.0821, 1.3264, -0.0174, 1.2566, 1.4137, 0.8203, 0.1047, 1.2740, 0.1396, 1.0122};
            angles.data = {0.055832, -0.045003, 0.341649, 0.210546, 1.271088, 0.672026, 0.751380, 0.859493, 0.161638, 0.601370, 1.047405, 1.336240, 0.088053, 0.836770, 1.173271, 0.577937 };
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::PointerIndex:     // pinch with index finger
          {
            // angles.data = {0.4712, 0.8008, 1.2566, 0.680678, 0.139626, 0.15708, 1.32645, 1.0472, 0.0, 0.139626, 0.139626, 0.15708, 0.0, 0.139626, 0.139626, 0.15708};
            angles.data = {0.1053, 0.9131, 0.6610, 1.5252, 1.8915, 0.0991, 0.4600, 0.9239, -0.054767, 0.280580, 0.532845, 1.185964,-0.004172, 0.200516, 0.793276, 0.902366};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::PointerMiddle:  // pinch wiith middle finger
          {
            // angles.data = {0.0, 0.0698, 0.0698, 0.1221, 1.2915, 0.0698, 0.4188, 1.2391, 0.4712, 0.8028, 1.2566, 0.7504, 0.0, 0.0698, 0.0698, 0.1221};
            // angles.data = {-0.082372, 0.307298, 0.570037, 1.182769, 1.304108, 0.205220, 0.260431, 1.148506, 0.305434, 1.099953, 0.568528, 1.252625, -0.003994, 0.200960, 0.793720, 0.902544 };
            angles.data = {0.024676, 0.115215, -0.015267, 0.403517, 1.972614, 0.504066, 0.394907, 1.203806, 0.673144, 0.911242, 0.854434, 1.125783,-0.003728, 0.261407, 0.104119, 0.081840};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::Grasp3:    // 3 finger grasp
          {
            // angles.data = {0.1047, 0.8028, 1.2566, 0.8028, 1.3962, 0.05235, 0.3490, 1.2566, 0.1047, 0.8028, 1.2566, 0.7504, 0.0, 0.0698, 0.0698, 0.1221};
            // angles.data = {0.078644, 1.427925, 0.660842, 1.275881,  1.557929, 0.081928, -0.133944, 1.694932, 0.230252, 1.424618, 0.282711, 1.531963, 0.152318, 0.473729, 0.002929, 0.251466};
            angles.data = {0.1053, 0.9131, 0.6610, 1.5252, 1.972614, 0.404066, 0.4600, 0.9239, 0.2890, 0.9871, 0.4510, 1.5210, 0.080153, 0.133145, 0.366858, -0.127819};
            // angles.data = {0.270994, 0.871920, 1.259105, 0.740640, 1.226884, -0.028227, 0.099503, 1.375651, 0.364195, 1.177709, 0.438135, 1.402724, -0.004882, 0.092491, 0.065152, 0.046068};
            ang_pub_->publish(angles);
            break;
          }
          case HGRCode::Grasp4:  // 4 finger grasp
          {
            // angles.data = {0.0349, 0.7679, 1.3962, 1.0646, 1.2740, 0.4712, 0.4886, 1.4660, 0.1396, 0.7853, 1.3439, 0.9599, 0.3141, 0.8028, 1.2566, 1.1170};
            // angles.data = {0.099148, 1.033647, 0.931303, 0.910799, 1.972614, 1.504066, 0.394907, 1.203806, 0.075094, 1.011278, 0.756706, 1.365710, 0.052903, 1.012699, 0.956334, 0.916213};
            angles.data = {0.1053, 1.2131, 0.6610, 1.5252, 1.966578, 1.504066, 0.467338, 1.339081, 0.1890, 1.2871, 0.4510, 1.5210, 0.289748, 1.171599, 0.519869, 1.066578};
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
