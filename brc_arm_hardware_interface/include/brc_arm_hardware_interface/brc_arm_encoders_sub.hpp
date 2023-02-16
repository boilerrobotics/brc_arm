#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "brc_arm_msg_srv/msg/joints.hpp"
using std::placeholders::_1;

class ArmEncodersSubscriber : public rclcpp::Node {
public:
    ArmEncodersSubscriber() : Node("brc_arm_encoders_subscriber") {
        subscriber_ = this -> create_subscription<brc_arm_msg_srv::msg::Joints>("arm_joints", 10,
            std::bind(&ArmEncodersSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const brc_arm_msg_srv::msg::Joints& msg) const {
        RCLCPP_INFO(this -> get_logger(), "Heard: '%lf' for joint 1", msg.encoder_goal[1]);
    }
    rclcpp::Subscription<brc_arm_msg_srv::msg::Joints>::SharedPtr subscriber_;
};
