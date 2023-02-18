#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "brc_arm_msg_srv/msg/joints.hpp"

class ArmJointsPublisher : public rclcpp::Node {
public:
    ArmJointsPublisher() : Node("brc_arm_joints_publisher") {
        publisher_ = this -> create_publisher<brc_arm_msg_srv::msg::Joints>("brc_arm/positions", 10);
        // auto message = brc_arm_msg_srv::msg::Joints();
        // auto encoder_goal = std::vector<double, std::allocator<double>>();
        // encoder_goal.resize(5);
        // encoder_goal[0] = 54.45;
        // message.encoder_goal = encoder_goal;
        // publisher_ -> publish(message);
    }
    void pub(const brc_arm_msg_srv::msg::Joints & msg) {
        publisher_ -> publish(msg);
    }

private:
    rclcpp::Publisher<brc_arm_msg_srv::msg::Joints>::SharedPtr publisher_;
};
