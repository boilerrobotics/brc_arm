#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "brc_arm_msg_srv/msg/positions.hpp"

class ArmJointsPublisher : public rclcpp::Node {
public:
    ArmJointsPublisher() : Node("brc_arm_joints_publisher") {
        publisher_ = this -> create_publisher<brc_arm_msg_srv::msg::Positions>("brc_arm/positions", 10);
    }
    void pub(const brc_arm_msg_srv::msg::Positions & msg) {
        publisher_ -> publish(msg);
    }

private:
    rclcpp::Publisher<brc_arm_msg_srv::msg::Positions>::SharedPtr publisher_;
};
