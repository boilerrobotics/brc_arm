#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "brc_arm_msg_srv/msg/encoders.hpp"
using std::placeholders::_1;

class ArmEncodersSubscriber : public rclcpp::Node {
public:
    ArmEncodersSubscriber() : Node("brc_arm_encoders_subscriber") {
        subscriber_ = this -> create_subscription<brc_arm_msg_srv::msg::Encoders>("/brc_arm/encoders", 10,
            std::bind(&ArmEncodersSubscriber::topic_callback, this, _1));

        enc_counts.resize(7, 0);
    }

    std::vector<double> enc_counts;

private:
    void topic_callback(const brc_arm_msg_srv::msg::Encoders& msg) {
        RCLCPP_INFO(this -> get_logger(), "Heard: '%lf' for joint 1", msg.encoder_count[1]);
        enc_counts = msg.encoder_count;
    }
    rclcpp::Subscription<brc_arm_msg_srv::msg::Encoders>::SharedPtr subscriber_;
};
