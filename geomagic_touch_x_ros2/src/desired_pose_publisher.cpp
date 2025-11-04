#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class DesiredPosePublisher : public rclcpp::Node {
public:
  DesiredPosePublisher() : Node("desired_pose_publisher") {
    // Parameters
    this->declare_parameter<std::string>("frame_id", "touch_x_base");
    this->declare_parameter<double>("rate_hz", 1000.0);

    // Initial desired pose = zeros
    desired_pose_.header.frame_id = this->get_parameter("frame_id").as_string();
    desired_pose_.pose.position.x = 0.0;
    desired_pose_.pose.position.y = 0.0;
    desired_pose_.pose.position.z = 0.0;
    desired_pose_.pose.orientation.x = 0.0;
    desired_pose_.pose.orientation.y = 0.0;
    desired_pose_.pose.orientation.z = 0.0;
    desired_pose_.pose.orientation.w = 1.0;

    // Publisher (realtime desired pose @ 1kHz default)
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/desired_pose", rclcpp::SystemDefaultsQoS());

    // Subscription to update desired pose externally
    // Topic: ~/set_desired_pose (PoseStamped)
    setter_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/set_desired_pose", rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          // Update internal desired pose; keep incoming header.frame_id if provided, else use parameter default
          if (!msg->header.frame_id.empty()) {
            desired_pose_.header.frame_id = msg->header.frame_id;
          }
          desired_pose_.pose = msg->pose;
        });

    const double rate_hz = this->get_parameter("rate_hz").as_double();
    const auto period = std::chrono::microseconds(static_cast<int64_t>(1e6 / rate_hz));
    timer_ = this->create_wall_timer(period, [this]() { publishDesiredPose(); });

    RCLCPP_INFO(this->get_logger(), "DesiredPosePublisher started at %.1f Hz on topic '%s'",
                rate_hz, (this->get_fully_qualified_name() + std::string("/desired_pose")).c_str());
  }

private:
  void publishDesiredPose() {
    desired_pose_.header.stamp = this->now();
    publisher_->publish(desired_pose_);
  }

  geometry_msgs::msg::PoseStamped desired_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr setter_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DesiredPosePublisher>());
  rclcpp::shutdown();
  return 0;
}


