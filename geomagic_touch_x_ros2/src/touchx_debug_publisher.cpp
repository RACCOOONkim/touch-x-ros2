#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <sstream>
#include <iomanip>

class TouchXDebugPublisher : public rclcpp::Node {
public:
  TouchXDebugPublisher() : Node("touchx_debug_publisher") {
    // Subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/geomagic_touch_x/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          joint_state_ = msg;
        });

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/geomagic_touch_x/twist", 10,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          twist_ = msg;
        });

    // Publishers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/touchx_debug/markers", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 10Hz 업데이트
        std::bind(&TouchXDebugPublisher::publish_debug_markers, this));
  }

private:
  void publish_debug_markers() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // Joint States 정보 표시
    if (joint_state_) {
      auto msg = joint_state_;

      // Position 값들
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        auto marker = create_text_marker(
            id++, 
            "touch_x_ee",
            -0.05, -0.15 - i * 0.03, 0.0,
            msg->name[i] + ": " + format_value(msg->position[i]),
            0.02
        );
        marker_array.markers.push_back(marker);
      }

      // Velocity 값들 (새 줄에)
      for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
        auto marker = create_text_marker(
            id++,
            "touch_x_ee",
            0.05, -0.15 - i * 0.03, 0.0,
            "v" + std::to_string(i+1) + ": " + format_value(msg->velocity[i]),
            0.015
        );
        marker_array.markers.push_back(marker);
      }

      // Effort 값들
      for (size_t i = 0; i < msg->name.size() && i < msg->effort.size(); ++i) {
        auto marker = create_text_marker(
            id++,
            "touch_x_ee",
            0.15, -0.15 - i * 0.03, 0.0,
            "e" + std::to_string(i+1) + ": " + format_value(msg->effort[i]),
            0.015
        );
        marker_array.markers.push_back(marker);
      }
    }

    // Twist 정보 표시
    if (twist_) {
      auto msg = twist_;
      
      auto marker_linear = create_text_marker(
          id++,
          "touch_x_ee",
          0.0, -0.3, 0.0,
          "Linear: [" + format_value(msg->twist.linear.x) + ", " +
                      format_value(msg->twist.linear.y) + ", " +
                      format_value(msg->twist.linear.z) + "]",
          0.018
      );
      marker_array.markers.push_back(marker_linear);

      auto marker_angular = create_text_marker(
          id++,
          "touch_x_ee",
          0.0, -0.35, 0.0,
          "Angular: [" + format_value(msg->twist.angular.x) + ", " +
                       format_value(msg->twist.angular.y) + ", " +
                       format_value(msg->twist.angular.z) + "]",
          0.018
      );
      marker_array.markers.push_back(marker_angular);
    }

    // 이전 마커들 삭제
    for (int i = id; i < last_marker_count_; ++i) {
      auto delete_marker = create_text_marker(i, "touch_x_ee", 0, 0, 0, "", 0);
      delete_marker.action = visualization_msgs::msg::Marker::DELETE;
      marker_array.markers.push_back(delete_marker);
    }
    last_marker_count_ = id;

    marker_pub_->publish(marker_array);
  }

  visualization_msgs::msg::Marker create_text_marker(
      int id,
      const std::string& frame_id,
      double x, double y, double z,
      const std::string& text,
      double text_size) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "touchx_debug";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = text_size;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = text;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    return marker;
  }

  std::string format_value(double value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << value;
    return ss.str();
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState::SharedPtr joint_state_;
  geometry_msgs::msg::TwistStamped::SharedPtr twist_;
  int last_marker_count_ = 0;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TouchXDebugPublisher>());
  rclcpp::shutdown();
  return 0;
}

