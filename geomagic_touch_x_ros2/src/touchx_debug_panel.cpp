#include "geomagic_touch_x/touchx_debug_panel.hpp"

#include <string>
#include <iomanip>
#include <sstream>

#include <rviz_common/display_context.hpp>

namespace geomagic_touch_x
{

TouchXDebugPanel::TouchXDebugPanel(QWidget * parent)
: Panel(parent)
{
  // Create UI elements
  auto layout = new QVBoxLayout;
  
  status_label_ = new QLabel("Status: Waiting for data...");
  layout->addWidget(status_label_);
  
  text_display_ = new QTextEdit;
  text_display_->setReadOnly(true);
  text_display_->setFont(QFont("Courier", 9));
  layout->addWidget(text_display_);
  
  setLayout(layout);
  
  // Initialize ROS 2 node (will be created in onInitialize)
  node_ = nullptr;
}

TouchXDebugPanel::~TouchXDebugPanel()
{
}

void TouchXDebugPanel::onInitialize()
{
  // Create ROS 2 node
  node_ = std::make_shared<rclcpp::Node>("touchx_debug_panel_node");
  
  // Create subscriptions
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/geomagic_touch_x/joint_states",
    10,
    std::bind(&TouchXDebugPanel::jointStateCallback, this, std::placeholders::_1)
  );
  
  twist_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/geomagic_touch_x/twist",
    10,
    std::bind(&TouchXDebugPanel::twistCallback, this, std::placeholders::_1)
  );
  
  // Create timer for updating display
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &TouchXDebugPanel::updateDisplay);
  timer_->start(100);  // Update every 100ms (10Hz)
  
  status_label_->setText("Status: Connected");
}

void TouchXDebugPanel::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  last_joint_state_ = msg;
  last_update_time_ = node_->now();
}

void TouchXDebugPanel::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_twist_ = msg;
}

void TouchXDebugPanel::updateDisplay()
{
  // Spin ROS 2 node
  if (node_) {
    rclcpp::spin_some(node_);
  }
  
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3);
  
  ss << "=== Touch X Debug Info ===\n\n";
  
  // Joint States
  if (last_joint_state_) {
    ss << "--- Joint States ---\n";
    auto msg = last_joint_state_;
    
    if (msg->name.size() == msg->position.size()) {
      ss << "Position (rad):\n";
      for (size_t i = 0; i < msg->name.size(); ++i) {
        ss << "  " << msg->name[i] << ": " << formatValue(msg->position[i]) << "\n";
      }
    }
    
    if (msg->name.size() == msg->velocity.size()) {
      ss << "\nVelocity (rad/s):\n";
      for (size_t i = 0; i < msg->name.size(); ++i) {
        ss << "  " << msg->name[i] << ": " << formatValue(msg->velocity[i]) << "\n";
      }
    }
    
    if (msg->name.size() == msg->effort.size()) {
      ss << "\nEffort (N⋅m):\n";
      for (size_t i = 0; i < msg->name.size(); ++i) {
        ss << "  " << msg->name[i] << ": " << formatValue(msg->effort[i]) << "\n";
      }
    }
    
    // Timestamp
    auto time_since_epoch = rclcpp::Time(msg->header.stamp).seconds();
    ss << "\nTimestamp: " << std::fixed << std::setprecision(6) << time_since_epoch << " s\n";
  } else {
    ss << "--- Joint States ---\n";
    ss << "  Waiting for data...\n";
  }
  
  // Twist
  ss << "\n--- Twist (End Effector) ---\n";
  if (last_twist_) {
    auto msg = last_twist_;
    ss << "Linear velocity (m/s):\n";
    ss << "  x: " << formatValue(msg->twist.linear.x) << "\n";
    ss << "  y: " << formatValue(msg->twist.linear.y) << "\n";
    ss << "  z: " << formatValue(msg->twist.linear.z) << "\n";
    
    ss << "\nAngular velocity (rad/s):\n";
    ss << "  x: " << formatValue(msg->twist.angular.x) << "\n";
    ss << "  y: " << formatValue(msg->twist.angular.y) << "\n";
    ss << "  z: " << formatValue(msg->twist.angular.z) << "\n";
  } else {
    ss << "  Waiting for data...\n";
  }
  
  // Update status
  if (last_joint_state_ || last_twist_) {
    auto now = node_->now();
    auto time_diff = (now - last_update_time_).seconds();
    if (time_diff < 1.0) {
      status_label_->setText("Status: Active");
    } else {
      status_label_->setText("Status: Timeout (>1s)");
    }
  } else {
    status_label_->setText("Status: Waiting...");
  }
  
  // Update text display
  text_display_->setPlainText(QString::fromStdString(ss.str()));
}

std::string TouchXDebugPanel::formatValue(double value, int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  return ss.str();
}

}  // namespace geomagic_touch_x

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(geomagic_touch_x::TouchXDebugPanel, rviz_common::Panel)

