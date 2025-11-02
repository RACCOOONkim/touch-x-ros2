#ifndef GEOMAGIC_TOUCH_X__TOUCHX_DEBUG_PANEL_HPP_
#define GEOMAGIC_TOUCH_X__TOUCHX_DEBUG_PANEL_HPP_

#include <memory>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rviz_common/panel.hpp>

namespace geomagic_touch_x
{

class TouchXDebugPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TouchXDebugPanel(QWidget * parent = 0);
  ~TouchXDebugPanel();

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateDisplay();

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  std::string formatValue(double value, int precision = 3);

  // ROS 2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  
  // Qt UI
  QTextEdit * text_display_;
  QLabel * status_label_;
  QTimer * timer_;
  
  // Data storage
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  geometry_msgs::msg::TwistStamped::SharedPtr last_twist_;
  rclcpp::Time last_update_time_;
};

}  // namespace geomagic_touch_x

#endif  // GEOMAGIC_TOUCH_X__TOUCHX_DEBUG_PANEL_HPP_

