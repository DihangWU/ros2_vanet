#include "cosim_rviz/ground_panel.hpp"
#include <QCheckBox>
#include <QLabel>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace cosim_rviz {
GroundPanel::GroundPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto *layout = new QVBoxLayout(this);
  checkbox_ = new QCheckBox(QString::fromUtf8("显示地面回波"), this);
  checkbox_->setChecked(true);
  checkbox_->setObjectName("show_ground_returns");
  layout->addWidget(checkbox_);
  auto *description = new QLabel(QString::fromUtf8("统一控制激光雷达、毫米波及超声波显示"), this);
  description->setWordWrap(true);
  layout->addWidget(description);
  layout->addStretch();
  connect(checkbox_, &QCheckBox::toggled, this, [this](bool) {
    publish();
    Q_EMIT configChanged();
  });
}
void GroundPanel::onInitialize() {
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::Bool>(
    "/display/show_ground_returns", rclcpp::QoS(1).reliable().transient_local());
  publish();
}
void GroundPanel::publish() {
  if (!publisher_) return;
  std_msgs::msg::Bool message;
  message.data = checkbox_->isChecked();
  publisher_->publish(message);
}
void GroundPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
  bool show = true;
  config.mapGetBool("Show Ground", &show);
  checkbox_->setChecked(show);
  publish();
}
void GroundPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Show Ground", checkbox_->isChecked());
}
}
PLUGINLIB_EXPORT_CLASS(cosim_rviz::GroundPanel, rviz_common::Panel)
