#pragma once
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
class QCheckBox;
namespace cosim_rviz {
class GroundPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit GroundPanel(QWidget *parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;
private:
  void publish();
  QCheckBox *checkbox_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};
}
