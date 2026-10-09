#include <QApplication>
#include <QCheckBox>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv) {
  QApplication app(argc, argv);
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto panel = loader.createSharedInstance("cosim_rviz/GroundReturns");
  auto *checkbox = panel->findChild<QCheckBox *>("show_ground_returns");
  if (!checkbox || !checkbox->isChecked()) throw std::runtime_error("Missing enabled checkbox");
  bool changed = false;
  QObject::connect(panel.get(), &rviz_common::Panel::configChanged, [&changed] {changed=true;});
  checkbox->setChecked(false);
  rviz_common::Config config;
  panel->save(config);
  bool saved = true;
  if (!config.mapGetBool("Show Ground", &saved) || saved || !changed)
    throw std::runtime_error("Checkbox state was not saved");
  checkbox->setChecked(true);
  panel->load(config);
  if (checkbox->isChecked()) throw std::runtime_error("Checkbox state was not restored");
  std::cout << "PASS: plugin loads, checkbox toggles, saved configuration restores state\n";
}
