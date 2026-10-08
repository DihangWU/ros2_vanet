// Keep the sensor scene current even during externally paced, paused frames.
#include <gz/sim/System.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/plugin/Register.hh>

class SensorRenderSync : public gz::sim::System,
                         public gz::sim::ISystemConfigure {
public:
    void Configure(const gz::sim::Entity &,
                   const std::shared_ptr<const sdf::Element> &,
                   gz::sim::EntityComponentManager &,
                   gz::sim::EventManager &events) override {
        // Sensors detects a render listener and refreshes the scene every
        // update. Individual sensor update_rate still limits data publication.
        connection = events.Connect<gz::sim::events::PreRender>([] {});
    }
private:
    gz::common::ConnectionPtr connection;
};

GZ_ADD_PLUGIN(SensorRenderSync, gz::sim::System, gz::sim::ISystemConfigure)
