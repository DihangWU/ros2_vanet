// SPDX-License-Identifier: GPL-3.0-or-later
// See lisence/THIRD_PARTY_NOTICES.md and lisence/GPL-3.0.txt.
// Demo adapter: Veins is the only SUMO step owner; INET delivers warning packets.
#include <omnetpp.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include <cmath>
#include <set>
#include "veins_inet/VeinsInetManagerBase.h"
#include "veins_inet/VeinsInetApplicationBase.h"
#include "veins_inet/VeinsInetMobility.h"
#include "veins/modules/mobility/traci/TraCIConstants.h"
#include "EmergencyWarning_m.h"
using namespace omnetpp;
using namespace veins::TraCIConstants;
using Json = nlohmann::json;

class CoSimManager : public veins::VeinsInetManagerBase {
    int fd = -1;
    bool braking = false, frontBraked = false, held = false;
    double desiredGap = 2.5, appliedAt = -1;
    bool externalSpeedMode = false;
    double controlTarget = 15, controlTime = -1;
    uint64_t controlSequence = 0;
    std::string eventId;
    std::set<std::string> removedVehicles;
    Json warnings = Json::array();
    Json networkEvents = Json::array();
    std::ofstream events;
public:
    static CoSimManager* instance;
    veins::TraCIBuffer queryValue(uint8_t command, uint8_t variable, const std::string& id) {
        auto response = getConnection()->query(command, veins::TraCIBuffer() << variable << id);
        uint8_t length, responseCommand, responseVariable;
        std::string responseId;
        response >> length;
        if (length == 0) { uint32_t extendedLength; response >> extendedLength; }
        response >> responseCommand >> responseVariable >> responseId;
        if (responseCommand != command + 0x10 || responseVariable != variable || responseId != id)
            throw cRuntimeError("Unexpected TraCI response");
        return response;
    }
    Json vehicleState(const std::string& id) {
        // Read the current SUMO front position, including the first insertion frame.
        auto response = queryValue(CMD_GET_VEHICLE_VARIABLE, VAR_POSITION, id);
        uint8_t type;
        double x, y;
        response >> type >> x >> y;
        if (type != POSITION_2D || !response.eof()) throw cRuntimeError("Invalid vehicle position");
        auto vehicle = getCommandInterface()->vehicle(id);
        double yaw = (90.0 - vehicle.getAngle()) * std::acos(-1.0) / 180.0;
        return {{"x", x - vehicle.getLength()/2*std::cos(yaw)},
                {"y", y - vehicle.getLength()/2*std::sin(yaw)},
                {"yaw", yaw}, {"speed", vehicle.getSpeed()}};
    }
    Json collisions() {
        auto response = queryValue(CMD_GET_SIM_VARIABLE, VAR_COLLIDING_VEHICLES_IDS, "");
        uint8_t type;
        int32_t count;
        response >> type >> count;
        if (type != TYPE_STRINGLIST || count < 0) throw cRuntimeError("Invalid collision list");
        Json ids = Json::array();
        for (int32_t i = 0; i < count; ++i) { std::string id; response >> id; ids.push_back(id); }
        return ids;
    }
    void record(const Json& event) {
        events << event.dump() << std::endl;
        networkEvents.push_back(event);
    }
    void received(const inet::Ptr<const EmergencyWarning>& payload, double received) {
        auto receiver = vehicleState("car_b");
        Json event = {{"event", "packet_received"}, {"source_time_s",payload->getSourceTime()},{"send_time_s",payload->getSendTime()}, {"receive_time_s",received}, {"event_id",payload->getEventId()},{"x",payload->getX()},{"y",payload->getY()},{"speed",payload->getSpeed()}, {"receive_x",receiver["x"]}, {"receive_y",receiver["y"]}};
        record(event); warnings.push_back(event);
    }
    ~CoSimManager() override { if(fd >= 0) ::close(fd); instance = nullptr; }
protected:
    void initialize(int stage) override {
        veins::TraCIScenarioManager::initialize(stage);
        veins::VeinsInetManagerBase::initialize(stage);
        if(stage == 0) {
            instance = this; events.open("results/network_events.jsonl");
            fd = ::socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in address{}; address.sin_family=AF_INET; address.sin_port=htons(par("rosPort"));
            inet_pton(AF_INET,"127.0.0.1",&address.sin_addr);
            if(::connect(fd,(sockaddr*)&address,sizeof(address)) != 0) throw cRuntimeError("Cannot connect to ROS lockstep server");
        }
    }
    Json exchange(const Json& state) {
        std::string bytes=state.dump()+"\n";
        size_t offset=0;
        while(offset<bytes.size()) { auto n=::send(fd,bytes.data()+offset,bytes.size()-offset,MSG_NOSIGNAL); if(n<=0)throw cRuntimeError("ROS bridge disconnected"); offset+=n; }
        std::string reply; char ch;
        while(::recv(fd,&ch,1,0)==1) { if(ch=='\n')return Json::parse(reply); reply+=ch; }
        throw cRuntimeError("ROS bridge disconnected while waiting for tick acknowledgement");
    }
    void handleSelfMsg(cMessage* msg) override {
        bool step = (msg == executeOneTimestepTrigger);
        if(step && par("lidarControl").boolValue() && externalSpeedMode) {
            // Only execute the ROS lidar command; no lead position/speed/gap feedback here.
            auto rear = getCommandInterface()->vehicle("car_b");
            double target = controlTarget;
            if(controlTime >= 0 && simTime().dbl()-controlTime > .35)
                target = std::max(0.0, rear.getSpeed()-8.0*.05);
            rear.setSpeed(target);
        }
        else if(step && braking) {
            auto a=getCommandInterface()->vehicle("car_a"), b=getCommandInterface()->vehicle("car_b");
            double gap=a.getLanePosition()-a.getLength()-b.getLanePosition();
            if(a.getSpeed()<.01 && b.getSpeed()<.05 && gap<=desiredGap+.05)held=true;
            b.setSpeed(held ? 0 : std::min(15.0,std::max(0.0,a.getSpeed()+gap-desiredGap)));
        }
        veins::TraCIScenarioManager::handleSelfMsg(msg);
        if(!step)return;
        double now=simTime().dbl();
        if(par("lidarControl").boolValue() && !externalSpeedMode && getManagedHosts().count("car_b")) {
            // Disable only SUMO's safe-speed/car-follow override; keep actuator limits.
            getCommandInterface()->vehicle("car_b").setSpeedMode(30);
            externalSpeedMode=true;
        }
        if(now>=5 && !frontBraked) {
            getCommandInterface()->vehicle("car_a").slowDown(0,SimTime(2)); frontBraked=true;
            record({{"event","front_brake"},{"sim_time_s",now}});
        }
        if(now>=7)getCommandInterface()->vehicle("car_a").setSpeed(0);
        if (par("followSumoVehicles").boolValue()) {
            // setBoundary converts these Veins coordinates back to SUMO coordinates.
            auto stateA = vehicleState("car_a"), stateB = vehicleState("car_b");
            auto a = getConnection()->traci2omnet(veins::TraCICoord(stateA["x"], stateA["y"]));
            auto b = getConnection()->traci2omnet(veins::TraCICoord(stateB["x"], stateB["y"]));
            double centerX = (a.x + b.x) / 2;
            double centerY = (a.y + b.y) / 2;
            double halfWidth = std::max(50.0, std::abs(a.x - b.x) / 2 + 25.0);
            for (const auto& view : getCommandInterface()->getGuiViewIds()) {
                getCommandInterface()->guiView(view).setBoundary(
                    veins::Coord(centerX - halfWidth, centerY + 25.0),
                    veins::Coord(centerX + halfWidth, centerY - 25.0));
            }
        }
        Json cars=Json::object();
        for (const auto& entry : getManagedHosts()) {
            if (!removedVehicles.count(entry.first))
                cars[entry.first] = vehicleState(entry.first);
        }
        Json state={{"time",now},{"cars",cars},{"warnings",warnings},{"network_events",networkEvents},{"collisions",collisions()},{"command_active",braking},{"applied_time",appliedAt},{"control_sequence",controlSequence},{"control_time",controlTime}};
        warnings=Json::array();
        networkEvents=Json::array();
        Json reply=exchange(state);
        // The same TraCI owner handles Gazebo edit requests; no second SUMO client.
        if (reply.contains("remove_vehicles")) {
            for (const auto& value : reply["remove_vehicles"]) {
                std::string id = value.get<std::string>();
                if (!getManagedHosts().count(id) || !removedVehicles.insert(id).second) continue;
                // SUMO drops a removed vehicle's subscriptions immediately. Cancel
                // while it still exists, otherwise Veins later unsubscribes a missing ID.
                unsubscribeFromVehicleVariables(id);
                subscribedVehicles.erase(id);
                auto response = getConnection()->query(CMD_SET_VEHICLE_VARIABLE,
                    veins::TraCIBuffer() << REMOVE << id << TYPE_BYTE << REMOVE_VAPORIZED);
                if (!response.eof()) throw cRuntimeError("Unexpected vehicle remove response");
                deleteManagedModule(id);
                record({{"event", "vehicle_removed"}, {"sim_time_s", now}, {"vehicle_id", id}});
            }
        }
        if (reply.value("stop", false)) { endSimulation(); return; }
        if(par("lidarControl").boolValue() && reply.contains("longitudinal") && !reply["longitudinal"].is_null()) {
            auto control = reply["longitudinal"];
            double target=control["target_speed_mps"], stamp=control["time_s"];
            uint64_t sequence=control["sequence"];
            if(!std::isfinite(target) || target<0 || target>15 || stamp>now+.05 || now-stamp>.35)
                throw cRuntimeError("Invalid longitudinal command");
            if(sequence>controlSequence) {
                controlTarget=target; controlTime=stamp; controlSequence=sequence;
                events << Json({{"event","lidar_control_accepted"},{"sim_time_s",now},
                                {"sequence",sequence},{"target_speed_mps",target}}).dump() << std::endl;
            }
        }
        if(reply.contains("command") && !reply["command"].is_null() && !braking) {
            desiredGap=reply["command"]["desired_gap_m"]; eventId=reply["command"]["event_id"];
            if(desiredGap<2.5 || desiredGap>10)throw cRuntimeError("Invalid stopping gap");
            braking=true; appliedAt=now;
            record({{"event","brake_command_applied"},{"sim_time_s",now},{"event_id",eventId}});
        }
        if(now>=par("duration").doubleValue())endSimulation();
    }
};
CoSimManager* CoSimManager::instance=nullptr;
Define_Module(CoSimManager);

class EmergencyWarningApp : public veins::VeinsInetApplicationBase {
    bool receivedOnce=false;
protected:
    bool startApplication() override {
        if(mobility->getExternalId()=="car_a") {
            timerManager.create(veins::TimerSpecification([this]() {
                auto payload=inet::makeShared<EmergencyWarning>();
                payload->setChunkLength(inet::B(100)); payload->setEventId("front_brake_1"); payload->setSourceTime(5.0); payload->setSendTime(simTime().dbl());
                auto state = CoSimManager::instance->vehicleState("car_a");
                payload->setX(state["x"]); payload->setY(state["y"]); payload->setSpeed(state["speed"]); timestampPayload(payload);
                auto packet=createPacket("EMERGENCY_BRAKE"); packet->insertAtBack(payload); sendPacket(std::move(packet));
                CoSimManager::instance->record({{"event","packet_sent"},{"send_time_s",simTime().dbl()},{"event_id","front_brake_1"},{"x",payload->getX()},{"y",payload->getY()}});
            }).oneshotAt(SimTime(5.001)));
        }
        return true;
    }
    void processPacket(std::shared_ptr<inet::Packet> packet) override {
        auto payload=packet->peekAtFront<EmergencyWarning>();
        if(mobility->getExternalId()!="car_b" || receivedOnce || std::string(payload->getEventId())!="front_brake_1")return;
        receivedOnce=true;
        CoSimManager::instance->received(payload,simTime().dbl());
        getParentModule()->getDisplayString().setTagArg("i",1,"green");
    }
};
Define_Module(EmergencyWarningApp);
