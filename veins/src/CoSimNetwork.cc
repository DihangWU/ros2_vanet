// Demo adapter: Veins is the only SUMO step owner; INET delivers warning packets.
#include <omnetpp.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fstream>
#include "veins_inet/VeinsInetManagerBase.h"
#include "veins_inet/VeinsInetApplicationBase.h"
#include "EmergencyWarning_m.h"
using namespace omnetpp;
using Json = nlohmann::json;

class CoSimManager : public veins::VeinsInetManagerBase {
    int fd = -1;
    bool braking = false, frontBraked = false, held = false;
    double desiredGap = 2.5, appliedAt = -1;
    std::string eventId;
    Json warnings = Json::array();
    std::ofstream events;
public:
    static CoSimManager* instance;
    void record(const Json& event) { events << event.dump() << std::endl; }
    void received(const inet::Ptr<const EmergencyWarning>& payload, double received) {
        Json event = {{"event", "packet_received"}, {"source_time_s",payload->getSourceTime()},{"send_time_s",payload->getSendTime()}, {"receive_time_s",received}, {"event_id",payload->getEventId()},{"x",payload->getX()},{"y",payload->getY()},{"speed",payload->getSpeed()}};
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
        if(step && braking) {
            auto a=getCommandInterface()->vehicle("car_a"), b=getCommandInterface()->vehicle("car_b");
            double gap=a.getLanePosition()-a.getLength()-b.getLanePosition();
            if(a.getSpeed()<.01 && b.getSpeed()<.05 && gap<=desiredGap+.05)held=true;
            b.setSpeed(held ? 0 : std::min(15.0,std::max(0.0,a.getSpeed()+gap-desiredGap)));
        }
        veins::TraCIScenarioManager::handleSelfMsg(msg);
        if(!step)return;
        double now=simTime().dbl();
        if(now>=5 && !frontBraked) {
            getCommandInterface()->vehicle("car_a").slowDown(0,SimTime(2)); frontBraked=true;
            record({{"event","front_brake"},{"sim_time_s",now}});
        }
        if(now>=7)getCommandInterface()->vehicle("car_a").setSpeed(0);
        Json cars=Json::object();
        for(auto id : {"car_a","car_b"}) {
            auto v=getCommandInterface()->vehicle(id);
            cars[id]={{"x",v.getLanePosition()-v.getLength()/2},{"y",-1.6},{"yaw",0.0},{"speed",v.getSpeed()}};
        }
        Json state={{"time",now},{"cars",cars},{"warnings",warnings},{"command_active",braking},{"applied_time",appliedAt}};
        warnings=Json::array();
        Json reply=exchange(state);
        if(reply.contains("command") && !reply["command"].is_null() && !braking) {
            desiredGap=reply["command"]["desired_gap_m"]; eventId=reply["command"]["event_id"];
            if(desiredGap<2.5 || desiredGap>10)throw cRuntimeError("Invalid stopping gap");
            braking=true; appliedAt=now;
            record({{"event","brake_command_applied"},{"sim_time_s",now},{"event_id",eventId}});
        }
        if(now>=15)endSimulation();
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
                payload->setX(traciVehicle->getLanePosition()-traciVehicle->getLength()/2); payload->setY(-1.6); payload->setSpeed(traciVehicle->getSpeed()); timestampPayload(payload);
                auto packet=createPacket("EMERGENCY_BRAKE"); packet->insertAtBack(payload); sendPacket(std::move(packet));
                CoSimManager::instance->record({{"event","packet_sent"},{"send_time_s",simTime().dbl()},{"event_id","front_brake_1"}});
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
