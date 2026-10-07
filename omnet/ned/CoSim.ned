// SPDX-License-Identifier: GPL-3.0-or-later
// See lisence/THIRD_PARTY_NOTICES.md and lisence/GPL-3.0.txt.
import org.car2x.veins.modules.mobility.traci.TraCIScenarioManager;
import org.car2x.veins.subprojects.veins_inet.VeinsInetApplicationBase;
import org.car2x.veins.subprojects.veins_inet.VeinsInetCar;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211DimensionalRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;

simple CoSimManager extends TraCIScenarioManager {
    parameters:
        int rosPort = default(9998);
        bool followSumoVehicles = default(false);
        @class(CoSimManager);
}
simple EmergencyWarningApp extends VeinsInetApplicationBase {
    parameters:
        @class(EmergencyWarningApp);
}
network CoSim {
    submodules:
        manager: CoSimManager { @display("p=100,100"); }
        radioMedium: Ieee80211DimensionalRadioMedium { @display("p=100,200"); }
        visualizer: IntegratedVisualizer { @display("p=100,300"); }
        node[0]: VeinsInetCar;
}
