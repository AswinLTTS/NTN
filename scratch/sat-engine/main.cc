#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"

#include "orbit.h"
#include "links.h"
#include "addressing.h"
#include "routing.h"
#include "traffic.h"
#include "visibility.h"

#include <fstream>

using namespace ns3;

/* ---------- GLOBAL STATE ---------- */
std::ofstream orbitLog;
std::vector<NetDeviceContainer> allDevices;
std::vector<GsLink> gsLinks;

/* ---------- NODE CONTAINERS ---------- */
NodeContainer satellites;
NodeContainer groundStations;

int main(int argc, char *argv[])
{
    /* ---------- Logging ---------- */
    LogComponentEnable("UdpServer", LOG_LEVEL_INFO);

    /* ---------- Parameters ---------- */
    uint32_t numSatellites = 100;
    double orbitRadiusKm = 6371 + 550;
    double satelliteSpeed = 7500;
    double baseOmega = satelliteSpeed / (orbitRadiusKm * 1000);

    CommandLine cmd;
    cmd.AddValue("numSatellites", "Number of satellites", numSatellites);
    cmd.Parse(argc, argv);

    /* ---------- Logs ---------- */
    orbitLog.open("orbit.csv");
    orbitLog << "time,sat_id,x,y,z\n";

    /* ---------- Create Nodes ---------- */
    satellites.Create(numSatellites);
    groundStations.Create(1);

    /* ---------- Internet Stack ---------- */
    InternetStackHelper internet;
    internet.Install(satellites);
    internet.Install(groundStations);

    /* ---------- Mobility ---------- */
    MobilityHelper satMob;
    satMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    satMob.Install(satellites);

    MobilityHelper gsMob;
    gsMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gsMob.Install(groundStations);

    /* GS at Earth surface (equator) */
    groundStations.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(6371e3, 0, 0)
    );

    /* ---------- Orbit Scheduling ---------- */
    const uint32_t NUM_PLANES = 4;
    const double RAAN_STEP = M_PI / 2;
    uint32_t satsPerPlane = numSatellites / NUM_PLANES;

    for (uint32_t plane = 0; plane < NUM_PLANES; ++plane)
    {
        double raan = plane * RAAN_STEP;

        for (uint32_t i = 0; i < satsPerPlane; ++i)
        {
            uint32_t satId = plane * satsPerPlane + i;
            double phase = 2 * M_PI * i / satsPerPlane;

            Simulator::Schedule(
                Seconds(0.0),
                &UpdateOrbit,
                satellites.Get(satId),
                orbitRadiusKm * 1000.0,
                baseOmega,
                phase,
                raan
            );
        }
    }

    /* ---------- Links ---------- */
    CreateIslLinks(satellites, allDevices);
    CreateGsSatLinks(groundStations, satellites, allDevices, gsLinks);

    /* ---------- Addressing ---------- */
    ConfigureSatelliteLoopbacks(satellites);
    AssignPointToPointAddresses(allDevices);

    /* ---------- Routing ---------- */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    InstallGsStaticRoutes(groundStations.Get(0), gsLinks);

    /* ---------- Traffic ---------- */
    uint16_t port = 4000;
    InstallUdpServers(satellites, port);
    InstallUdpClient(groundStations.Get(0));

    /* ---------- Visibility & Handover ---------- */
    double maxGsDistance = 7500e3;
    Simulator::Schedule(
        Seconds(1.0),
        &UpdateGsVisibility,
        maxGsDistance
    );

    /* ---------- Run ---------- */
    Simulator::Stop(Seconds(3600));
    Simulator::Run();
    Simulator::Destroy();

    orbitLog.close();
    return 0;
}
