#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/point-to-point-module.h"

#include <cmath>
#include <vector>
#include <sstream>

using namespace ns3;

void UpdateOrbit(Ptr<Node> node, double radius, double omega, double phase)
{
    double t = Simulator::Now().GetSeconds();
    double angle = omega * t + phase;

    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
    mob->SetPosition(Vector(
        radius * std::cos(angle),
        radius * std::sin(angle),
        0));

    // 🔁 Reschedule next update (THIS IS CRITICAL)
    Simulator::Schedule(
        Seconds(0.1),
        &UpdateOrbit,
        node,
        radius,
        omega,
        phase);
}


int main (int argc, char *argv[])
{

uint32_t numSatellites = 100;   // change to 500, 1000 later
double orbitRadiusKm = 6371+550;  // Earth + LEO altitude
double satelliteSpeed = 7500; // m/s (abstracted)
CommandLine cmd;
    cmd.AddValue("numSatellites", "Number of satellites", numSatellites);
    cmd.Parse(argc, argv);
//Satellite Node creation

NodeContainer satellites;
satellites.Create(numSatellites);

NodeContainer groundStations;
groundStations.Create(1);


InternetStackHelper internet;
internet.Install(satellites);
internet.Install(groundStations);

// Point-to-Point link setup between satellites and ground station
PointToPointHelper isl;
isl.SetDeviceAttribute("DataRate", StringValue("2Gbps"));
isl.SetChannelAttribute("Delay", StringValue("10ms"));


// Mobility model for satellites
MobilityHelper satelliteMobility;
satelliteMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
satelliteMobility.Install(satellites);

//Make the satellites orbit in a circular pattern
double angularVelocity = satelliteSpeed / (orbitRadiusKm * 1000);

for (uint32_t i = 0; i < numSatellites; ++i)
{
    double phase = 2 * M_PI * i / numSatellites;

    Simulator::Schedule(
        Seconds(0.0),
        &UpdateOrbit,
        satellites.Get(i),
        orbitRadiusKm * 1000,
        angularVelocity,
        phase);
}

std::vector<NetDeviceContainer> islDevices;

for (uint32_t i = 0; i < numSatellites; ++i)
{
    uint32_t next = (i + 1) % numSatellites;

    NetDeviceContainer link =
        isl.Install(satellites.Get(i), satellites.Get(next));

    islDevices.push_back(link);
}

Ipv4AddressHelper address;

for (uint32_t i = 0; i < islDevices.size(); ++i)
{
    std::ostringstream subnet;
    subnet << "10." << i+1 << ".0.0";

    address.SetBase(subnet.str().c_str(), "255.255.255.0");
    address.Assign(islDevices[i]);
}



MobilityHelper gsMobility;
gsMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
gsMobility.Install(groundStations);

// Set ground station position at origin
groundStations.Get(0)->GetObject<MobilityModel>()->SetPosition(
    Vector(0, 0, 0));

AnimationInterface anim("phase1-engine-day3.xml");

for (uint32_t i = 0; i < satellites.GetN(); ++i)
{
    anim.UpdateNodeDescription(satellites.Get(i), "SAT");
    anim.UpdateNodeColor(satellites.Get(i), 0, 255, 255);
}

anim.UpdateNodeDescription(groundStations.Get(0), "GS");
anim.UpdateNodeColor(groundStations.Get(0), 255, 0, 0);


Simulator::Stop(Seconds(2000.0));
Simulator::Run();
Simulator::Destroy();
return 0;
}

