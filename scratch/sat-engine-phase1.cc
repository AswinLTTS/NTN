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

/* =======================
   GS–SAT LINK STRUCT
   ======================= */
struct GsLink
{
    Ptr<Node> gs;
    Ptr<Node> sat;
    NetDeviceContainer devices;
};

static std::vector<GsLink> gsLinks;

/* =======================
   ORBIT UPDATE (ENGINE CORE)
   ======================= */
void UpdateOrbit(Ptr<Node> node, double radius, double omega, double phase)
{
    double t = Simulator::Now().GetSeconds();
    double angle = omega * t + phase;

    node->GetObject<MobilityModel>()->SetPosition(
        Vector(radius * std::cos(angle),
               radius * std::sin(angle),
               0));

    Simulator::Schedule(
        Seconds(0.1),
        &UpdateOrbit,
        node,
        radius,
        omega,
        phase);
}

/* =======================
   GS VISIBILITY & HANDOVER
   ======================= */
void UpdateGsVisibility(double maxGsDistance)
{
    Ptr<Node> gs = gsLinks[0].gs;
    Ptr<MobilityModel> gsMob = gs->GetObject<MobilityModel>();

    double bestDist = 1e18;
    GsLink* bestLink = nullptr;

    /* Find closest visible satellite */
    for (auto &link : gsLinks)
    {
        Ptr<MobilityModel> satMob =
            link.sat->GetObject<MobilityModel>();

        double dist = gsMob->GetDistanceFrom(satMob);

        if (dist <= maxGsDistance && dist < bestDist)
        {
            bestDist = dist;
            bestLink = &link;
        }
    }

    /* Bring only best GS–SAT link UP */
    for (auto &link : gsLinks)
    {
        for (uint32_t i = 0; i < link.devices.GetN(); ++i)
        {
            Ptr<NetDevice> dev = link.devices.Get(i);
            Ptr<Node> node = dev->GetNode();
            Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
            int32_t iface = ipv4->GetInterfaceForDevice(dev);
            if (iface < 0) continue;

            bool shouldBeUp = (&link == bestLink);

            if (shouldBeUp && !ipv4->IsUp(iface))
                ipv4->SetUp(iface);
            else if (!shouldBeUp && ipv4->IsUp(iface))
                ipv4->SetDown(iface);
        }
    }

    Simulator::Schedule(
        Seconds(1.0),
        &UpdateGsVisibility,
        maxGsDistance);
}

/* =======================
   MAIN
   ======================= */
int main(int argc, char *argv[])
{
    uint32_t numSatellites = 20;
    double orbitRadiusKm = 6371 + 550;     // Earth + LEO altitude
    double satelliteSpeed = 7500;          // m/s
    double baseOmega = satelliteSpeed / (orbitRadiusKm * 1000);

    CommandLine cmd;
    cmd.AddValue("numSatellites", "Number of satellites", numSatellites);
    cmd.Parse(argc, argv);

    /* ---------- Nodes ---------- */
    NodeContainer satellites;
    satellites.Create(numSatellites);

    NodeContainer groundStations;
    groundStations.Create(1);

    /* ---------- Internet ---------- */
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

    groundStations.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(0, 0, 0));

    /* ---------- Orbit Scheduling ---------- */
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        double phase = 2 * M_PI * i / numSatellites;

        Simulator::Schedule(
            Seconds(0.0),
            &UpdateOrbit,
            satellites.Get(i),
            orbitRadiusKm * 1000,
            baseOmega,
            phase);
    }

    /* ---------- Inter-Satellite Links (STATIC RING) ---------- */
    PointToPointHelper isl;
    isl.SetDeviceAttribute("DataRate", StringValue("2Gbps"));
    isl.SetChannelAttribute("Delay", StringValue("10ms"));

    std::vector<NetDeviceContainer> allDevices;

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        uint32_t next = (i + 1) % numSatellites;
        NetDeviceContainer link =
            isl.Install(satellites.Get(i), satellites.Get(next));
        allDevices.push_back(link);
    }

    /* ---------- GS–SAT LINKS ---------- */
    PointToPointHelper gsLink;
    gsLink.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    gsLink.SetChannelAttribute("Delay", StringValue("30ms"));

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        NetDeviceContainer link =
            gsLink.Install(groundStations.Get(0), satellites.Get(i));

        allDevices.push_back(link);

        gsLinks.push_back({
            groundStations.Get(0),
            satellites.Get(i),
            link
        });
    }

    /* ---------- IP Addressing ---------- */
    Ipv4AddressHelper address;
    for (uint32_t i = 0; i < allDevices.size(); ++i)
    {
        std::ostringstream subnet;
        subnet << "10." << (i + 1) << ".0.0";
        address.SetBase(subnet.str().c_str(), "255.255.255.0");
        address.Assign(allDevices[i]);
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    /* ---------- GS Visibility Controller ---------- */
    double maxGsDistance = 3000e3; // 3000 km
    Simulator::Schedule(
        Seconds(0.0),
        &UpdateGsVisibility,
        maxGsDistance);

    /* ---------- NetAnim (temporary visualization) ---------- */
    AnimationInterface anim("phase1-day4-engine.xml");

    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        anim.UpdateNodeDescription(satellites.Get(i), "SAT");
        anim.UpdateNodeColor(satellites.Get(i), 0, 255, 255);
    }

    anim.UpdateNodeDescription(groundStations.Get(0), "GS");
    anim.UpdateNodeColor(groundStations.Get(0), 255, 0, 0);

    /* ---------- Run ---------- */
    Simulator::Stop(Seconds(600.0));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
