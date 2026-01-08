#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/ipv4-static-routing-helper.h"

#include <cmath>
#include <vector>
#include <sstream>

#include <fstream>

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
std::ofstream orbitLog;
std::ofstream gsLinkLog;

static std::vector<GsLink> gsLinks;

/* =======================
   ORBIT UPDATE (ENGINE CORE)
   ======================= */
void UpdateOrbit(Ptr<Node> node, double radius, double omega,double phase, double raan)
{
    double t = Simulator::Now().GetSeconds();
    double angle = omega * t + phase;
    const double INCLINATION_DEG = 53.0;
    const double INCLINATION_RAD = INCLINATION_DEG * M_PI / 180.0;
    
    double x0 = radius * cos(angle);
    double y0 = radius * sin(angle) * cos(INCLINATION_RAD);
    double z0 = radius * sin(angle) * sin(INCLINATION_RAD);

    // Rotate plane by RAAN
    double x = x0 * cos(raan) - y0 * sin(raan);
    double y = x0 * sin(raan) + y0 * cos(raan);
    double z = z0;

    node->GetObject<MobilityModel>()->SetPosition(Vector(x, y, z));

    orbitLog << t << ","
             << node->GetId() << ","
             << x << ","
             << y << ","
             << z << "\n";

    Simulator::Schedule(
        Seconds(1.0),   // 1 second granularity is enough
        &UpdateOrbit,
        node,
        radius,
        omega,
        phase,
    raan);
}
/* =======================
   ELEVATION ANGLE COMPUTATION
   ======================= */
double ComputeElevationAngle(Ptr<MobilityModel> gsMob, Ptr<MobilityModel> satMob){
    Vector G = gsMob->GetPosition();
    Vector S = satMob->GetPosition();

    Vector v = S - G;   // GS → SAT
    Vector u = G;       // Earth center → GS

    // Normalize u
    double uMag = std::sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
    u.x /= uMag;
    u.y /= uMag;
    u.z /= uMag;

    // Dot product
    double dot = v.x*u.x + v.y*u.y + v.z*u.z;
    double vMag = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

    double sinElev = dot / vMag;

    // Clamp for safety
    sinElev = std::max(-1.0, std::min(1.0, sinElev));

    return std::asin(sinElev) * 180.0 / M_PI; // degrees
}

/* =======================
   GS VISIBILITY & HANDOVER
   ======================= */
void UpdateGsVisibility(double maxGsDistance)
{
   
    Ptr<Node> gs = gsLinks[0].gs;
    Ptr<MobilityModel> gsMob = gs->GetObject<MobilityModel>();

    GsLink* bestLink = nullptr;
    double bestElev = -90.0;
    /* Find closest visible satellite */
    for (auto &link : gsLinks)
    {
        Ptr<MobilityModel> satMob =
            link.sat->GetObject<MobilityModel>();

        double elev = ComputeElevationAngle(gsMob, satMob);

        if (elev >= 10.0 && elev > bestElev){
            bestElev = elev;
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
    // 🔁 Recompute routes after interface state changes
    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();
    static uint32_t currentServingSat = UINT32_MAX;

    if (bestLink)
    {   
        currentServingSat = bestLink->sat->GetId();
        NS_LOG_UNCOND("t=" << Simulator::Now().GetSeconds()
            << " GS connected to SAT "
            << bestLink->sat->GetId()
            << " elev=" << bestElev);
    }

    else
    {
    NS_LOG_UNCOND("t=" << Simulator::Now().GetSeconds()
        << " GS has NO visible satellite");
    }

    Simulator::Schedule(
        Seconds(1.0),
        &UpdateGsVisibility,
        maxGsDistance);
}
void RxTrace(Ptr<const Packet> p)
{
    std::cout << "RX packet size=" << p->GetSize()
              << " at t=" << Simulator::Now().GetSeconds()
              << "s\n";
}
void UpdateClientTarget(Ptr<UdpClient> client)
{
    if (currentServingSat == UINT32_MAX) return;

    Ptr<Ipv4> ipv4 = NodeList::GetNode(currentServingSat)->GetObject<Ipv4>();
    Ipv4Address newIp = ipv4->GetAddress(1,0).GetLocal();

    client->SetRemote(newIp, 4000);

    Simulator::Schedule(Seconds(1.0),
        &UpdateClientTarget,
        client);
}


/* =======================
   MAIN
   ======================= */
int main(int argc, char *argv[])
{
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        uint32_t numSatellites = 100;
        double orbitRadiusKm = 6371 + 550;     // Earth + LEO altitude
        double satelliteSpeed = 7500;          // m/s
        double baseOmega = satelliteSpeed / (orbitRadiusKm * 1000);

        CommandLine cmd;
        cmd.AddValue("numSatellites", "Number of satellites", numSatellites);
        cmd.Parse(argc, argv);
        orbitLog.open("orbit.csv");
        orbitLog << "time,sat_id,x,y,z\n";
        gsLinkLog.open("gs_links.csv");
        gsLinkLog << "time,gs_id,sat_id\n";


    /* ---------- Nodes ---------- */
    NodeContainer satellites;
    satellites.Create(numSatellites);

    NodeContainer groundStations;
    groundStations.Create(1);

    /* ---------- Internet ---------- */
    InternetStackHelper internet;
    internet.Install(satellites);
    internet.Install(groundStations);
    Ipv4AddressHelper loopback;
    loopback.SetBase("192.168.0.0", "255.255.255.255");

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        Ptr<Ipv4> ipv4 = satellites.Get(i)->GetObject<Ipv4>();
        int32_t ifIndex = ipv4->AddInterface(CreateObject<LoopbackNetDevice>());

        std::ostringstream ip;
        ip << "192.168.0." << (i + 1);

        ipv4->AddAddress(
            ifIndex,
            Ipv4InterfaceAddress(
                Ipv4Address(ip.str().c_str()),
                Ipv4Mask("255.255.255.255")
            )
        );

        ipv4->SetUp(ifIndex);
    }



    /* ---------- Mobility ---------- */
    MobilityHelper satMob;
    satMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    satMob.Install(satellites);

    MobilityHelper gsMob;
    gsMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gsMob.Install(groundStations);
    //GS position
    // double lat = 19.6868 * M_PI / 180.0;  // radians
    // double lon = 83.2185 * M_PI / 180.0;  // radians
    // double R   = 6371e3;                  // Earth radius (meters)

    // double x = R * cos(lat) * cos(lon);
    // double y = R * cos(lat) * sin(lon);
    // double z = R * sin(lat);

    groundStations.Get(0)->GetObject<MobilityModel>()->SetPosition(
        Vector(6371e3, 0, 0)
    );


    /* ---------- Orbit Scheduling ---------- */
    const uint32_t NUM_PLANES = 4;
    const double RAAN_STEP = M_PI / 2; // 90 degrees

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
                orbitRadiusKm*1000.0,
                baseOmega,
                phase,
                raan   // 👈 now used correctly
            );
        }
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
    // NetDeviceContainer link =
    // gsLink.Install(groundStations.Get(0), satellites.Get(0));
    // allDevices.push_back(link);
    // gsLinks.push_back({
    //     groundStations.Get(0),
    //     satellites.Get(0),
    //     link
    // });


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

    Ipv4StaticRoutingHelper staticRoutingHelper;
    Ptr<Ipv4> gsIpv4 = groundStations.Get(0)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> gsStaticRouting =
        staticRoutingHelper.GetStaticRouting(gsIpv4);

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        std::ostringstream ip;
        ip << "192.168.0." << (i + 1);
        Ipv4Address satLoop(ip.str().c_str());

        // Interface index of GS -> SAT link
        uint32_t iface = gsIpv4->GetInterfaceForDevice(
            gsLinks[i].devices.Get(0)
        );

        gsStaticRouting->AddHostRouteTo(
            satLoop,
            iface
        );
    }


    uint16_t port = 4000;

    /* ---------- UDP APPs ---------- */
    ApplicationContainer serverApp;
    UdpServerHelper udpServer(port);
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        serverApp.Add(udpServer.Install(satellites.Get(i)));
    }
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
    Ptr<Application> app = serverApp.Get(i);
    Ptr<UdpServer> server = DynamicCast<UdpServer>(app);
    if (server)
    {
        server->TraceConnectWithoutContext("Rx", MakeCallback(&RxTrace));
    }
    }

    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(100.0));
    
    // IP of Satellite 0 (check your addressing!)
    // Client on GS
    // Ptr<Ipv4> ipv4 = satellites.Get(0)->GetObject<Ipv4>();
    // Ipv4Address satIp = ipv4->GetAddress(1, 0).GetLocal();
    // std::cout << "Satellite IP = " << satIp << std::endl;

    Ipv4Address serviceIp("192.168.0.1");
    UdpClientHelper udpClient(serviceIp, port);

    //UdpClientHelper udpClient(satIp, port);
    udpClient.SetAttribute("MaxPackets", UintegerValue(100000));
    udpClient.SetAttribute("Interval", TimeValue(MilliSeconds(200)));
    udpClient.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApp = udpClient.Install(groundStations.Get(0));
    Ptr<Application> app = clientApp.Get(0);
    Ptr<UdpClient> udp = DynamicCast<UdpClient>(app);
    Simulator::Schedule(Seconds(1.1),&UpdateClientTarget,udp);

    clientApp.Start(Seconds(2.0));
    clientApp.Stop(Seconds(100.0));



    /* ---------- GS Visibility Controller ---------- */
    double maxGsDistance = 7500e3; // 7500 km
    Simulator::Schedule(
        Seconds(1.0),
        &UpdateGsVisibility,
        maxGsDistance);
    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();

    /* ---------- NetAnim (temporary visualization) ---------- */
    // AnimationInterface anim("phase1-day4-engine.xml");

    // for (uint32_t i = 0; i < satellites.GetN(); ++i)
    // {
    //     anim.UpdateNodeDescription(satellites.Get(i), "SAT");
    //     anim.UpdateNodeColor(satellites.Get(i), 0, 255, 255);
    // }

    // anim.UpdateNodeDescription(groundStations.Get(0), "GS");
    // anim.UpdateNodeColor(groundStations.Get(0), 255, 0, 0);

    /* ---------- Run ---------- */
    Simulator::Stop(Seconds(3600));
    Simulator::Run();
    Simulator::Destroy();

    orbitLog.close();
    gsLinkLog.close();

    return 0;
}
