#include "visibility.h"
#include "ns3/internet-module.h"
#include "ns3/simulator.h"
#include <cmath>

using namespace ns3;

double ComputeElevationAngle(
    Ptr<MobilityModel> gsMob,
    Ptr<MobilityModel> satMob)
{
    Vector G = gsMob->GetPosition();
    Vector S = satMob->GetPosition();

    Vector v = S - G;   // GS → SAT
    Vector u = G;       // Earth center → GS

    // Normalize u
    double uMag = std::sqrt(u.x*u.x + u.y*u.y + u.z*u.z);
    u /= uMag;

    // Projection
    double dot = v.x*u.x + v.y*u.y + v.z*u.z;
    double vMag = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

    double sinElev = dot / vMag;
    sinElev = std::clamp(sinElev, -1.0, 1.0);

    return std::asin(sinElev) * 180.0 / M_PI;
}
void UpdateGsVisibility(double maxGsDistance)
{
    Ptr<Node> gs = gsLinks[0].gs;
    Ptr<MobilityModel> gsMob = gs->GetObject<MobilityModel>();

    GsLink* bestLink = nullptr;
    double bestElev = -90.0;

    for (auto &link : gsLinks)
    {
        Ptr<MobilityModel> satMob =
            link.sat->GetObject<MobilityModel>();

        double elev = ComputeElevationAngle(gsMob, satMob);

        if (elev >= 10.0 && elev > bestElev)
        {
            bestElev = elev;
            bestLink = &link;
        }
    }

    for (auto &link : gsLinks)
    {
        for (uint32_t i = 0; i < link.devices.GetN(); ++i)
        {
            Ptr<NetDevice> dev = link.devices.Get(i);
            Ptr<Ipv4> ipv4 = dev->GetNode()->GetObject<Ipv4>();
            int32_t iface = ipv4->GetInterfaceForDevice(dev);
            if (iface < 0) continue;

            bool up = (&link == bestLink);
            up ? ipv4->SetUp(iface) : ipv4->SetDown(iface);
        }
    }

    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();

    if (bestLink)
    {
        NS_LOG_UNCOND(
            "t=" << Simulator::Now().GetSeconds()
            << " GS → SAT "
            << bestLink->sat->GetId()
            << " elev=" << bestElev
        );
    }
    else
    {
        NS_LOG_UNCOND(
            "t=" << Simulator::Now().GetSeconds()
            << " GS has NO visible satellite"
        );
    }

    Simulator::Schedule(
        Seconds(1.0),
        &UpdateGsVisibility,
        maxGsDistance
    );
}
