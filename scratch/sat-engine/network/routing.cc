#include "routing.h"

using namespace ns3;

/* =======================
   Static GS → SAT Routing
   ======================= */
void InstallGsStaticRoutes(
    Ptr<Node> gsNode,
    const std::vector<GsLink>& gsLinks)
{
    Ipv4StaticRoutingHelper staticRoutingHelper;
    Ptr<Ipv4> gsIpv4 = gsNode->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> gsStaticRouting =
        staticRoutingHelper.GetStaticRouting(gsIpv4);

    for (uint32_t i = 0; i < gsLinks.size(); ++i)
    {
        std::ostringstream ip;
        ip << "192.168.0." << (i + 1);
        Ipv4Address satLoop(ip.str().c_str());

        uint32_t iface =
            gsIpv4->GetInterfaceForDevice(gsLinks[i].devices.Get(0));

        gsStaticRouting->AddHostRouteTo(satLoop, iface);
    }
}

/* =======================
   Global Routing Refresh
   ======================= */
void RecomputeGlobalRoutes()
{
    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();
}
