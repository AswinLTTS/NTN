#include "links.h"

using namespace ns3;

/* =======================
   Inter-Satellite Links
   ======================= */
void CreateIslLinks(
    NodeContainer satellites,
    std::vector<NetDeviceContainer>& allDevices)
{
    PointToPointHelper isl;
    isl.SetDeviceAttribute("DataRate", StringValue("2Gbps"));
    isl.SetChannelAttribute("Delay", StringValue("10ms"));

    uint32_t numSatellites = satellites.GetN();

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        uint32_t next = (i + 1) % numSatellites;

        NetDeviceContainer link =
            isl.Install(satellites.Get(i), satellites.Get(next));

        allDevices.push_back(link);
    }
}

/* =======================
   GS–SAT Links
   ======================= */
void CreateGsSatLinks(
    NodeContainer groundStations,
    NodeContainer satellites,
    std::vector<NetDeviceContainer>& allDevices,
    std::vector<GsLink>& gsLinks)
{
    PointToPointHelper gsLink;
    gsLink.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    gsLink.SetChannelAttribute("Delay", StringValue("30ms"));

    Ptr<Node> gs = groundStations.Get(0);
    uint32_t numSatellites = satellites.GetN();

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        NetDeviceContainer link =
            gsLink.Install(gs, satellites.Get(i));

        allDevices.push_back(link);

        gsLinks.push_back({
            gs,
            satellites.Get(i),
            link
        });
    }
}
