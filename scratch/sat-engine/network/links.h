#pragma once

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <vector>

/* =======================
   GS–SAT LINK STRUCT
   ======================= */
struct GsLink
{
    ns3::Ptr<ns3::Node> gs;
    ns3::Ptr<ns3::Node> sat;
    ns3::NetDeviceContainer devices;
};

/* =======================
   LINK CREATION APIs
   ======================= */

// Creates inter-satellite ring links
void CreateIslLinks(
    ns3::NodeContainer satellites,
    std::vector<ns3::NetDeviceContainer>& allDevices
);

// Creates GS–SAT links and fills gsLinks
void CreateGsSatLinks(
    ns3::NodeContainer groundStations,
    ns3::NodeContainer satellites,
    std::vector<ns3::NetDeviceContainer>& allDevices,
    std::vector<GsLink>& gsLinks
);
