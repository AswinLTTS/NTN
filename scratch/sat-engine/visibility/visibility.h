#pragma once

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "links.h"   // for GsLink

double ComputeElevationAngle(
    ns3::Ptr<ns3::MobilityModel> gs,
    ns3::Ptr<ns3::MobilityModel> sat
);

void UpdateGsVisibility(double maxGsDistance);

// shared GS–SAT links
extern std::vector<GsLink> gsLinks;
