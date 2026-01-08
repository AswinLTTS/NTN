#pragma once

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include <fstream>

// Extern log (defined in main.cc)
extern std::ofstream orbitLog;

void UpdateOrbit(
    ns3::Ptr<ns3::Node> node,
    double radius,
    double omega,
    double phase,
    double raan
);
double ComputeElevationAngle(
    ns3::Ptr<ns3::MobilityModel> gsMob,
    ns3::Ptr<ns3::MobilityModel> satMob
);