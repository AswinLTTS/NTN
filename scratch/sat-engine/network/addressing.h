#pragma once

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"

#include <vector>

void ConfigureSatelliteLoopbacks(
    ns3::NodeContainer satellites
);

void AssignPointToPointAddresses(
    const std::vector<ns3::NetDeviceContainer>& allDevices
);
