#pragma once
#include "ns3/applications-module.h"

void InstallUdpServers(ns3::NodeContainer sats, uint16_t port);
void InstallUdpClient(ns3::Ptr<ns3::Node> gs);
