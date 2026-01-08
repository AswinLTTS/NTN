#include "addressing.h"

#include "ns3/network-module.h"
#include "ns3/internet-module.h"

#include <sstream>

using namespace ns3;

/* =========================
   Satellite Loopback IPs
   ========================= */
void ConfigureSatelliteLoopbacks(NodeContainer satellites)
{
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        Ptr<Ipv4> ipv4 = satellites.Get(i)->GetObject<Ipv4>();

        // Create loopback interface
        int32_t ifIndex =
            ipv4->AddInterface(CreateObject<LoopbackNetDevice>());

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
}

/* =========================
   P2P Subnet Assignment
   ========================= */
void AssignPointToPointAddresses(
    const std::vector<NetDeviceContainer>& allDevices)
{
    Ipv4AddressHelper address;

    for (uint32_t i = 0; i < allDevices.size(); ++i)
    {
        std::ostringstream subnet;
        subnet << "10." << (i + 1) << ".0.0";

        address.SetBase(
            subnet.str().c_str(),
            "255.255.255.0"
        );

        address.Assign(allDevices[i]);
    }
}
