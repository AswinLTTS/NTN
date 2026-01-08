#include "traffic.h"
#include "ns3/udp-server-helper.h"
#include "ns3/udp-client-helper.h"
#include "ns3/log.h"

using namespace ns3;

/* =======================
   UDP SERVERS (Satellites)
   ======================= */
void InstallUdpServers(NodeContainer sats, uint16_t port)
{
    UdpServerHelper udpServer(port);

    ApplicationContainer serverApps;
    serverApps = udpServer.Install(sats);

    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(1000.0));

    NS_LOG_UNCOND("UDP servers installed on satellites");
}

/* =======================
   UDP CLIENT (Ground Station)
   ======================= */
void InstallUdpClient(Ptr<Node> gs)
{
    // IMPORTANT:
    // This IP is NOT a physical interface IP
    // It is the satellite LOOPBACK address
    // Routing decides which satellite gets traffic
    Ipv4Address serviceIp("192.168.0.1");
    uint16_t port = 4000;

    UdpClientHelper udpClient(serviceIp, port);
    udpClient.SetAttribute("MaxPackets", UintegerValue(1000000));
    udpClient.SetAttribute("Interval", TimeValue(MilliSeconds(200)));
    udpClient.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApp = udpClient.Install(gs);
    clientApp.Start(Seconds(2.0));
    clientApp.Stop(Seconds(1000.0));

    NS_LOG_UNCOND("UDP client installed on GS");
}
