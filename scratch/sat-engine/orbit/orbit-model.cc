#include "orbit.h"

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include <cmath>

using namespace ns3;

void UpdateOrbit(
    Ptr<Node> node,
    double radius,
    double omega,
    double phase,
    double raan
)
{
    double t = Simulator::Now().GetSeconds();
    double angle = omega * t + phase;

    const double INCLINATION_DEG = 53.0;
    const double INCLINATION_RAD = INCLINATION_DEG * M_PI / 180.0;

    // Base orbital plane
    double x0 = radius * std::cos(angle);
    double y0 = radius * std::sin(angle) * std::cos(INCLINATION_RAD);
    double z0 = radius * std::sin(angle) * std::sin(INCLINATION_RAD);

    // Rotate plane by RAAN
    double x = x0 * std::cos(raan) - y0 * std::sin(raan);
    double y = x0 * std::sin(raan) + y0 * std::cos(raan);
    double z = z0;

    node->GetObject<MobilityModel>()->SetPosition(Vector(x, y, z));

    // Log orbit
    orbitLog << t << ","
             << node->GetId() << ","
             << x << ","
             << y << ","
             << z << "\n";

    // Reschedule
    Simulator::Schedule(
        Seconds(1.0),
        &UpdateOrbit,
        node,
        radius,
        omega,
        phase,
        raan
    );
}
