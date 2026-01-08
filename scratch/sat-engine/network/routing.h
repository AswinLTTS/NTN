#pragma once

#include "ns3/internet-module.h"
#include "links.h"   // for GsLink

#include <vector>

/* =======================
   ROUTING APIs
   ======================= */

// Install static GS → SAT host routes
void InstallGsStaticRoutes(
    ns3::Ptr<ns3::Node> gsNode,
    const std::vector<GsLink>& gsLinks
);

// Recompute global routing tables (after handover)
void RecomputeGlobalRoutes();
