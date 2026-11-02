# FUEL source and component notices

FUEL is pinned to HKUST-Aerial-Robotics/FUEL commit 662dd23c7b52b258d3c4a0155ff6632118e8984f. Its root GPLv3 LICENSE and README are retained in fuel/. EasyFly integration files carry their own source notices; linking does not relicense the upstream files.

The original bundled LKH files are retained byte-for-byte, including the RMIT notice in Delaunay.c. Upstream lkh_tsp_solver/package.xml says license TODO; an exact copy is retained here rather than replacing that declaration with Apache or claiming a separately verified LKH redistribution grant. This local simulation qualification does not establish commercial redistribution rights. No external LKH package is silently substituted.

The native high-level FIS, global LKH ordering, local graph refinement and A* are compiled. Original ROS1 transport/FSM and B-spline motion/control are outside the selected port. Own GCOPTER navigation executes the selected viewpoint. The original LKHmain.c is retained for provenance and excluded from the shared library; lkh_interface.cpp is the actual invoked solver entry.

2026-10-28 follow-up: the author’s university publication page describes LKH-3
as available without charge for academic and non-commercial use:
https://forskning.ruc.dk/da/publications/an-extension-of-the-lin-kernighan-helsgaun-tsp-solver-for-constra/
This does not identify the exact bundled LKH revision or establish its
redistribution terms. Direct author download pages timed out during review.
The original RMIT notice is specific to Delaunay.c. D-04 therefore retains a
release-permission gap for this component; no commercial or general
redistribution permission is asserted and no solver is silently substituted.

2026-11-02: the 119 selected files under fuel_planner/utils/lkh_tsp_solver are
now fingerprinted in ../../../license_reviews/d04-20261102/review.json (D04-LKH).
Their reproducible identity is the FUEL commit plus this exact subtree and file
hashes; an independent LKH release number is still unverified. The university's
LKH-3 description must not be used as this bundled solver's license or version.
The rights-holder clarification draft is in that review directory. No request
has been sent and no new redistribution grant has been obtained.
