#ifndef BP_NAVIGATION_HPP
#define BP_NAVIGATION_HPP

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <stack>
#include <vector>

#include "Graph2.hpp"
#include "Preprocessing.hpp"
// enum class NavigationType2 -> in "Vehicle"

using route = std::vector<hyperedge_ref>;

class Graph2;
class Preprocessing;
class Navigation;
class Simulation;

class Navigation {
private:
    static NavigationType2 nav_type;
	static std::map<std::pair<int, int>, route> ideal_routes;

	static route computeIdealRoute(int from, int to);
	static float expectedSlowDown(const hyperedge_ref& e, float when, bool multi_layer = false);
	// compute the route from predecessor map created when path finding
	static route parseRoute(hyperedge_ref last, const hyperedge_ref& from, std::map<hyperedge_ref, hyperedge_ref>& predecessor);
	static float trafficFlow(float occupation, float length);
	// #vehicles present on hyperedge 'he' in time 'when'
	static float vehiclesInTime(const hyperedge_ref& he, float when, bool multi_layer = false);

public:
	static Graph2 g2;
	static std::vector<int64_t> query_times;

	// if true, time to travel a street will be computed with respect to the streets occupation.
	static bool real_time_system;
	static unsigned int dwpl_jumps; // 
	static constexpr unsigned int astar_jumps = 5; // 
	static constexpr unsigned int specify_more = 0; // specifies the number of edges used on the 0th layer on top of 'dwpl_jumps'
	static constexpr float vehicles_per_unit = 30.f; // 30.f for real use, 5.f for throughput testing(experiments tab.4.4)

	// --- ROUTING ---------

	// compute the ideal route from 'from' to 'to'
	static route& idealRoute(int from, int to);
	// prepare ideal routes for pairs of endpoints in 'rts'
	static void prepIdealRoutes(const std::vector<std::pair<int, int>>& rts);
	// remove the ideal routes
	static void clearRoutes() { ideal_routes.clear(); }

	// --- PATH-FINDING ALGORITHMS ---------

	// compute a route using navigation type 'nt'
	static route findRoute(int from, int to, long long int& nodes_scanned, float when = -1, NavigationType2 nt = NavigationType2::UNKNOWN, std::set<hyperedge_ref> been_through = std::set<hyperedge_ref>());
	// compute a route using Dijkstra
	static route dijkstraEdged(int from, int to, long long int& nodes_scanned, float when = -1);
	// compute a route using A*
	static route aStarEdged(int from, int to, long long int& nodes_scanned, float when = -1, bool uses_potential = true, bool real_time = true);
	// compute a route using DwPL
	static route dijkstraWPL(int from, int to, long long int& nodes_scanned, float when, std::set<hyperedge_ref>& been_through, bool real_time = true);
	
	// compute the euclidean distance from 'x' to 'y' based on geographic coords
	static float euclideanDistance(int x, int y, int lvl = 0);
	// compute the distances from 'from' to every other hyperedge
	static std::map<int,float> dijkstraDistances(int from);

	// the time it is expected to take to traverse 'e' in time 'when'
	static float expectedEdgeWeight(const hyperedge_ref& e, float when = -1.f);
	// the time it takes to traverse hyperedge 'e' in time 'when'
	static float currentEdgeWeight(const hyperedge_ref& e, float when);
	// compute the weight of a route
	static float routeWeight(const route& rte);

	// --- NAVIGATION TYPE ---------

    static void setNavigationType(NavigationType2 nt) { Navigation::nav_type = nt; }
    static NavigationType2 getNavigationType() { return Navigation::nav_type; }

	// --- STATS ---------

	// compute the average query time
	static float avgQueryTime();

};


#endif
