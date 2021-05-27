#ifndef BP_VEHICLE_HPP
#define BP_VEHICLE_HPP

#include <map>
#include <numeric>
#include <set>
#include <vector>

struct vehicle_ref{
    vehicle_ref(){};
    vehicle_ref(int id) : id(id) {};
    int id = -1;
};

struct hyperedge_ref {
    hyperedge_ref() {};
    hyperedge_ref(int i) { level = 0; id = i; }
    hyperedge_ref(int lvl, int i) { level = lvl; id = i; }

    int level = -1;
    int id = -1;

    bool operator==(const hyperedge_ref h1) const {
        return level == h1.level && id == h1.id;
    }

    bool operator<(const hyperedge_ref h1) const {
        if(level < h1.level) return true;
        else if(level == h1.level) return id < h1.id;
        else return false;
    }

    bool operator!=(const hyperedge_ref h1) const {
        return level != h1.level || id != h1.id;
    }
};

enum class NavigationType2 {
	DIJKSTRA_EDGED,
    ASTAR_EDGED,
    DIJKSTRA_WPL,
	UNKNOWN
};

class Vehicle2 {
private:
	std::vector<hyperedge_ref> route, edges_used;
    std::vector<float> time_spent_on_edge;
    std::set<hyperedge_ref> reservations;
    
    bool reroutable = true;
	
    float start_of_drive = -1.f, last_turn = -1.f; // route timing
    long long int number_of_scans = 0;
	int rerouted = -1; // first route is not rerouting
	int edges_of_route = 0;


    // --- RESERVATIONS ---------

    void createReservations(float when = -1);
    void removeReservations();

    void addOccupation(int where);
    void removeOccupation(int where);
    
public:
    Vehicle2(int id, int from, int to, NavigationType2 nt = NavigationType2::UNKNOWN, bool reroute = true);
    ~Vehicle2();

    // --- GETTERS ---------

    int id, from, to, current;
    NavigationType2 nav_type = NavigationType2::UNKNOWN;
    
    float getStartOfDrive() { return start_of_drive; }
    std::vector<hyperedge_ref> & getRoute() { return route; }
	
    // --- ROUTING ---------

    bool better_reroute = false; // indicator for vehicle to ask for a new route. Set to true when a congestion happens
    std::set<hyperedge_ref> been_through; // hyperedges vehicle used during traversal

    // sets new route 'new_route' for vehicle
    void newRoute(std::vector<hyperedge_ref> && new_route, long long int ns, float when = -1.f);
    //  indicates whether vehicle has any more edges to go
    bool nextStreet(float turn_time = -1.f);
    // check whether there are any edges on base layer (DwPL) or astar_jumps limit reached
    bool timeToReroute(); 
    // starts the timing of vehicles route
    void startDriving(float when) {start_of_drive = last_turn = when; }

    // --- RESERVATIONS ---------
    
    // makes the vehicle to move its reservations wrt time it is going to reach next hyperedge
    void moveReservations(float next_checkpoint_reached);

    // --- ROUTE LOGS ---------

    // get the number of scanned hyperdedges for this vehicle
	long long int nrScans() { return number_of_scans; }
	// compute the time vehcile spent driving
    float routeTime();
    // get the number of reroutes
    int reroutedTimes() { return rerouted; }
    // get the edges vehicle used during traversal
    std::vector<hyperedge_ref>& edgesUsed() { return edges_used; }
    // get the times vehicle spent on each edge.
    std::vector<float>& timesOnEdges() { return time_spent_on_edge; }
    

    // --- COMMENTED OUT ---------

    // bool usingReroutingNavigation() { return reroutable; }
    // bool nextStreet();
    // int nrEdgesOfRoute() { return edges_of_route; }

    // int getStart() { return from; }
    // int getCurrent() { return current; }
    // int getDestination() { return to; }
};

#endif
