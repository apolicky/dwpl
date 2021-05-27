#ifndef BP_EVENT_HPP
#define BP_EVENT_CPP

#include <queue>

#include "Random.hpp"
#include "Navigation.hpp"

struct EventComparator;

enum class EventType{
	START_DRIVING,
    CHECKPOINT_REACHED,
    DESTINATION_REACHED,
    VEHICLE_SPAWN,
    VEHICLE_SPAWN_ST,
    EDGE_CONGESTION_END,
	ROUTE_NOT_FOUND,
    NONE
};

class Event2{
private:
    // --- VEHICLE DRIVING ----------
    
    // checks vehicles route, reroutes if necessary
    bool checkpointReached(Event2& new_e);
    // creates stats when vehicle reached its destination
    bool destinationReached(Event2& new_e);
	bool routeNotFound(Event2& new_e);

    // --- VEHICLE SPAWNING ----------

	// creates a new vehicle with random route and makes it start its drive.
	bool vehicleSpawn(Event2& new_e, Random& r);
    // creates a new vehicle with route from s to t
    bool vehicleSpawn(Event2& new_e, int s, int t);
	// makes vehicle start driving or says that no route is found.
	bool startDriving(Event2& new_e);

    // --- CONGESTION ---------
    bool congestionEnd(Event2& new_e);

public:
    Event2(int id);
    Event2(int id, float when, EventType type);
    Event2(int id, float when, EventType type, std::pair<int,int> st_r);
    Event2(int id, float when, hyperedge_ref he);
    ~Event2();

    bool operator==(const Event2&) const;

    int id;
    float time = -1.f;
    EventType type = EventType::NONE;

    vehicle_ref veh_ref{-1};
    hyperedge_ref h;
    std::pair<int,int> st;

    // makes event do its action, if necessary using 'r' for random choices
    bool doAction(Event2& new_e, Random & r);
};

struct EventComparator{
    bool operator()(const Event2& e_1, const Event2& e_2){
        return e_1.time > e_2.time;
    }
};

#endif