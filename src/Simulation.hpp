#ifndef BP_SIMULATION_HPP
#define BP_SIMULATION_HPP

#include "EventManager.hpp"
#include "Preprocessing.hpp"
#include "Vehicle.hpp"

class Event;
class EventManager;

class Simulation {
private:
    EventManager event_manager;

	unsigned int number_of_vehicles_to_run = 30;
	unsigned int number_of_routes = 0; // # routes the vehicles should use
    unsigned int vehicle_id = 0; // next vehicle id
    
    // generates a vehicle with random route endpoints
    void genVehicleRand();
    // generates a vehicle with endpoints from the pair
    void genVehicleST(std::pair<int,int>);
    // generates routes 
    void genRoutes();

public:
    Random r{123};

	unsigned int getNumberOfVehicles() { return number_of_vehicles_to_run; }
	unsigned int getNumberOfRoutes() { return number_of_routes; }
	void setNumberOfVehicles(unsigned int nv) { number_of_vehicles_to_run = nv; }
    void setNumberOfRoutes(unsigned int nr) { number_of_routes = nr; }
    // runs the simulation set up with the config that can be listed by command 'conf'
    void run(); 
    // does what clear() does and sets the random seed to the same as on start up
    void reset();
    // clears the used routes, vehicles and stats about the finished routes
    void clear();

    bool random_routes = true;
    // if any routes are specified, the number of routes is overriden.
    std::vector<std::pair<int,int>> routes;
};

#endif