#include "Simulation.hpp"

void Simulation::genVehicleRand(){   
    event_manager.vehicleSpawn();
}

void Simulation::genVehicleST(std::pair<int,int> st) {
	event_manager.vehicleSpawn(st.first,st.second);
}

void Simulation::genRoutes(){
    // #define SF2K_EFFICIENCY // uncomment if you want to replicate the efficiency test figs 4.5

    #ifndef SF2K_EFFICIENCY
    for(auto i = 0; i < number_of_routes; i++){
        auto s = r.nextInt(Navigation::g2.edges[0].size());
        auto t = r.nextInt(Navigation::g2.edges[0].size());
        routes.push_back(std::make_pair(s,t));
    }
    #endif

    #ifdef SF2K_EFFICIENCY
    routes.push_back(std::make_pair(3/*top left*/,2519/*bottom right*/));
    #endif
}

void Simulation::run(){

    // if some routes are specified, use only those.
    // else if number of routes is specified, generate them and use them by mod.
    // else if number of routes is NOT specified, generate vehicles randomly, i.e. generate V routes for V vehicles and use i-th route for i-th vehicle
    
    // there are no routes from the previous simulation run
    if(!routes.size() && number_of_routes){
        // generate 'number_of_routes' routes
        genRoutes();
    }
    // as above and the user did not specify the #routes -> each vehicle gets a random route
    else if(!routes.size() && number_of_routes == 0){
        number_of_routes = number_of_vehicles_to_run;
        genRoutes();
    }
    // else use the routes from previous run of the simulation

    for(auto i = 0; i < number_of_vehicles_to_run; i++){
        genVehicleST(routes[i % routes.size()]);
    }
    
	Navigation::prepIdealRoutes(routes);

    // while there is something to do, do it.
    while(!event_manager.empty()){
        event_manager.runNextEvent();
    }

	Navigation::clearRoutes();
    // Navigation::g2.reset();
}

void Simulation::reset(){
    clear();
    event_manager = EventManager();
    r.reseed();
}

void Simulation::clear() {
    Navigation::g2.reset();
    routes.clear();
}
