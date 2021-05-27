#include "Vehicle.hpp"
#include "Navigation.hpp"
#include <iostream>

Vehicle2::Vehicle2(int id, int from, int to, NavigationType2 nt, bool reroute)
: id(id), from(from), current(from), to(to), reroutable(reroute), nav_type(nt) {}

Vehicle2::~Vehicle2(){
	// Navigation::g2.removeReservation(reservations,id);
}

bool Vehicle2::nextStreet(float turn_time){
	edges_used.push_back(hyperedge_ref{current});
	been_through.insert(hyperedge_ref{current});
	time_spent_on_edge.push_back(turn_time - last_turn);

	removeOccupation(current);

	if (!route.empty()) {

		last_turn = turn_time;
		current = route[0].id;
		edges_of_route++;
		route.erase(route.begin());
		
		addOccupation(current);

		#if 0 // logging
		std::cout << "v: " << time_spent_on_edge.back() << std::endl;
		std::cout << "route: ";
		for(auto&& r:route)
			std::cout << r.id << '#' << r.level << ' ';
		std::cout << std::endl;
		#endif
		
		return true;
	}
	return false;
}


void Vehicle2::newRoute(std::vector<hyperedge_ref> && new_route, long long int ns, float when){
	#if 0 // logging
	std::cout << "vehicle got a new route: ";
	for(auto&& r:new_route)
		std::cout << r.id << '#' << r.level << '/' << Navigation::g2[r].weight << ' ';
	std::cout << std::endl;
	#endif

	// all vehicles create reservations/occupations
	removeReservations();
	route = std::move(new_route);
	createReservations(when);

	number_of_scans += ns;
	rerouted++;
}

bool Vehicle2::timeToReroute(){
	if(reroutable && better_reroute){
		printf("Vehicle %i rerouted due to a congestion/edge removal\n",id);
		better_reroute = false;
		return true;
	}
	
	if(nav_type == NavigationType2::DIJKSTRA_WPL) {
		// the vehicle has not finished driving && the next hyperedge is not on the base layer
		return (route.size() > 0) && (route.front().level != 0);
	}

	return edges_of_route % Navigation::astar_jumps == 0;
}

void Vehicle2::createReservations(float when){
	if(when == -1) when = start_of_drive;

	for(auto& he : route){
		when += Navigation::expectedEdgeWeight(he, when);
		
		// correct to have it here. when computing time, only vehicles in front of us should be taken to account
		Navigation::g2.addReservation(he,id,when,nav_type == NavigationType2::DIJKSTRA_WPL);
		reservations.insert(he);
	}
}

void Vehicle2::moveReservations(float next_checkpoint_reached){
	removeReservations();
	createReservations(next_checkpoint_reached);
}

void Vehicle2::removeReservations(){	
	Navigation::g2.removeReservation(route, id);
}

void Vehicle2::addOccupation(int where){
	Navigation::g2.vehicles_on_edge[where].insert(id);
}

void Vehicle2::removeOccupation(int where){
	Navigation::g2.vehicles_on_edge[where].erase(id);
}

float Vehicle2::routeTime(){
	float f = 0;
	for(auto i : time_spent_on_edge){
		f += i;
	}
	return f;
}
