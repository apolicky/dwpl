#include "Event.hpp"
Event2::Event2(int id) : id(id) {}

Event2::Event2(int id, float when, EventType ty)
: id(id), time(when), type(ty) {}

Event2::Event2(int id, float when, EventType ty, std::pair<int,int> st)
: id(id), st(st), time(when), type(ty) {}

Event2::Event2(int id, float when, hyperedge_ref he)
: id(id), time(when), type(EventType::EDGE_CONGESTION_END), h(he) {}

Event2::~Event2(){
}

bool Event2::operator==(const Event2& e2) const {
	return id == e2.id;
}

bool Event2::doAction(Event2& new_e, Random& r){
    switch (type) {
    case EventType::CHECKPOINT_REACHED:
        return checkpointReached(new_e);
    case EventType::DESTINATION_REACHED:
        return destinationReached(new_e);
    case EventType::VEHICLE_SPAWN:
        return vehicleSpawn(new_e,r);
	case EventType::VEHICLE_SPAWN_ST:
        return vehicleSpawn(new_e, st.first, st.second);
	case EventType::ROUTE_NOT_FOUND:
		return routeNotFound(new_e);
	case EventType::START_DRIVING:
		return startDriving(new_e);
	case EventType::EDGE_CONGESTION_END:
		return congestionEnd(new_e);

    default:
        return false;
    }
}

bool Event2::checkpointReached(Event2& new_e){
	auto& v = Navigation::g2[veh_ref];
	
	if(v.nextStreet(time)){
		if(v.timeToReroute()){
			long long ns = 0;
			auto start = std::chrono::steady_clock::now();
			auto nr = Navigation::findRoute(v.current, v.to, ns, time, v.nav_type, v.been_through);
			auto end = std::chrono::steady_clock::now();
			Navigation::query_times.push_back(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());

			if(nr.size() == 0 && v.current != v.to){
				printf("V%i HAS ROUTE LEN 0, CURR POSITION %i\n",v.id,v.current);
				new_e.type = EventType::ROUTE_NOT_FOUND;
				new_e.time = time;
				new_e.veh_ref = veh_ref;
				return true;
			}
			v.newRoute(std::move(nr), ns, time);

		}

		// push new time to event_queue.
		auto t = Navigation::currentEdgeWeight(hyperedge_ref(v.current), time); // newstyle
		
		new_e.time = time + t;
		new_e.type = EventType::CHECKPOINT_REACHED;
				
	} else {
		// no other street to take, destination is reached then
		new_e.time = time;
		new_e.type = EventType::DESTINATION_REACHED;
	}

	new_e.veh_ref = vehicle_ref(v.id);
	return true;
}

bool Event2::destinationReached(Event2& ne){
	auto& v = Navigation::g2[veh_ref];

	auto& ideal_route = Navigation::idealRoute(v.from,v.to);
    
	std::printf("dest_reached: v%3i f:%4i t:%4i dur:%.3f i_dur:%.3f ns:%9lli rerouted:%i e:%lu ideal:%lu\n",
		veh_ref.id,
		v.from,
		v.to,
		v.routeTime(),
		Navigation::routeWeight(ideal_route),
		v.nrScans(),
		v.reroutedTimes(),
		v.edgesUsed().size(),
		ideal_route.size()
	);

    return false;
}

bool Event2::routeNotFound(Event2& ne) {
	std::printf("route_not_found: v%3i did not reach destination %i.\n",
		veh_ref.id,
		Navigation::g2[veh_ref].to
	);

	return false;
} 

bool Event2::vehicleSpawn(Event2& ne, Random& r) {
	
	auto s = r.nextInt(Navigation::g2[0].size());
	auto t = r.nextInt(Navigation::g2[0].size());

	return vehicleSpawn(ne, s, t);
}

bool Event2::vehicleSpawn(Event2& ne, int s, int t){
	ne.time = time;
	ne.type = EventType::START_DRIVING;
	ne.veh_ref = vehicle_ref(Navigation::g2.addVehicle(s, t, Navigation::getNavigationType()));
	return true;
}

bool Event2::startDriving(Event2& ne) {
	
	auto& v = Navigation::g2[veh_ref];
	long long ns = 0;
	auto r = Navigation::findRoute(v.from, v.to, ns, time, v.nav_type);	

	if (r.size() > 0){
		v.startDriving(time);
		v.newRoute(std::move(r), ns);
		ne.type = EventType::CHECKPOINT_REACHED;
	}
	else {
		ne.type = EventType::ROUTE_NOT_FOUND;
	}
	ne.time = time;
	ne.veh_ref = veh_ref;
	return true;
}

bool Event2::congestionEnd(Event2& ne) {
	Navigation::g2.known_congestions.erase(h);
	return true;
}