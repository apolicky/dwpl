#include "EventManager.hpp"

void EventManager::runNextEvent(){
    auto e = eq.top();
    eq.pop();

    last_event_time = e.time;
    Event2 ne(next_event_id);
    
    if(e.doAction(ne,r)){
        eq.push(ne);
        next_event_id++;
    }
}

void EventManager::vehicleSpawn(int s, int t){
   if(s == -1 || t == -1) eq.push(Event2(next_event_id++, last_event_time, EventType::VEHICLE_SPAWN));
   else eq.push(Event2(next_event_id++, last_event_time, EventType::VEHICLE_SPAWN_ST,std::make_pair(s,t)));
   upTheTime();
}

void EventManager::edgeCongestion(const hyperedge_ref& e, float start, float duration){
    if(start == -1) start = last_event_time;
    eq.slowDownVehiclesOn(e, duration, start, Navigation::getNavigationType() == NavigationType2::DIJKSTRA_WPL);
    Navigation::g2.known_congestions[e] = std::make_pair(start, start + duration);
    Navigation::g2.problemOnReservedEdge(e, start, start + duration);
    printf("there is a congestion on %d#%d from %.2f to %.2f\n",e.id,e.level,start,start+duration);

    eq.push(Event2(next_event_id++, start + duration, e));
}