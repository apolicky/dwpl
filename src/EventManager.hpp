#ifndef BP_EVENT_MANAGER_HPP
#define BP_EVENT_MANAGER_HPP

#include "Event.hpp"

class EventQueue : public std::priority_queue<Event2,std::vector<Event2>,EventComparator>{
public:
    bool remove(const Event2& value) {
        auto it = std::find(this->c.begin(), this->c.end(), value);
        if (it != this->c.end()) {
            this->c.erase(it);
            std::make_heap(this->c.begin(), this->c.end(), this->comp);
            return true;
        }
        else {
            return false;
        }
    }

    void slowDownVehiclesOn(const hyperedge_ref& he, float dur, float current_time, bool vehicles_change_reservs = true){
        if(vehicles_change_reservs){
            for(auto& e : c){
                auto vr = e.veh_ref;
                if(vr.id != -1){
                    if(hyperedge_ref{Navigation::g2[vr].current} == he){
                        auto next_checkpoint_reached = e.time + dur;
                        e.time = next_checkpoint_reached;
                        Navigation::g2[vr].moveReservations(next_checkpoint_reached);
                    }
                }
            }    
        }
        else{
            for(auto& e : c){
                auto vr = e.veh_ref;
                if(vr.id != -1){
                    if(hyperedge_ref{Navigation::g2[vr].current} == he){
                        e.time += dur;
                    }
                }
            }
        }
        std::make_heap(this->c.begin(), this->c.end(), this->comp);
    }
};

// class that processes the simulation events
class EventManager
{
private:
    EventQueue eq;
    Random r{123};

    int next_event_id = 0;
    
    void upTheTime() { last_event_time += 0.1f; }

public:
    float last_event_time = 0;

    bool empty() { return eq.size() == 0; }
    // runs next event on the priority queue
    void runNextEvent();
    // adds an event to spawn a vehicle
    void vehicleSpawn(int s = -1, int t = -1);
    // creates new congestion on hyperedge 'e'
    void edgeCongestion(const hyperedge_ref& e, float when = -1.f, float duration = 30.f);
};



#endif