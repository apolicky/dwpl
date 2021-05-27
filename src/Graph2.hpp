#ifndef BP_GRAPH2_HPP
#define BP_GRAPH2_HPP

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "Vehicle.hpp"

// hyperedge_ref -> defined in "Vehicle.hpp"

struct hyperedge {
    hyperedge() {};
    hyperedge(float w) { weight = w; };

    float weight = 0.f;
    std::vector<int> neighbours;
    std::vector<int> children;
    int parent = -1;
};

struct gps {
    gps(){};
    gps(float x_, float y_) { x = x_; y = y_; };
    
	float x = -1.f;
    float y = -1.f;

    gps operator-(const gps& o) const {
        return gps{x - o.x, y - o.y};
    }
    
    gps operator+(const gps& o) const {
        return gps{x + o.x, y + o.y};
    }
};

class Graph2
{
private:
    int vehicle_id = 0;

    void dfs(std::set<int>& use_these, std::map<int,bool>& visited, int level, int index);

public:
    

    Graph2(/* args */);
    ~Graph2();

    std::string name;

    void reset();

    // --- EDGES ---------

    std::vector</*layers*/ std::vector</*edges*/ hyperedge>> edges;

    // --- VEHICLES ---------

    bool vehicles_reroutable = true;
    std::map<int, Vehicle2> vehicles;
    // creates a vehicle with route endpoints 's' and 't' and returns its id
    int addVehicle(int s, int t, NavigationType2 nt = NavigationType2::UNKNOWN);
    // removes the vehicle 'vid'
    void removeVehicle(const vehicle_ref& vid);

    // --- CONGESTIONS ---------

    std::map</*where*/hyperedge_ref,/*until when*/std::pair<float,float>> known_congestions;

    // --- RESERVATIONS ---------

    long long number_of_scans = 0;
    long long rsrv_scan = 0;

    std::map</*edge_id*/ hyperedge_ref, /*vehicles(ids) that have reservation and when*/ std::map<int, float>> rsrv_vehs_times;
    std::map</*edge_id*/ hyperedge_ref, /*when and vehicles(ids) that have reservation*/ std::multimap<float, int>> rsrv_times_vehs;
    std::map</*edge_id*/ hyperedge_ref, /*vehicles(ids) that reserved edges child and the first start*/ std::map<int,float>> rsrv_children_vehs_times;
    std::map</*edge_id*/ hyperedge_ref, /*when and vehicles(ids) that reserved edges child*/ std::multimap<float, int>> rsrv_children_times_vehs;
    std::map<hyperedge_ref, std::set<int>> vehicles_on_edge;

    // adds reservation on 'he' and all of its parents and children appropriately
    void addReservation(const hyperedge_ref& he, int vehicle_id, float when, bool multi_layer);
    // suggests to reroute to the vehicle with reservation on 'he' and only 'he' in case of congestion
    void problemOnReservedEdge(const hyperedge_ref& he, float from = -1.f, float to = MAXFLOAT);
    // removes reservations of vehicle 'vid' on all hyperedges of 'rte'
    void removeReservation(std::vector<hyperedge_ref>& rte, int vehicle_id);

    // --- DYNAMIC CHANGES IN NETWORK ----------
    
    std::set<int> removed_edges;

    // adds a new hyperedge to graph 
    void addEdge(float weight, gps a, gps b, std::vector<int>& neighbours);
    // puts back removed edge with index 'index'
    void addEdge(int index);
    // removes hyperedge with index 'index', informs all vehicles present or with reservation on it
    void removeEdge(int index, float curr_time, bool inform_upper_layer = false); 
    
    bool isConnected(const hyperedge_ref& he);
    void splitComponent(const hyperedge_ref& he, float when, bool inform_upper_layer = false);
    void createComponent(const hyperedge_ref& he, std::set<int>& poss_children);
    bool updatedNeighbourRelations(const hyperedge_ref& parent);

    // --- LANDMARKS ---------

    std::vector<int> landmark_ids;
    std::vector<std::map</*other_edge*/ int, /*potential*/ float>> landmark_potentials;

    // --- GPSES ---------

    std::map<hyperedge_ref,std::vector<gps>> edges_gpses;
    gps avgGPSOf(int h);
    gps avgGPSOf(const hyperedge_ref& h);

    // graph size for rendering
    gps max_size{-1, -1};
    gps min_size{99999999.f, 99999999.f};
    void changeSize(gps new_gps);

    // --- OPERATORS ---------

    hyperedge& operator[](const hyperedge_ref& hr);
    std::vector<hyperedge>& operator[](int level);
    Vehicle2& operator[](const vehicle_ref& vehicle_id);
};

#endif
