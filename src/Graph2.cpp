#include "Graph2.hpp"

Graph2::Graph2(){
    edges.push_back(std::vector<hyperedge>());
}

Graph2::~Graph2(){
}

// clears vehicles, congestrions, reservations, returns removed edges back
void Graph2::reset(){
    // vehicles
    vehicles.clear();
    // congestions
    known_congestions.clear();
    
    //reservations
    rsrv_scan = 0;
    rsrv_times_vehs.clear();
    rsrv_vehs_times.clear();
    rsrv_children_vehs_times.clear();
    rsrv_children_times_vehs.clear();

    // keep edges removed?
    for(auto removed_edge : removed_edges){
        addEdge(removed_edge);
    }
}

// --- OPERATORS ---------

hyperedge& Graph2::operator[](const hyperedge_ref& edge){
    return edges[edge.level][edge.id];
}

std::vector<hyperedge>& Graph2::operator[](int level){
    return edges[level];
}

Vehicle2& Graph2::operator[](const vehicle_ref& v_id){
    return vehicles.at(v_id.id);
}

// --- VEHICLES ---------

int Graph2::addVehicle(int s, int t, NavigationType2 nt) { 
    vehicles.emplace(vehicle_id, Vehicle2(vehicle_id, s, t, nt, vehicles_reroutable));
    return vehicle_id++; // up the number of vehicles
}

void Graph2::removeVehicle(const vehicle_ref& vid){
    if(vehicles.count(vid.id)){
        vehicles.erase(vid.id);
    }
}

// --- RESERVATIONS ---------

void Graph2::addReservation(const hyperedge_ref& he, int vehicle_id, float when, bool multi_layer){
    
    rsrv_vehs_times[he][vehicle_id] = when;
    rsrv_times_vehs[he].insert(std::pair{when,vehicle_id});

    if(multi_layer){
        hyperedge_ref parent{he.level + 1, edges[he.level][he.id].parent};
        while(parent.id != -1){
            if(!rsrv_children_vehs_times[parent].count(vehicle_id)){
                rsrv_children_vehs_times[parent][vehicle_id] = when;
                rsrv_children_times_vehs[parent].insert(std::pair{when,vehicle_id});
            }
            parent = hyperedge_ref(parent.level + 1, edges[parent.level][parent.id].parent);
        }
    }
    
}

void Graph2::removeReservation(std::vector<hyperedge_ref>& rte, int vehicle_id){
    for(auto& he : rte){
        if(rsrv_vehs_times.count(he)){
            auto p = rsrv_times_vehs[he].equal_range(rsrv_vehs_times[he][vehicle_id]);

            for (; p.first != p.second; ++p.first) {
                if (p.first->second == vehicle_id) { 
                    rsrv_times_vehs[he].erase(p.first);
                    break; // we know that one vehicle has only one reservation hence we can break here.
                }
            }
            rsrv_vehs_times[he].erase(vehicle_id);
        }

        hyperedge_ref parent(edges[he.level][he.id].parent);
        while(parent.id != -1){
            // for dijkstra and A* there will not be a record because we did not put it in
            if(rsrv_children_vehs_times[parent].count(vehicle_id)){
                auto p = rsrv_children_times_vehs[he].equal_range(rsrv_children_vehs_times[he][vehicle_id]);

                for (; p.first != p.second; ++p.first) {
                    if (p.first->second == vehicle_id) { 
                        rsrv_children_times_vehs[he].erase(p.first);
                        break; // we know that one vehicle has only one reservation hence we can break here.
                    }
                }
                rsrv_children_vehs_times[parent].erase(vehicle_id);
            }
            parent = hyperedge_ref(parent.level + 1, edges[parent.level][parent.id].parent);
        }
    }
}

void Graph2::problemOnReservedEdge(const hyperedge_ref& he, float from, float to){
    if(from == -1){
        from = 0;
        to = MAXFLOAT;
    }

    for(auto& res : rsrv_vehs_times[he]){
        if(res.second >= from && res.second <= to){
            vehicles.at(res.first).better_reroute = true;
        }    
    }
}


// --- GPSES ---------

gps Graph2::avgGPSOf(int h){
    hyperedge_ref hr{h};
    return avgGPSOf(hr);
}

gps Graph2::avgGPSOf(const hyperedge_ref& h){
    float x = 0;
    float y = 0;
    for(auto& g : edges_gpses[h]){
        x += g.x; y += g.y;
    }
    gps g{
        x / edges_gpses[h].size(), 
        y / edges_gpses[h].size()};
    return g;
}

void Graph2::changeSize(gps new_gps){
    max_size.x = std::max(new_gps.x,max_size.x);
	max_size.y = std::max(new_gps.y,max_size.y);
	min_size.x = std::min(new_gps.x,min_size.x);
	min_size.y = std::min(new_gps.y,min_size.y);
}

// --- DYNAMIC NETWORK CHANGES ---------

void Graph2::addEdge(float weight, gps a, gps b, std::vector<int>& neighbours){
    edges[0].push_back(hyperedge(weight));
    auto index = edges[0].size() - 1;
    hyperedge_ref new_edge{(int) index};
    edges[new_edge.level][new_edge.id].neighbours = neighbours;
    
    // push gpses
    edges_gpses[new_edge].push_back(a);
    edges_gpses[new_edge].push_back(b);

    // change graphs size for rendering purposes
    changeSize(a);
    changeSize(b);

    if(neighbours.size() > 0){
        auto index_closest_neighbour = neighbours[0];
        
        // infrom neighbours
        for(auto n : neighbours){
            if(n < index){
                if (edges[0][n].weight < edges[0][index_closest_neighbour].weight) index_closest_neighbour = n;
                edges[0][n].neighbours.push_back(index);
            }
        }

       
        hyperedge_ref parent{1,edges[new_edge.level][index_closest_neighbour].parent};
        
        if(parent.id != -1){
            edges[new_edge.level][new_edge.id].parent = parent.id;
            edges[parent.level][parent.id].children.push_back(new_edge.id);
            edges[parent.level][parent.id].weight += weight;
        }


        // if there are more layers than only the basic one create connections between parents
        while(parent.id != -1){
            // if there is no change in neighbour relationships -> break
            if(!updatedNeighbourRelations(parent)){
                break;
            }
            parent = hyperedge_ref(parent.level + 1, edges[parent.level][parent.id].parent);
        }
    }
}

void Graph2::addEdge(int index){
    if(removed_edges.count(index)){
        // refresh neighbours
        for(auto neighb : edges[0][index].neighbours){
            edges[0][neighb].neighbours.push_back(index);
        }

        hyperedge_ref parent{1,edges[0][index].parent};
        
        if(parent.id != -1){
            edges[0][index].parent = parent.id;
            edges[parent.level][parent.id].children.push_back(index);
            edges[parent.level][parent.id].weight += edges[0][index].weight;
        }

        // if there are more layers than only the basic one create connections between parents
        while(parent.id != -1){
            // if there is no change in neighbour relationships -> break
            if(!updatedNeighbourRelations(parent)){
                break;
            }
            parent = hyperedge_ref(parent.level + 1, edges[parent.level][parent.id].parent);
        }

        removed_edges.erase(index);
    }
}

void Graph2::removeEdge(int index, float curr_time, bool inform_upper_layer){
    removed_edges.insert(index);
    hyperedge_ref hr{index};
    
    problemOnReservedEdge(hr, curr_time);

    // remove connections between edges on the same level
    for(auto n : edges[hr.level][hr.id].neighbours){
        auto p = std::find(edges[hr.level][n].neighbours.begin(),edges[hr.level][n].neighbours.end(),hr.id);
        edges[hr.level][n].neighbours.erase(p);
        printf("e%i erased ref to %i",index,n);
    }

    hyperedge_ref parent{hr.level + 1, edges[hr.level][hr.id].parent};

    // if the parent exists, remove hr from its children
    if(parent.id != -1 && edges.size() > 1){
        auto p = std::find(edges[parent.level][parent.id].children.begin(),edges[parent.level][parent.id].children.end(),hr.id);
        if(edges[parent.level][parent.id].children.erase(p) != edges[parent.level][parent.id].children.end()){
            printf("  parent %i#%i erased ref to child %i\n", parent.id,parent.level,index);
        }
    }    
    
    bool update_par = updatedNeighbourRelations(parent);
    bool is_con = isConnected(parent);

    while(parent.id != -1 && parent.level < edges.size() && ( update_par || !is_con)){
        printf("splitting parent %i#%i\n",parent.id,parent.level);
        splitComponent(parent, curr_time, inform_upper_layer);
        parent = hyperedge_ref{parent.level + 1, edges[parent.level][parent.id].parent};
        update_par = updatedNeighbourRelations(parent);
        is_con = isConnected(parent);
    }

    printf("DONE SPLITTING upd: %i !is_con: %i\n",(int)update_par,(int)!is_con);
}

bool Graph2::isConnected(const hyperedge_ref& he){
    std::map<int,bool> visited;
    std::set<int> edges_used;

    int start = -1;
    for(auto e : edges[he.level][he.id].children){
        edges_used.insert(e);
        visited[e] = false;
        start = e;
    }  

    if(start != -1) {

        // dfs in children level
        dfs(edges_used, visited, he.level - 1, start);
    
        return std::all_of(visited.begin(),visited.end(),[](std::pair<int,bool> i){return i.second;});
        // return true;
    }
    return false;
}

void Graph2::splitComponent(const hyperedge_ref& he, float curr_time, bool inform_upper_layer){
    if(inform_upper_layer){
        problemOnReservedEdge(he,curr_time);
    }
    
    std::set<int> children;

    for(auto c : edges[he.level][he.id].children){
        children.insert(c);
    }    
    
    int i = 0;
    for(auto n : edges[he.level][he.id].neighbours){
        auto& nnrs = edges[he.level][n].neighbours;
        auto p = std::find(nnrs.begin(),nnrs.end(),he.id);
        if(p != nnrs.end()){
            nnrs.erase(p);
            printf("     %i did have ref to %i#%i\n", n, he.id, he.level);
            i++;
        }
        else{
            printf("     %i did NOT have ref to %i#%i\n", n, he.id, he.level);
        }
    }  

    printf("  deleted %i refs from %lu neighbs for parent %i#%i\n", i, edges[he.level][he.id].neighbours.size(),he.id,he.level);

    edges[he.level][he.id].children.clear();
    edges[he.level][he.id].neighbours.clear();
    edges[he.level][he.id].weight = 0;
    createComponent(he, children);
    
    for(auto n : edges[he.level][he.id].neighbours){
        edges[he.level][n].neighbours.push_back(he.id);
    }
    
    hyperedge_ref parent{he.level + 1, edges[he.level][he.id].parent};

    while(children.size() > 0){
        hyperedge h;
        edges[he.level].push_back(h);
        hyperedge_ref new_comp{he.level, (int) edges[he.level].size() - 1};
        createComponent(new_comp, children);
    
        for(auto n : edges[new_comp.level][new_comp.id].neighbours){
            edges[he.level][n].neighbours.push_back(new_comp.id);
        }    
        
        // this is a child of the `parent` too
        if(parent.id != -1){
            edges[parent.level][parent.id].children.push_back(new_comp.id);
            edges[new_comp.level][new_comp.id].parent = parent.id;
        }
    } 
}

void Graph2::createComponent(const hyperedge_ref& he, std::set<int>& possible_children){
    std::map<int,bool> child_visited;

    int start_index = -1;
    for(auto c : possible_children){
        child_visited[c] = false;
        start_index = c;
    }

    if(start_index != -1){
        // find all connected hyperedges
        dfs(possible_children, child_visited, he.level - 1, start_index);

        std::set<int> hes_neighbours;
        for(auto c : possible_children){
            if(child_visited[c]){
                // insert children
                edges[he.level][he.id].children.push_back(c);
                edges[he.level][he.id].weight += edges[he.level - 1][c].weight;

                // get possible neighbouring parents
                for(auto n : edges[he.level - 1][c].neighbours){
                    auto p = edges[he.level - 1][n].parent;
                    // insert only parents that are not `he`
                    if(p != he.id){
                        hes_neighbours.insert(p);
                    }
                }   
            }
        }

        // `he` has new neighbours and neighbours have new neighb `he.id`
        for(auto n : hes_neighbours){
            edges[he.level][he.id].neighbours.push_back(n);
        }

        for(auto& child_vis : child_visited){
            // delete children that are taken
            if(child_vis.second == true){
                possible_children.erase(child_vis.first);
            }
        }
    }
}

void Graph2::dfs(std::set<int>& use_these, std::map<int,bool>& visited, int level, int index){
    visited[index] = true; 
    
    for(auto neighbour : edges[level][index].neighbours){
        if (use_these.count(neighbour)){
            if (!visited.count(neighbour) || visited[neighbour] == false) 
                dfs(use_these, visited, level, neighbour);
        }
    }
}

// for hyperedge `parent` updates neighbours, i.e. deletes or adds them if connection changed
bool Graph2::updatedNeighbourRelations(const hyperedge_ref& parent){
    std::set<int> neighbours;
    
    auto parent_lvl = parent.level;
    auto child_lvl = parent_lvl - 1;

    // get all possible neighbours to `parent` by scanning connections between `parent`s children
    for(auto child : edges[parent_lvl][parent.id].children){
        for(auto childs_neighb : edges[child_lvl][child].neighbours){
            auto neighbs_par = edges[child_lvl][childs_neighb].parent;
            if(neighbs_par != parent.id && parent.id != -1){
                neighbours.insert(neighbs_par);
            }
        }
    }

    

    // if there is a neighbour that is missing or is newly added
    if(neighbours.size() != edges[parent.level][parent.id].neighbours.size()){
        for(auto neighb : edges[parent.level][parent.id].neighbours){
            if(!neighbours.count(neighb)){
                // erase neighbours that are not connected any more
                auto p = std::find(edges[parent.level][neighb].neighbours.begin(),edges[parent.level][neighb].neighbours.end(), parent.id);
                edges[parent.level][neighb].neighbours.erase(p);
            }
            else{
                // if they are connected, forget them
                neighbours.erase(neighb);
            }
        }

        // `neighbours` have only the new neighbours left
        for(auto new_neighb : neighbours){
            // let them know they are neighbours now
            edges[parent.level][new_neighb].neighbours.push_back(parent.id);
            edges[parent.level][parent.id].neighbours.push_back(new_neighb);
        }

        return true;
    }
    return false;
}
