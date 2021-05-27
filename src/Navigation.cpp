#include "Navigation.hpp"

// --- IDEAL ROUTES ---------

route& Navigation::idealRoute(int from, int to){
	auto p = std::make_pair(from, to);
	if (ideal_routes.count(p)) return ideal_routes[p];
	else {
		ideal_routes[p] = computeIdealRoute(from, to);
		return ideal_routes[p];
	}
}

void Navigation::prepIdealRoutes(const std::vector<std::pair<int, int>>& rts){
	int i = 0;
	for (auto&& r : rts) {
		ideal_routes[r] = computeIdealRoute(r.first, r.second);
		i++;
	}
	printf("%i ROUTES PREPPED rts.size(): %lu\n",i,rts.size());
}

route Navigation::computeIdealRoute(int from, int to) {
	long long ns = 0;
	// return std::move(dijkstraEdged(from,to,ns));
	return std::move(aStarEdged(from,to,ns,-1,false,false));
}

// --- PATH-FINDING ---------

route Navigation::findRoute(int from, int to, long long int& ns, float when, NavigationType2 nt, std::set<hyperedge_ref> been_through){
	if(nt == NavigationType2::UNKNOWN) nt = nav_type;

	switch (nt)
	{
	case NavigationType2::DIJKSTRA_EDGED:
		return dijkstraEdged(from,to,ns,when);
	case NavigationType2::ASTAR_EDGED:
		return aStarEdged(from,to,ns,when,true,real_time_system);
	case NavigationType2::DIJKSTRA_WPL:
		return dijkstraWPL(from,to,ns,when,been_through,real_time_system);
	default:
		return dijkstraEdged(from,to,ns,when);
	}
}

// --- PATH-FINDING ALGORITHMS ---------

route Navigation::dijkstraEdged(int from, int to, long long int& ns, float when){
	return aStarEdged(from, to, ns, when, false, real_time_system);
}

route Navigation::aStarEdged(int from, int to, long long int& ns, float when, bool uses_potential, bool real_time){

	using sortierung_pair = std::tuple<float, float, hyperedge_ref, hyperedge_ref>;
	// tuple : current cost; current potential; predecessor; current edge
	std::priority_queue<sortierung_pair, std::vector<sortierung_pair>, std::greater<sortierung_pair>> pq;
	
	auto potential = (uses_potential) ?
					[](int a, int b)->float{
						return euclideanDistance(a,b);
					} :
					[](int a, int b)->float{
						return 0.f;
					};
	if(when == -1){
		real_time = false;
	}


	hyperedge_ref hr_from{from};
	hyperedge_ref hr_to{to};

	std::map<hyperedge_ref, hyperedge_ref> predecessor;
	
	pq.push(std::make_tuple(0, potential(to,from), hr_from, hr_from));
	ns++;
	g2.number_of_scans++;
	
	hyperedge_ref last;

	while (!pq.empty())
	{
		auto [curr_cost, curr_potential, pred, curr_e] = pq.top();
		pq.pop();

		// edge has already been scanned
		if(predecessor.count(curr_e)) continue;

		predecessor[curr_e] = pred;

		// target reached
		if (curr_e == hr_to) {
			last = curr_e;
			break;
		}
		
		for (auto&& other_eid : g2[curr_e].neighbours){

			hyperedge_ref other_e{curr_e.level,other_eid};
			auto n_potential = potential(to, other_eid);
			auto edge_weight = g2[other_e].weight;
			if(real_time){
				auto sd = expectedSlowDown(other_e, when + curr_cost);
				if(sd < 1){
					printf("sd LESS THAN 1 -- %f\n",sd);
				}
				edge_weight *= sd;
			}
			auto n_cost = curr_cost - curr_potential + edge_weight + n_potential;
			pq.push(std::make_tuple(n_cost, n_potential, curr_e, other_e));
			ns++; // beancounting
			g2.number_of_scans++;
		}
	}

	auto r = parseRoute(last, hr_from, predecessor);
	return std::move(r);
}

route Navigation::dijkstraWPL(int from, int to, long long int& ns, float when, std::set<hyperedge_ref>& been_through, bool real_time){
	using sortierung_pair = std::tuple<float, int, hyperedge_ref, hyperedge_ref>;
	// tuple : current cost; number of jumps in current level; predecessor; current edge
	std::priority_queue<sortierung_pair, std::vector<sortierung_pair>, std::greater<sortierung_pair>> pq;
	
	hyperedge_ref hr_from{from};
	std::set<hyperedge_ref> targets;

	when = (when == -1) ? 0 : when;

	// make target not only 'to' but all of its parents
	for(hyperedge_ref hr_to{to}; hr_to.id != -1; hr_to = hyperedge_ref(hr_to.level+1, g2[hr_to].parent))
		targets.insert(hr_to);

	std::map<hyperedge_ref, hyperedge_ref> predecessor;
	
	pq.push(std::make_tuple(0,0,hr_from,hr_from));
	ns++;
	g2.number_of_scans++;
	
	hyperedge_ref last;

	while (!pq.empty())
	{
		auto [curr_cost, curr_jumps, pred, curr_e] = pq.top();
		pq.pop();

		// printf("curr_e %i#%i\n", curr_e.id, curr_e.level);

		// edge has already been scanned
		if(predecessor.count(curr_e)) continue;
		// edge has already been used
		if(been_through.count(curr_e)) {
			continue;
		}

		predecessor[curr_e] = pred;

		// target or its parent reached
		if (targets.count(curr_e)) {
			/*std::cout << "from: " << from << std::endl;
			std::cout << "dist: " << curr_cost << std::endl;
			std::cout << "jumps: " << curr_jumps << std::endl;
			std::cout << "id: " << curr_e.id << std::endl;*/
			last = curr_e;
			break;
		}
		
		for (auto&& other_eid : g2[curr_e].neighbours){

			hyperedge_ref other_e{curr_e.level,other_eid};

			//level switch time! jump only if other_e has a different parent
			if((curr_jumps > dwpl_jumps + specify_more + dwpl_jumps * curr_e.level) && g2[curr_e].parent != g2[other_e].parent) {
				// printf("tried to jump up\n");
				other_e = hyperedge_ref(other_e.level+1, g2[other_e].parent);
				// printf("jump worked out\n");
			}

			// printf("   other_e %i#%i\n", other_e.id,other_e.level);

			ns++; //beancounting
			g2.number_of_scans++;
			float sd = 1;
			if(real_time){
				sd = expectedSlowDown(other_e, when + curr_cost, true);
			}
			auto n_cost = curr_cost + g2[other_e].weight * sd;
			pq.push(std::make_tuple(n_cost, curr_jumps+1, curr_e, other_e));
		}
	}

	auto r = parseRoute(last, hr_from, predecessor);
	return std::move(r);
}

// --- HELPER FUNCTIONS ---------

route Navigation::parseRoute(hyperedge_ref last, const hyperedge_ref& from, std::map<hyperedge_ref, hyperedge_ref>& predecessor){
	std::vector<hyperedge_ref> res;

	if(last.id == -1){
		return std::move(route());
	}

	while(last != from){
		res.push_back(last);
		//std::cout << "last id: " << last.id << std::endl;
		last = predecessor[last];
	}

	std::reverse(res.begin(), res.end());

	return std::move(res);
}

float Navigation::routeWeight(const route& rte){
	float f = 0;
	for(auto&& e : rte){
		f += g2[e].weight;
	}
	// float f = std::accumulate(rte.begin(),rte.end(),0);
	return f;
}

float Navigation::expectedEdgeWeight(const hyperedge_ref& e, float when){
	return g2[e].weight * expectedSlowDown(e, when);
}

// --- POTENTIALS FOR ALGORITHMS ---------

float Navigation::euclideanDistance(int a, int b,int lvl){
	auto ap = g2.avgGPSOf(a);
	auto bp = g2.avgGPSOf(b);
	return std::sqrt(std::pow((ap.x - bp.x), 2) + std::pow((ap.y - bp.y), 2));
}

std::map<int,float> Navigation::dijkstraDistances(int from){
	using sortierung_pair = std::tuple<float, int, int>;
	// tuple : current cost; predecessor; current edge
	std::priority_queue<sortierung_pair, std::vector<sortierung_pair>, std::greater<sortierung_pair>> pq;
	pq.push(std::make_tuple(0, from, from));

	std::map<int, int> predecessor;
	std::map<int, float> distances;

	while (!pq.empty())
	{
		auto [curr_cost, pred, curr_e] = pq.top();
		pq.pop();

		distances[curr_e] = curr_cost;

		// edge has already been scanned
		if(predecessor.count(curr_e)) continue;
		predecessor[curr_e]=pred;
		
		for (auto&& other_eid : g2[0][curr_e].neighbours){
			auto n_cost = curr_cost + g2[0][other_eid].weight;
			pq.push(std::make_tuple(n_cost, curr_e, other_eid));
		}
	}

	return std::move(distances);
}

// --- EXPERIMENTAL / REAL-TIME INFORMAITON ---------

float Navigation::currentEdgeWeight(const hyperedge_ref& e, float when){
	
	float new_weight = g2[e].weight;

	#if 1
	float nr_vehicles = g2.vehicles_on_edge[e].size() - 1; // vehicle asking is already on the edge. We want to know, how many other vehicles entered before this one
	nr_vehicles = (nr_vehicles < 0) ? 0 : nr_vehicles;

	new_weight *= trafficFlow(nr_vehicles, g2[e].weight);

	// CONGESTION
	if(g2.known_congestions.count(e)){
		auto cong = g2.known_congestions[e];
		// congestion happens when vehicle is on the edge.
		if(when < cong.first && when + new_weight < cong.second){
			// vehicle is there for the time it takes to travel the edge with the slowdown due to vehicle density
			// plus the time it takes to remove the congestiom
			new_weight += (cong.second - cong.first);
		}
		// vehicle enters the edge when the congestion is present. It waits until the congestion is removed and then
		// the route takes `edge_travel_time`
		else if(when > cong.first && when < cong.second){
			new_weight += (cong.second - when);
		}
	}
	#endif

	if(new_weight < g2[e].weight){
		printf("new weight < g2[e].weight \n");
	}

	return new_weight;
}


// returns the ratio how much longer the traveling through `e` would take
// ment to multiply the edge weight
float Navigation::expectedSlowDown(const hyperedge_ref& e, float when, bool multi_layer){
	// if there is a time_spent_in_t_jam on an edge, the time it would take to travel it
	// is the original time + the time to remove the time_spent_in_t_jam

	float slow_down = 0.f;

	#if 0
	float density = g2.reservationsOn2(e,when) / g2[e].weight;
	// float density = vehiclesInTime(e,when) / g2[e].weight;
	slow_down = 1 + density / vehicles_per_unit;
	
	#endif
		
	#if 1 
	auto vehicles_in_t = vehiclesInTime(e, when);

	auto ew = g2[e].weight;

	slow_down = trafficFlow(vehicles_in_t, ew);
	
	#endif

	// g2.max_exp_slowdown = std::max(slow_down, g2.max_exp_slowdown);
	// g2.exp_slo_dn_called++;
	// g2.exp_slo_when.push_back(when);

	return slow_down;
}

// ment to be multiplied with the edge weight
float Navigation::trafficFlow(float occupation, float length){
	float density = occupation / length;
	
	// old 
	// auto res = std::atan(density / 6 - 2) + 1;
	// return (res < 0) ? 1.f : res + 1;

	// new
	auto res = 1 + density / vehicles_per_unit;	
	return res;
}

// returns the expected number of vehicles present on `he` in time `when`
// takes to an account congestions on `he`
float Navigation::vehiclesInTime(const hyperedge_ref& he, float when, bool multi_layer){
	
	// the best way to go about this to be optimistic and say that the vehicles
	// that have reservation in the upper layer will not affect this edge so
	// much and will not slow down the traffic. But we will add the number of 
	// the reservations present in time `when` divided by the number of edges of ancestor
	
	float edge_weight = g2[he].weight;
	std::set<int> vehicles_present; // = 0;
	bool cong_present = false;
	std::vector<std::pair<int,float>> times_vehs_leave_edge;
	std::set<int> vehicles_scanned;
	
	std::pair<float,float> cong_from_to = std::make_pair(-1.f, -1.f);

	std::vector<hyperedge_ref> ancestors;
	hyperedge_ref parent{he.level + 1, };
	while(parent.id != -1){
		ancestors.push_back(parent);
		parent = hyperedge_ref(parent.level + 1, g2.edges[parent.level][parent.id].parent);
	}
 
	if(g2.known_congestions.count(he)){
		cong_from_to = g2.known_congestions[he];
		cong_present = true;
	}

	for(auto& res : g2.rsrv_times_vehs[he]){
		// current time is `res.first`
		
		g2.rsrv_scan++;
		
		if(res.first > when){
			// this part is for multilayer purposes
			// there will not be 
			
			float vehicles_in_ancs = 0.f;	

			if(multi_layer){
			
				for(auto& anc : ancestors){
					float anc_children_count = g2[anc].children.size();
					float anc_weight = g2[anc].weight;
					float children_present = 0;

					auto a = g2.rsrv_children_times_vehs[anc].lower_bound(when - anc_weight);

					for (; a != g2.rsrv_children_times_vehs[anc].end(); ++a) {
						if (a->first < when && !vehicles_scanned.count(a->second)) { 
							children_present += 1;
						}
						else{
							break;
						}
						g2.rsrv_scan++;
					}
					vehicles_in_ancs += children_present / anc_children_count;
				}
			}
			
			return vehicles_present.size() + vehicles_in_ancs;
		}

		while(!times_vehs_leave_edge.empty() && times_vehs_leave_edge[0].second < res.first){
			auto r = times_vehs_leave_edge.begin();
			vehicles_present.erase(r->first);
			times_vehs_leave_edge.erase(r);
		}

		// CONGESTIONS
		if(cong_present){
			float edge_travel_time = edge_weight * trafficFlow(vehicles_present.size(),edge_weight);
			
			// vehicle will be out by the time congestion starts
			if(res.first + edge_travel_time < cong_from_to.first){
				times_vehs_leave_edge.push_back(std::make_pair(res.second, res.first + edge_travel_time));
			}
			// congestion happens when vehicle is on the edge.
			else if(res.first < cong_from_to.first && res.first + edge_travel_time < cong_from_to.second){
				// vehicle is there for the time it takes to travel the edge with the slowdown due to vehicle density
				// plus the time it takes to remove the congestiom
				times_vehs_leave_edge.push_back(std::make_pair(res.second, res.first + edge_travel_time + (cong_from_to.second - cong_from_to.first)));
			}
			// vehicle enters the edge when the congestion is present. It waits until the congestion is removed and then
			// the route takes `edge_travel_time`
			else if(res.first > cong_from_to.first && res.first < cong_from_to.second){
				times_vehs_leave_edge.push_back(std::make_pair(res.second, cong_from_to.second + edge_travel_time));
			}
			else{
				times_vehs_leave_edge.push_back(std::make_pair(res.second, res.first + edge_travel_time));	
			}
		}
		else{
			times_vehs_leave_edge.push_back(std::make_pair(res.second, res.first + edge_weight * trafficFlow(vehicles_present.size(),edge_weight)));
		}

		// arrival_time = res.first
		vehicles_present.insert(res.second);
		vehicles_scanned.insert(res.second);
	}


	float vehicles_in_ancs = 0.f;
	if(multi_layer){ 

		for(auto& anc : ancestors){
			float anc_children_count = g2[anc].children.size();
			float anc_weight = g2[anc].weight;
			float children_present = 0;

			auto a = g2.rsrv_children_times_vehs[anc].lower_bound(when - anc_weight);

			for (; a != g2.rsrv_children_times_vehs[anc].end(); ++a) {
				if (a->first < when && !vehicles_scanned.count(a->second)) { 
					children_present += 1;
				}
				else{
					break;
				}
				g2.rsrv_scan++;
			}
			vehicles_in_ancs += children_present / anc_children_count;
			}
		}
	return vehicles_present.size() + vehicles_in_ancs;
}

// --- STATS ---------

// returns the average query time from in microseconds
float Navigation::avgQueryTime() {
	auto sum = std::accumulate(query_times.begin(), query_times.end(), (int64_t) 0);
	return (float) sum / query_times.size();
}


