#include "Preprocessing.hpp"

bool Preprocessing::createGraph2(Graph2 & g, const std::string& filename){
	std::ifstream ifs;

    ifs.open(filename);

    if(ifs.is_open()){

        std::string line;
        int edge_id;
		std::istringstream iss;

		std::getline(ifs,g.name);

        while(std::getline(ifs,line)){
            iss.clear();
			iss.str(line);

			iss >> edge_id;

			hyperedge h;
			iss >> h.weight;

			hyperedge_ref hr{(int) g.edges[0].size()};
			
			gps gp;
			iss >> gp.x;
			iss >> gp.y;	
			g.changeSize(gp);

			gps gp2;	
			iss >> gp2.x;
			iss >> gp2.y;
			g.changeSize(gp2);

			g.edges_gpses[hr].push_back(gp);
			g.edges_gpses[hr].push_back(gp2);

			int neigh_id;
			while(iss >> neigh_id){
				h.neighbours.push_back(neigh_id);
			}

			g.edges[0].push_back(h);

        }

		printf("> graph loaded from %s.\n", filename.c_str());
		printf("> edges: %lu, maxsize: %.3f, %.3f minsize: %.3f, %.3f\n",
			g.edges[0].size(), g.max_size.x, g.max_size.y, g.min_size.x, g.min_size.y);

		ifs.close();
		return true;
    }

	ifs.close();    
	return false;
    
}

void Preprocessing::createLayers2(Graph2 & g){
	auto components = loadLayers2(g);
	std::map<int,hyperedge_ref> id_to_href;
	
	if(components.size() > 0){
		for(auto i = 0; i < g[0].size(); i++){
			id_to_href[i] = hyperedge_ref{0,i};
		}
	}

	// printf("CREATING LAYERS/ ---------------\n\n");

	int curr_layer = 1;
	for(auto& layer : components){
		// parent position one layer up
		std::map<int,int> edge_parent;

		// printf("STARTING LAYER %i ----------\n", curr_layer);
		g.edges.push_back(std::vector<hyperedge>());
		int index_in_layer = 0;
		for(auto& comp : layer){
			// printf("  for id %i adding he %i#%i \n", comp.first, index_in_layer, curr_layer);
			id_to_href[comp.first] = hyperedge_ref{curr_layer,index_in_layer};
			hyperedge h;
			float children_weight = 0;
			// printf("   children: ");
			for (auto e : comp.second){
				h.children.push_back(id_to_href[e].id);
				g[id_to_href[e]].parent = index_in_layer;
				children_weight += g[id_to_href[e]].weight;
				edge_parent[id_to_href[e].id] = index_in_layer;
				// printf("%i(%i) ", id_to_href[e].id, e);
			}
			// printf(" (actual edge_id)\n");
			h.weight = children_weight;
			// printf("   its weight is %f\n", h.weight);
			g[curr_layer].push_back(h);
			// printf("   he added in layer %lu on index %lu\n",g.edges.size()-1, g[curr_layer].size()-1);
			index_in_layer++;
		}

		
		for (auto i = 0; i < g[curr_layer].size(); i++) {
			std::set<int> connected_hyperedges;
			hyperedge& h = g[curr_layer][i];
			// printf("   curr layer %i\n",curr_layer);
			// if children are connected to other components
			for (auto child : h.children){
				// printf("   checking child %i#%i of parent %i#%i\n",child,curr_layer -1, i, curr_layer);
				for(auto childs_neighbour : g[curr_layer-1][child].neighbours){
					// printf("    its neighb %i",childs_neighbour);
					auto other_hyperedge = edge_parent[childs_neighbour];
					// printf(" - parents neighbour found\n");
					if(i != other_hyperedge){
						connected_hyperedges.insert(other_hyperedge);
					}
				}
			}

			for (auto ch : connected_hyperedges){
				h.neighbours.push_back(ch);
			}
		}
		curr_layer++;
	}
	
	printf("> layers of graph %p are created. There are %lu of them.\n", &g, g.edges.size());
}

std::vector<std::map<int,std::set<int>>> Preprocessing::loadLayers2(Graph2 &g){
	std::ifstream ifs;
	std::string filename = "../data/" + g.name + ".layers";

	std::vector<std::map<int,std::set<int>>> layers;

    ifs.open(filename);

    if(ifs.is_open()){
        std::string line;
        int layer, edge_id, child_id;
		std::istringstream iss;

		while(std::getline(ifs,line)){
			iss.clear();
			iss.str(line);

			iss >> layer;
			while(layer >= layers.size()){
				layers.push_back(std::map<int,std::set<int>>());
			}

			iss >> edge_id;
			while(iss >> child_id){
				layers[layer][edge_id].insert(child_id);
			}
		}

		printf("> components loaded for %s.\n", filename.c_str());
    }
	else{
		printf("> file %s does not exist. Layers are going to be created now\n", filename.c_str());
		saveLayers2(g);
		return std::move(loadLayers2(g));
	}
    
    ifs.close();
	return std::move(layers);
}

void Preprocessing::saveLayers2(Graph2& g){
	auto comps = HCA(g,0);

	std::string filename = "../data/" + g.name + ".layers";
	std::ofstream ofs(filename);
    if(ofs.is_open()){

		int layer_id = 0;
		for(auto& layer : comps){
			for(auto& key_val : layer){
				ofs << layer_id << " " << key_val.first << " ";
				for(auto child : key_val.second){
					ofs << child << " ";
				}
				ofs << "\n";
			}
			layer_id++;
		}

    }
    ofs.close();
}

std::vector<std::map<int,std::set<int>>> Preprocessing::HCA(Graph2& g, int curr_lvl){
	// starting from layer 0. Each key in map is assigned its children
	std::vector<std::map<int,std::set<int>>> new_layers;

	#define debug 0
	#define debug_more 0

	std::map<int,std::set<int>> components;
	std::map<int,std::set<int>> comp_neighbs; // neighbors of components
	std::map<std::pair<int,int>,float> pair_score;
	std::map<int,std::pair<int,float>> comp_best_score;
	std::vector<float> comp_b_score; // best score
	std::vector<int> comp_b_neighb; // best neighbor
	std::map<int,int> comp_level;
	std::map<int,std::set<int>> comp_made_from;
	std::set<int> deleted;
	int next_component = 0;

	// put back to p
	auto s = -1.f; // best score sofar in an iteration

	// merges two HEs into one layer higher
	auto merge = [&](int c1, int c2){
		auto
		 &C1=components[c1],
		 &C2=components[c2],
		 &CM=components[next_component];

		auto
		 C1_lvl = comp_level[c1],
		 C2_lvl = comp_level[c2];

		auto CM_lvl = std::min(C1_lvl, C2_lvl) + 1;
		
		std::move(C1.begin(), C1.end(), std::inserter(CM, CM.begin()));
		std::move(C2.begin(), C2.end(), std::inserter(CM, CM.begin()));
		
		auto
		 &N1=comp_neighbs[c1],
		 &N2=comp_neighbs[c2],
		 &NM=comp_neighbs[next_component];

		comp_b_score.push_back(-1);
		comp_b_neighb.push_back(-69);
		comp_level[next_component] = CM_lvl;
		comp_made_from[next_component].insert(c1);
		comp_made_from[next_component].insert(c2);

		std::move(N1.begin(), N1.end(), std::inserter(NM, NM.begin()));
		std::move(N2.begin(), N2.end(), std::inserter(NM, NM.begin()));


		++next_component;
		
		NM.erase(c1);
		NM.erase(c2);
	};

	// computes the score of two HEs
	auto score = [&](int c1, int c2)->float{
		float sc = 0;
		auto
		 &C1=components[c1],
		 &C2=components[c2];

		auto 
		 C1_lvl = comp_level[c1],
		 C2_lvl = comp_level[c2];

		auto is_in=[&](int cid) {
			return C1.count(cid) || C2.count(cid);
		};

		using sp = std::tuple<float, int,int>;
		std::priority_queue<sp, std::vector<sp>, std::greater<sp>> pq;


		auto start_edge = [&](int e) {
			for(auto n : g[curr_lvl][e].neighbours)
				if(!is_in(n)) {
					// each edge needs only one border. Hence the break
					pq.push(sp{0,n,e});
					break;
				}
		};

		for(auto e : components[c1]) start_edge(e);
		for(auto e : components[c2]) start_edge(e);

		std::set<std::pair<int,int>> been_through;
		std::set<int> been_through_edges;

		while (!pq.empty())
		{
			auto [curr_cost, start, end] = pq.top();
			pq.pop();

			auto p = std::pair(start,end);

			sc = curr_cost;

			// edge has already been scanned
			if(been_through_edges.count(end)) {
				continue;
			}
			if(been_through.count(p) || been_through.count(std::pair(end,start))) {
				continue;
			}

			been_through.insert(p);
			been_through_edges.insert(end);			
			
			// we have to use edge `end` to get to the next junction
			auto edge_weight = g[curr_lvl][end].weight;

			for (auto other_eid : g[curr_lvl][end].neighbours){
				// go inside component
				if(is_in(other_eid)){
					auto n_cost = curr_cost + edge_weight;
					pq.push(std::make_tuple(n_cost, end, other_eid));
				}
			}
		}

		float member_sum = 0;
		// don't know how to use accumulate on these two
		for(auto m1 : C1){
			member_sum += g[curr_lvl][m1].weight;
		}
		for(auto m2 : C2){
			member_sum += g[curr_lvl][m2].weight;
		}

		float additional = 1000 / (comp_level[c1] + 1) + (1000 / (comp_level[c2] + 1));

		return sc / (float) std::pow(member_sum, hca_k )/**/ + additional;
	};

	// moves HE up a layer
	auto move_up = [&](int cid){
		#ifdef debug_more
		printf("moving from cid: %i to comp %i\n", cid, next_component);
		#endif
		auto &NC = components[next_component];
		comp_level[next_component] = comp_level[cid] + 1;
		std::move(components[cid].begin(), components[cid].end(), std::inserter(NC,NC.begin()));
		std::move(comp_neighbs[cid].begin(), comp_neighbs[cid].end(), std::inserter(comp_neighbs[next_component], comp_neighbs[next_component].begin()));
		comp_b_score.push_back(-1);
		comp_b_neighb.push_back(-69);
		comp_made_from[next_component].insert(cid);
		++next_component;
	};

	// sets best scores and neighbors for each HE
	auto compute_comps_score = [&](int cid){
		for(auto neighb : comp_neighbs[cid]){
			auto sc = score(cid,neighb);
			if(comp_b_neighb[cid] != -1 && comp_b_score[cid] < sc){
				comp_b_neighb[cid] = neighb;
				comp_b_score[cid] = sc;
			}
		}
	};

	// initialize components for the base layer
	for (; next_component < g[curr_lvl].size(); next_component++) {
		components[next_component] = std::set<int>{next_component};
		comp_best_score[next_component] = std::pair{-1,-1};
		comp_b_score.push_back(-1);
		comp_b_neighb.push_back(-69);
		comp_level[next_component] = 0;
		for(auto n : g[curr_lvl][next_component].neighbours){
			comp_neighbs[next_component].insert(n);
		}
	}

	for(auto& c1 : components){
		compute_comps_score(c1.first);
	}
	
	#if debug || debug_more
	printf("init done, comps.size():%lu\n", components.size());
	#endif

	int i = 0;
	int last_level = 0;
	int this_iter = components.size(), // #components in this iteration
		last_iter = -1; // #components the previous iteration
	// while(this_iter != last_iter){

	bool new_layer_done = false;
	while(components.size() > 80 || !new_layer_done){
		new_layer_done = false;

		auto c1 = -1, c2 = -1;

		for(auto index = 0; index < comp_b_score.size(); index++){
			if(comp_b_score[index] > s && comp_b_neighb[index] != -1){
				c1 = index;
				c2 = comp_b_neighb[index];
				s = comp_b_score[index];
			}
		}

		if(s == -1){
			#if debug && debug_more
			printf("s == -1 and deleted.size():%lu, comps.size():%lu, pair_score.size():%lu\n", deleted.size(),components.size(), pair_score.size());
			#endif
			return std::move(new_layers);
		}
		else{
			#if debug && debug_more
			printf("%06i(%03i) %06i(%03i) sc:%09.4f comp.size():%lu\n",c1,comp_level[c1],c2,comp_level[c2],s, components.size());
			#endif
		}

		if(comp_level[c1] != comp_level[c2]){
			std::set<int> moved_up_a_layer;
			auto mn = std::min(comp_level[c1], comp_level[c2]);

			std::set<int> del_lvl_of_these;
			std::set<int> comp_best_score_of_these;
			for(auto& c : comp_level){
				if(c.second == mn){
					move_up(c.first);
					// DONE create new component
					// DONE level is mn + 1
					// DONE its children are same as c.first
					// DONE has the same neighbs as c.first
					// DONE has no best score
					// DONE has no best neighb
					// DONE is made from c.first
					// DONE inform new neighbs
					for(auto n : comp_neighbs[next_component-1]){
						comp_neighbs[n].erase(c.first);
						comp_neighbs[n].insert(next_component-1);
						comp_b_neighb[n] = -69;
						comp_b_score[n] = -69;
						comp_best_score_of_these.insert(n);
					}

					comp_best_score_of_these.insert(next_component-1);

					// DONE delete c.first compoonents
					components.erase(c.first);
					// DONE set best neighb -1
					comp_b_neighb[c.first] = -1;
					deleted.insert(c.first);
					del_lvl_of_these.insert(c.first);
					// delete component_level
				}
			}

			for(auto c : comp_best_score_of_these){
				compute_comps_score(c);
			}

			for(auto d : del_lvl_of_these){
				comp_level.erase(d);
			}

			new_layers.push_back(std::map<int,std::set<int>>());
			auto& NL = new_layers.back();
			for(auto& c : components){
				NL[c.first] = comp_made_from[c.first];
			}
				
			// DEBUG
			std::map<int/*level*/, int/*count*/> level_count;
			for(auto c : components){
				if(!level_count.count(comp_level[c.first])){
					level_count[comp_level[c.first]] = 1;
				}
				else{
					level_count[comp_level[c.first]] += 1;
				}
			}

			#if debug || debug_more
			printf("moving rest from %i to %i: ----------------------------\n", mn, mn + 1);
			for(auto lc : level_count){
				printf("level %i count %i\n", lc.first, lc.second);
			}
			printf("-------------------------------------------------------\n");
			#endif
			s = -1;

			last_level = mn + 1;
			new_layer_done = true;
			continue;
		}

		if(comp_level[c1] != last_level){
			
			std::set<int> del_lvl_of_these;

			#if debug
			printf("new level started --- %i", comp_level[c1]);
			#endif
			for(auto& c : components){
				if(comp_level[c.first] != comp_level[c1]){
					
					#if debug 
					printf("comp %i is on diff lvl(%i) than %i (%i)\n",c.first, comp_level[c.first], c1, comp_level[c1]);
					#endif
					
					
					move_up(c.first);
					del_lvl_of_these.insert(c.first);
					
				}
			}

			for(auto d : del_lvl_of_these){
				comp_level.erase(d);
				components.erase(d);
				comp_b_neighb[d] = -1;
				deleted.insert(d);
			}

			new_layers.push_back(std::map<int,std::set<int>>());
			auto& NL = new_layers.back();
			for(auto& c : components){
				NL[c.first] = comp_made_from[c.first];
			}

			#if debug
			printf(" comp size: %lu\n", components.size());
			#endif

			last_level = comp_level[c1];
			new_layer_done = true;
			s = -1;
			continue;
		}
		
		merge(c1, c2);
		
		#if debug || debug_more
		printf("merge done, comps.size():%lu\n", components.size());
		#endif

		components.erase(c1);
		components.erase(c2);
		
		#if debug && debug_more
		printf("comp erase done %i %i, comps.size():%lu\n",c1, c2, components.size());
		#endif
		
		comp_b_neighb[c1] = -1;
		comp_b_neighb[c2] = -1;
		
		#if debug && debug_more
		printf("comp neighb <- -1 done, comps.size():%lu\n", components.size());
		#endif
		
		comp_level.erase(c1);
		comp_level.erase(c2);
		
		deleted.insert(c1);
		deleted.insert(c2);

		// zmeni se to jen pro sousedy nove komponenty
		auto new_comp_id = next_component - 1;
		for(auto n : comp_neighbs[new_comp_id]){
			comp_neighbs[n].erase(c1);
			comp_neighbs[n].erase(c2);	
			comp_neighbs[n].insert(new_comp_id);
			comp_b_neighb[n] = -69;
			comp_b_score[n] = -69;
			compute_comps_score(n);
		}
		
		compute_comps_score(new_comp_id);
		
		// make sure new pair is selected
		s = -1;

		#if debug || debug_more
		if(i++ % 1000 == 999){
			printf("iter %i, comp.size(): %lu\n", i++, components.size());
		}
		#endif
		
		last_iter = this_iter;
		this_iter = components.size();
	}

	#if debug || debug_more
	printf("RETURNING\n");
	#endif

	return std::move(new_layers);
}