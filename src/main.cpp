
#include "Simulation.hpp"

#define USE_GRAPHICS 1
#define PREPROCESSING_EXPERIMENT 0
#define DWPL_EXPERIMENT 0
#define ROUTING_EFF_EXPERIMENT 0

#ifdef USE_GRAPHICS
#include "Graphics.hpp"
void graphicsCommand(std::vector<std::string>& words);
#endif

// functions 
void printConfiguration(Simulation& s);
void printHelp();
void runCommand(Simulation& s, std::string line);
bool getNavigationType(const std::string& nt, NavigationType2 & nav_type);
std::vector<std::string> splitLine(std::string& line);

// experiment functions
void preprocessingExperiment();
void routingEfficiencyExperiment(Simulation& s);
void dwplExperiment(Simulation& s);

// static vars initialization
std::map<std::pair<int, int>, route> Navigation::ideal_routes{};
std::vector<long int> Navigation::query_times{};
Graph2 Navigation::g2{};

// tuning parameters
NavigationType2 Navigation::nav_type{NavigationType2::DIJKSTRA_WPL};
bool Navigation::real_time_system = true;
unsigned int Navigation::dwpl_jumps = 4;

int main(int argc, char** argv) {
    Simulation s;
    
    std::string filename = "../data/sf2k.edgs";
    // std::string filename = "data/sf30k.edgs";
    // std::string filename = "data/sf220k.edgs";

    for(int i = 1; i < argc; i++){
        if(argv[i] == std::string("-f")){
            i++;
            filename = argv[i];
            printf("filename: %s\n", filename.c_str());
        }
    }

    if(Preprocessing::createGraph2(Navigation::g2, filename)){
        Preprocessing::createLayers2(Navigation::g2);

        #if PREPROCESSING_EXPERIMENT // preprocessing time and size test
        preprocessingExperiment();
        #endif

        #if ROUTING_EFF_EXPERIMENT // test routing efficiency
        routingEfficiencyExperiment(s);
        #endif

        #if 0
        s.r.reseed(123457);
        s.setNumberOfRoutes(1);
	    s.setNumberOfVehicles(30);
        Navigation::real_time_system = true;
        // Navigation::setNavigationType(NavigationType2::ASTAR_EDGED);
        // Navigation::setNavigationType(NavigationType2::DIJKSTRA_EDGED);
        auto start = std::chrono::steady_clock::now();
        printf("vehs: %u vehs_per_edge: %f real_time_system: %i\n", s.getNumberOfVehicles(), Navigation::vehicles_per_unit, (int)Navigation::real_time_system);
        printf("veh_id,from,to,dur,ideal_dur,ns,rerouted,edges_used,ideal_edges_used\n");
        s.run();
        auto end = std::chrono::steady_clock::now();
        printf("simulation took: %li\n", std::chrono::duration_cast<std::chrono::microseconds>(end-start).count());
        runCommand(s, "draw routes");
        printf("new rsrv scans: %lli\n", Navigation::g2.rsrv_scan);
        printf("g2.nodesscanned: %lli\n", Navigation::g2.number_of_scans);
        #endif

        

        #if DWPL_EXPERIMENT // dwpl parameter testing
        dwplExperiment(s);
        #endif

        std::string line = "";
        while (std::getline(std::cin, line)) {
            runCommand(s, line);
        }
    }
    else{
        printf("Graph from file %s could not be loaded.\n", filename.c_str());
    }

    return 0; 

}

void runCommand(Simulation & s, std::string line){
    auto words = splitLine(line);

    if(words[0] == "n"){
        if(words.size() > 1){
            NavigationType2 nt;
            if(getNavigationType(words[1],nt)) {
                printf("> Setting navigation type to %s\n", words[1].c_str());
                Navigation::setNavigationType(nt);
            }
            else {
                printf("? Unknown navigation type %s",words[1].c_str());
                printf("> Setting navigation type to %s\n", words[1].c_str());
                Navigation::setNavigationType(nt);
            }

        }
        else{
            printf("? Set the navigation type. See help\n");
            printf("? Run command with parameter: \"n [op]\", where options are:\n");
            printf("? d - Dijkstra\n> a - A*\n> dwpl - Dijkstra with Precision limit\n");
        }
    }
	else if (words[0] == "v") {
		if (words.size() > 1) {
			int c;
			if (c = std::stoi(words[1])) {
				printf("> Setting the number of vehicles to spawn to %s\n", words[1].c_str());
				s.setNumberOfVehicles((unsigned int) std::stoi(words[1]));
			}
            else{
                printf("? The type of the second argument must be 'int', %s is not this type.\n", words[1].c_str());
            }
		}
		else {
			printf("? Run command with parameter: \"v [nr]\", where \"nr\" is\n");
			printf("? the number of vehicles to run in the simulation.\n");
		}
	}
    else if(words[0] == "run"){
        std::string navtypes[] = {"Dijsktra", "A*","Dijkstra with Precision limit", "Unknown"};
        printf("> Starting a simulation where %u vehicles are navigated by %s on %u routes.\n",
			s.getNumberOfVehicles(),
            navtypes[(int) Navigation::getNavigationType()].c_str(),
            s.getNumberOfRoutes());
        auto start = std::chrono::steady_clock::now();
        printf("veh_id | from | to | dur | ideal_dur | scans | rerouted | edges_used | ideal_edges_used\n");
        s.run();
        auto end = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        printf("> Simulation finished in %li microseconds, scanning %lli hyperedges, scanning %lli reservations  \n", 
            ms,Navigation::g2.number_of_scans, Navigation::g2.rsrv_scan);
        printf(">    average query time: %f microseconds \n", Navigation::avgQueryTime());
    }
    else if(words[0] == "r"){
        if (words.size() > 1) {
			int c;
			if (c = std::stoi(words[1])) {
				printf("> Setting the number of routes to %s\n", words[1].c_str());
				s.setNumberOfRoutes(c);
            }
		}
		else {
			printf("? Run command with parameter: \"r [n]\", where \"n\" is\n");
			printf("? the number of routes in the simulation.\n");
		}
    }
    else if (words[0] == "conf") {
		printConfiguration(s);
	}
    else if(words[0] == "seed") {
        if (words.size() > 1) {
			int c;
			if (c = std::stoi(words[1])) {
				printf("> Setting new seed to %s\n", words[1].c_str());
				s.r.reseed(c);
            }
		}
		else {
			printf("? Run command with parameter: \"seed [n]\", where \"n\" is\n");
			printf("? the new seed for random number generator.\n");
		}
    }
    else if(words[0] == "reset"){
        s.reset();
        Navigation::g2.vehicles.clear();
    }
    else if(words[0] == "clear"){
        s.clear();
        Navigation::g2.vehicles.clear();
    }
    else if(words[0] == "help" || words[0] == "?"){
        printConfiguration(s);
        printHelp();
    }
    #ifdef USE_GRAPHICS
    else if(words[0] == "draw"){
        std::printf("calling gr command, words size: %lu\n", words.size());
        graphicsCommand(words);
    }
    #endif
    else{
        printf("? Unknown command %s.\n", line.c_str());
        printConfiguration(s);
        printHelp();
    }
}

/**
 * Tries to decide what navigation type to use based on users input 'nt'. 
 * If true is returned, the navigation type is set in variable 'nav_type'.
 */
bool getNavigationType(const std::string& nt, NavigationType2 & nav_type){
    if(nt == "d"){
        nav_type = NavigationType2::DIJKSTRA_EDGED;
        printf("> Navigation with Dijkstra's selected.\n");
        return true;
    }
    else if(nt == "a"){
        nav_type = NavigationType2::ASTAR_EDGED;
        printf("> Navigation with A* selected.\n");
        return true;
    }
    else if(nt == "dwpl"){
        nav_type = NavigationType2::DIJKSTRA_WPL;
        printf("> Navigation with Dijkstra's with precision limit selected.\n");
        return true;
    }
    else return false;
}

void printConfiguration(Simulation& s) {
    printf("> Current configuration:\n");
    printf(">    graph name: %s\n", Navigation::g2.name.c_str());
    printf(">    navigation type: %i (0 .. Dijkstra, 1 .. A*, 2 .. Dijkstra w/ prec. limit)\n", (int) Navigation::getNavigationType());
    printf(">    number of vehicles: %u\n", s.getNumberOfVehicles());
    printf(">    number of routes: %u(if 0 then each vehicle has a random route)\n", s.getNumberOfRoutes());
}

void printHelp() {
    printf("> COMMANDS: \n");
    printf(">   \"run\" - runs the simulation. Does not expect any arguments.\n");
    printf(">   \"conf\" - lists simulations configuration. Does not expect any arguments.\n");
    printf(">   \"n [type]\" - sets the type of navigation to \"type\". Choose one of the following\n");
    printf(">      \"d\" - Dijkstra, \"a\" - A*, \"dwpl\" - Dijkstra with Precision limit\n");
    printf(">   \"v [0-9]+\" - sets the number of vehicles to new value. Expects to recieve one non-negative integer.\n");
    printf(">   \"r [0-9]+\" - sets the number of routes. Vehicles will travel between\n");
    printf(">      [recieved number] pairs of locations. Expects to recieve one non-negative integer.\n");
    printf(">   \"draw [param]\" - if param is not set, draws the base layer (layer 0) of the graph\n");
    printf(">       \"param\" is an unsigned integer between 0 and #layers, draws that layer\n");
    printf(">       \"param\" === \"routes\", draws routes used in the simulation, if there are any.\n");
    printf(">   \"seed [0-9]+\" - sets new random seed\n");
    printf(">   \"help\" | \"?\" - print this help.\n");
    printf(">   \"reset\" - resets the random generator, deletes existing routes. Does not expect any arguments.\n");
    printf(">   \"clear\" - deletes existing routes. Does not expect any arguments.\n");
}

/**
 * Splits string 'line' to words.
 */
std::vector<std::string> splitLine(std::string& line){
    std::vector<std::string> vec;
    std::string word;
    std::istringstream iss(line);
    while(std::getline(iss,word,' ')){
        if(word != "") vec.push_back(word);
    }
    return vec;
}

#ifdef USE_GRAPHICS
void graphicsCommand(std::vector<std::string>& words){
    std::printf("in gr commands\n");
    for(auto && s : words) {
        std::printf("   %s\n", s.c_str());
    }
    if(words.size() > 1){
        Graphics gr(Navigation::g2.max_size, Navigation::g2.min_size);
        bool number = true;
            
        for(auto c : words[1]){
            number = number && std::isdigit(c);
        }         
        if(number){
            printf("drawing lvl %s\n", words[1].c_str());
            std::string lyr = "layer-";
            lyr += words[1] + "-";
            gr.render(Navigation::g2,std::stoi(words[1]),true, lyr);
        }
        else if(words[1] == "routes"){
            sf::Color start_clr = sf::Color::Yellow;
            sf::Color end_clr = sf::Color::Green;

            gr.render(Navigation::g2);
            for(auto& v : Navigation::g2.vehicles){
                gr.renderRoute(v.second.edgesUsed(), Navigation::g2);
                // #ifndef SF2K_EFFICIENCY
                gr.renderRoute(Navigation::idealRoute(v.second.from,v.second.to), Navigation::g2, true);
                // #endif
                // #ifdef SF2K_EFFICIENCY
                // gr.renderRoute(Navigation::g2.vehicles.begin()->second.edgesUsed(), Navigation::g2, true); // ideal by dwpl
                // #endif
                gr.renderEdge(Navigation::g2, hyperedge_ref(v.second.from), start_clr);
                gr.renderEdge(Navigation::g2, hyperedge_ref(v.second.to), end_clr);
            }
            gr.print("routes-");
        }
        else{
            gr.render(Navigation::g2,0, true);
        }
    }
    else{
        Graphics gr(Navigation::g2.max_size, Navigation::g2.min_size);
        gr.render(Navigation::g2,-1,true);
        printf("> You can also call \"draw [lvl]\", where \"lvl\" is level of graph.\n");
    }
    printf("> Drawing done.\n");
}
#endif

// --------- TESTING ----------------------------

void preprocessingExperiment() {
    long unsigned int graph_size = 0;
        Graph2& g = Navigation::g2;
        for(auto& l : g.edges){
            for(auto& h : l){
                graph_size += sizeof(h.weight);
                graph_size += sizeof(h.parent);
                graph_size += sizeof(int) * h.neighbours.size();
                graph_size += sizeof(int) * h.children.size();
            }
        }

        auto c_start = std::chrono::steady_clock::now();
        Preprocessing::createLayers2(Navigation::g2);
        auto c_end = std::chrono::steady_clock::now();
        

        long unsigned int graph_size2 = 0;
        for(auto& l : g.edges){
            for(auto& h : l){
                graph_size2 += sizeof(h.weight);
                graph_size2 += sizeof(h.parent);
                graph_size2 += sizeof(int) * h.neighbours.size();
                graph_size2 += sizeof(int) * h.children.size();
            }
        }
        
        std::printf("Size of graph is: %lu, size of layered graph is: %lu   [B]\n", graph_size, graph_size2);
        printf("creating layers took: %li   [microseconds]\n\n\n\n", std::chrono::duration_cast<std::chrono::microseconds>(c_end-c_start).count());
}

void routingEfficiencyExperiment(Simulation& s) {
    s.setNumberOfVehicles(30);
    s.routes.push_back(std::make_pair(3,2519));
   
    Navigation::real_time_system = false;
    // Navigation::setNavigationType(NavigationType2::ASTAR_EDGED);
    auto start = std::chrono::steady_clock::now();
    printf("vehs: %u vehs_per_edge: %f real_time_system: %i\n", s.getNumberOfVehicles(), Navigation::vehicles_per_unit, (int)Navigation::real_time_system);
    printf("veh_id,from,to,dur,ideal_dur,ns,rerouted,edges_used,ideal_edges_used\n");
    s.run();
    auto end = std::chrono::steady_clock::now();
    printf("simulation took: %li\n", std::chrono::duration_cast<std::chrono::microseconds>(end-start).count());
    runCommand(s, "draw routes");
}

void dwplExperiment(Simulation& s) {
    
    const unsigned int number_of_vehicles = 100;

    Navigation::real_time_system = true;
    s.setNumberOfVehicles(number_of_vehicles);

    printf("DwPL clustering parameter %s\n",".HCA_p2");
    printf("# vehs: %u | vehs_per_edge: %f | real_time_system: %i\n", s.getNumberOfVehicles(), Navigation::vehicles_per_unit, (int)Navigation::real_time_system);
    auto start = std::chrono::steady_clock::now();
    s.run();
    auto end = std::chrono::steady_clock::now();
    printf("simulation took: %li\n", std::chrono::duration_cast<std::chrono::microseconds>(end-start).count());
    printf("Edge scans: %lli | reservation scans: %lli\n", Navigation::g2.number_of_scans, Navigation::g2.rsrv_scan);
    
    // ---- jump constant testing ----

    Navigation::real_time_system = true;
    s.setNumberOfVehicles(number_of_vehicles);
    Navigation::dwpl_jumps = 10;

    printf("DwPL jump parameter %i\n",Navigation::dwpl_jumps);
    printf("# vehs: %u | vehs_per_edge: %f | real_time_system: %i\n", s.getNumberOfVehicles(), Navigation::vehicles_per_unit, (int)Navigation::real_time_system);
    start = std::chrono::steady_clock::now();
    s.run();
    end = std::chrono::steady_clock::now();
    printf("simulation took: %li\n", std::chrono::duration_cast<std::chrono::microseconds>(end-start).count());
    printf("Edge scans: %lli | reservation scans: %lli\n", Navigation::g2.number_of_scans, Navigation::g2.rsrv_scan);
}