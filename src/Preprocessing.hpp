#ifndef BP_PREPROCESSING_HPP
#define BP_PREPROCESSING_HPP

#include <algorithm>
#include <fstream>
#include <math.h>
#include <sstream>
#include <string>

#include "Navigation.hpp"

class Navigation;
class Preprocessing;

class Preprocessing {
private:
    static constexpr float hca_k = 2.0f; // clustering parameter for HCA()

    // computes the layeres structure for graph 'g'
    static std::vector<std::map<int,std::set<int>>> HCA(Graph2& g, int curr_lvl = -1);
    
public:
    // loads graph from file 'filename' into 'g'
    static bool createGraph2(Graph2 & g, const std::string& filename);
    // creates layers for graph 'g'. If the layers exist in data/*.layers, loads them
    static void createLayers2(Graph2 & g);
    // loads layers for 'g' from appropriate filename 
    static std::vector<std::map<int,std::set<int>>> loadLayers2(Graph2 & g);
    // saves layers for 'g' for later use
    static void saveLayers2(Graph2& g);
};


#endif