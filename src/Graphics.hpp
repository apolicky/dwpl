#ifndef BP_GRAPHICS_HPP
#define BP_GRAPHICS_HPP

#define USE_GRAPHICS 1 // should be commented out, it is specified in Makefile. Now for testing it is set to 1

#ifdef USE_GRAPHICS

#include <cmath>
#include <chrono>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include "Graph2.hpp"
#include "Vehicle.hpp"

class Graphics {
private:
    sf::RenderTexture texture;

    // texture name
    std::string tex_name = "";
    
    gps min_size;
    gps offset{2,2};

    std::vector<sf::Color> colors{
        sf::Color(230,230,230,255)/*light grey*/,
        sf::Color(100,0,100,255)/*violet*/,  
        sf::Color::Yellow,
        sf::Color(0,255,200,255)/*turqoise*/, 
        sf::Color(59,200,50,255)/*green*/, 
        sf::Color::Red, 
        sf::Color::Blue,
        sf::Color(128,128,128,255)/*grey*/, 
        sf::Color(205,133,63,255)/*brown*/};
	
    // draws the texture to a file in ../img
    void drawImage(std::string prefix = "");

public:
    Graphics(gps max_size, gps min_size = gps{0,0});
    ~Graphics() {};

    // draws 'g' in a single color (level == -1) or colored hyperedges (level > 0)
    // might also print the graph if print == true to file with prefix 'prefix'
    void render(Graph2& g, int level = -1, bool print = false, const std::string& prefix = "");
    // renders route 'route', 'g' is needed to provide geo coords of hyperedges of the route
    void renderRoute(const std::vector<hyperedge_ref>& route, Graph2& g, bool ideal = false);
    // renders a single hyperedge of 'g' with color 'c'
    void renderEdge(Graph2& g, const hyperedge_ref& e, sf::Color c = sf::Color::Magenta);
    // prints the texture to file in ../img starting with prefix 'prefix'
    void print(const std::string& prefix = "");
};

#endif
#endif
