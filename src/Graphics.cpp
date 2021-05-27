// #ifdef USE_GRAPHICS
#include "Graphics.hpp"

Graphics::Graphics(gps max_size, gps min_size) : min_size(min_size) {
    texture.create((unsigned int) std::ceil(max_size.x - min_size.x + offset.x), (unsigned int) std::ceil(max_size.y - min_size.y + offset.y));
}

// --- RENDERING ---------

void Graphics::render(Graph2& g, int lvl, bool print, const std::string& prefix){
    tex_name = g.name;
    auto nr_used_colors = colors.size();
    
    // draw edges of layer 0 with the same color if `lvl` was not specified
    if(lvl == -1) {
        lvl = 0;
        nr_used_colors = 1;
    }

    for (int i = 0; i < g[lvl].size(); i++) {
        if(lvl != 0 || !g.removed_edges.count(i)){
            hyperedge_ref h{lvl, i};
            renderEdge(g, h, colors[i % nr_used_colors]);            
        }
	}

    if(print) drawImage(prefix);
}

void Graphics::renderRoute(const std::vector<hyperedge_ref>& route, Graph2& g, bool ideal){
    // sf::Color route_clr = sf::Color::Blue; //
    auto route_clr = colors.at(1); // violet  // sf::Color(255,255,150,255); // cream yellow
    auto ideal_route_clr = sf::Color::Red;

    for (auto& e : route){
        if(ideal) renderEdge(g, e, ideal_route_clr);
        else renderEdge(g, e, route_clr);
    }
}

void Graphics::renderEdge(Graph2& g, const hyperedge_ref& h, sf::Color color){
    if (h.level != 0){
        for(auto child : g[h].children){
            hyperedge_ref ch{h.level - 1, child};
            renderEdge(g, ch, color);
        }
        
        return;
    }

    auto& gpses = g.edges_gpses[h];
    
    sf::VertexArray shape{sf::LineStrip, gpses.size()};
    
    for(int i = 0; i < gpses.size(); i++){
        shape[i].position = sf::Vector2f(gpses[i].x - min_size.x + offset.x, gpses[i].y - min_size.y);
	    shape[i].color = color;
    }

    texture.draw(shape);
}

void Graphics::print(const std::string& prefix){
    drawImage(prefix);
}

void Graphics::drawImage(std::string prefix){
    auto t = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
    std::string s = "../img/";
    s.append(prefix);
    s.append(tex_name);
    s.append("_");
    s.append(t);
    s.append(".png");
    printf("saving file to %s\n", s.c_str());
    texture.getTexture().copyToImage().saveToFile(s);
    texture.setActive(false);
}

// #endif
