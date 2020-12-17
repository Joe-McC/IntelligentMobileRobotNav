#pragma once

#include <SFML\Graphics.hpp>
#include <SFML\Window.hpp>
#include <SFML\System.hpp>
#include <SFML\Main.hpp>

#include "Aria.h"

#include <cmath>
#include <iostream>
#include <string>
#include <array>
#include <stack>
#include <random>
#include <functional>
#include <cstdlib>
#include <ostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <vector>


class sfmlSim_map : public sf::Drawable, public sf::Transformable
{
public:
	//Constructors
	sfmlSim_map(std::string map_name);
	~sfmlSim_map();

	// Functions
	void update();
	

	// Variables
	std::vector< sf::VertexArray > inputMap = std::vector<sf::VertexArray >(200);
	sf::Color sfml_mapLineColor = sf::Color::Magenta;
	sf::Vector2f robotXYStartingPosition;

	int minX;
	int minY;
	int maxX;
	int maxY;


protected: 
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
	sf::VertexArray m_vertices;
	sf::Texture m_texture;

	
};

