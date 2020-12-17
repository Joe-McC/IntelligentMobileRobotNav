#pragma once

#include "globalRobotPositionData.h"

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




class sfmlSim_trail : public sf::Drawable, public sf::Transformable
{
public:
	sfmlSim_trail(double robotLength, double robotWidth);
	~sfmlSim_trail();
	void update(globalRobotPositionData& globalRobotPosition);
	void clear();


	boolean doTrail = true;
	std::vector<  sf::RectangleShape > trailPoints;
	std::vector< sf::VertexArray > trailLines;


protected:
	virtual void draw(sf::RenderTarget& target3, sf::RenderStates states) const;
	sf::VertexArray m_vertices;
	sf::Texture m_texture;

	double rLength;
	double rWidth;
	time_t start = time(0);
	double old_second;
	double current_second;
};