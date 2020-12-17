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



class sfmlSim_hud : public sf::Drawable, public sf::Transformable
{
public:
	sfmlSim_hud(sf::Vector2f& startingPos);
	~sfmlSim_hud();

	void update(sf::View& camera, globalRobotPositionData& globalRobotPosition, sf::Vector2f windowCoords);

	
	
	


protected:
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
	sf::VertexArray m_vertices;
	sf::Texture m_texture;

	// FPS Counter
	sf::Clock clock;
	sf::Font font;
	float lastTime = 0;

	sf::Text mapCenter;
	sf::Text robotHome;
	sf::Text fps_text;
	sf::Text globalRobotPos_text;

};

