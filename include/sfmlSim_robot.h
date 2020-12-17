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


class sfmlSim_robot : public sf::Drawable, public sf::Transformable
{
public:
	sfmlSim_robot(ArRobot& robot, globalRobotPositionData& globalRobotPosition, sf::Vector2f& startingPos);
	~sfmlSim_robot();

	void update(ArRobot& robot, globalRobotPositionData& globalRobotPosition);


	// robot body
	sf::RectangleShape robot_body;

	// sonar lines
	std::vector< sf::VertexArray > sensorLines = std::vector<sf::VertexArray >(16);


	const int _USE_LPF = 0;
	float LPF_smooth_beta = 0.9;
	float smoothSonarX_array[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	float smoothSonarY_array[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

protected:
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
	sf::VertexArray m_vertices;
	sf::Texture m_texture;

};

