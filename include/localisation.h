#pragma once

#include "globalRobotPositionData.h"
#include "sfmlSim_occupancyMap.h"
#include "sfmlSim_map.h"
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

#define XMAX 175
#define YMAX 105
#define MAX 105*175
/*** Monte Carlo particle 
   x = particle x state
   y = particle y state
   x = particle theta state
   w = particle weight 
***/
struct particle
{
	float x;
	float y; 
	float theta;
	float w;
};

struct coord
{
	int x;
	int y;
	int theta;
};

class localisation : public sf::Drawable, public sf::Transformable
{
public:
	localisation();
	~localisation();	
	void setup(globalRobotPositionData globalRobotPosition, ArRobot& robot, sfmlSim_occupancyMap sfmlOccupancyMap, sfmlSim_map map);
	void update(globalRobotPositionData globalRobotPosition, ArRobot& robot, sfmlSim_occupancyMap sfmlOccupancyMap);
	coord getObsCoords(int x, int y, int theta, float sonarAngle, float sonarDistance);
	coord convertToGlobalCoords(int x, int y, int theta);
	float degrees(float angle);
	float radians(float angle);

	float getNormalRand(float c, float sigma);
	void reassignParticles();

	std::vector<particle> particles;

	int noParticles;
	int boundingBox;

	int xMin;
	int yMin;
	int xMax;
	int yMax;

	float oldX;
	float oldY;
	float oldTheta;


protected:
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
	
};
