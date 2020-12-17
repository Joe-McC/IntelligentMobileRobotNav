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




class sfmlSim_occupancyMap : public sf::Drawable, public sf::Transformable
{
public:
	sfmlSim_occupancyMap(std::string mapName, int mapOriginX, int mapOriginY, int mapMaxX, int mapMaxY);
	~sfmlSim_occupancyMap();

	int gridCellSize = 100;
	std::string oMapFolderName = "occupancyMapImages/";


	int originX;
	int originY;
	int maxX;
	int maxY;


	sf::Image oMapImage;
	sf::Texture texture;
	sf::Sprite sprite;
	

	double getProbability(int xPos, int yPos);


protected:
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;
};