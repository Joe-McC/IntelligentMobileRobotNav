#include "sfmlSim_occupancyMap.h"

sfmlSim_occupancyMap::sfmlSim_occupancyMap(std::string mapName, int mapOriginX, int mapOriginY, int mapMaxX, int mapMaxY)
{
	std::string oMapFile = oMapFolderName + mapName + ".png";

	originX = mapOriginX;
	originY = mapOriginY;
	maxX = mapMaxX;
	maxY = mapMaxY;

	oMapImage.loadFromFile(oMapFile);
	oMapImage.flipVertically();
	texture.loadFromImage(oMapImage);
	sprite.setTexture(texture);
	sprite.setPosition(sf::Vector2f(mapOriginX, mapOriginY));
	sprite.setScale(sf::Vector2f(100, 100));
}

sfmlSim_occupancyMap::~sfmlSim_occupancyMap()
{
}

double sfmlSim_occupancyMap::getProbability(int xPos, int yPos)
{
	// point positions
	int mapLength = maxX - originX;
	int mapHeight = maxY - originY;

	int pixelValue = 0;

	
	if (xPos <= originX || xPos >= maxX || yPos <= originY || yPos >= maxY) { // Change by JMc - the line below didnt make sense, and top half of map isn't reading occ grid occupancies
		//if (xPos <= originX || xPos >= maxX || yPos <= -maxY || yPos >= -originY) {
		pixelValue = 255;
	}
	else 
	{
		int x = xPos - originX;
		int y = yPos - originY; // Change by JMc - the line below didnt make sense, and top half of map isn't reading occ grid occupancies
		//int y = maxY + yPos;
		x = x / gridCellSize;
		y = y / gridCellSize;
		//printf("OCC GRID: x = %d \n ", x);
		//printf("OCC GRID: y = %d \n ", y);
		
		/******************NOT SURE WHY THERE IS A BUG HERE... BUT WE GET A REAL TIME ABORT IF Y=0 *******************/
		while (y < 1) {
			y++;
		}
		pixelValue = oMapImage.getPixel(x, oMapImage.getSize().y - y).r;
	
		//std::cout << "local " << x << " " << y << " " << pixelValue;
	}

	// calculate and normalise score
	// 255 -> white is not occupied
	// 0 -> black is occupied
	// need to be inverted

	double normalisedValue = double(255 - pixelValue) / 255;

	//std::cout << " " << invertedScore << " " << normalisedValue << std::endl;

	// return
	return normalisedValue;
}

void sfmlSim_occupancyMap::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	target.draw(sprite, states);
}

