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

#include "sfmlSim_robot.h"


class sfmlSim_camera
{
public:
	sfmlSim_camera();
	~sfmlSim_camera();

	void update(sfmlSim_robot& robot);
	void reset();
	void setSize(int width, int height);
	void setCenter(float x, float y);
	void follow();
	void follow(sfmlSim_robot& robot);
	void move(int x, int y);
	void zoom(int value);

	sf::View camera;

protected:
	bool cameraFollowRobot;
	float window_scaling_factor = 20;
	float window_scaling_delta_value = 0.5;
	float camera_movement_scaling_factor = 50;
	int camera_starting_posX = 0;
	int camera_starting_posY = 0;
	const int window_width = 1000;
	const int window_height = 600;
};

