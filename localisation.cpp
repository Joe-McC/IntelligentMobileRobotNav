#include "localisation.h"


localisation::localisation()
{
}

localisation::~localisation()
{
}


void localisation::setup(globalRobotPositionData globalRobotPosition, ArRobot& robot,  sfmlSim_occupancyMap sfmlOccupancyMap, sfmlSim_map map)
{
	// Initial setup here
	std::cout << "localisation setup" << std::endl;

	xMin = map.minX;
	yMin = map.minY;
	xMax = map.maxX;
	yMax = map.maxY;

	printf("xMin = %d \n ", xMin);
	printf("yMin = %d \n ", yMin);
	printf("xMax = %d \n ", xMax);
	printf("yMax = %d \n ", yMax);

	oldX = globalRobotPosition.x;
	oldY = globalRobotPosition.y;
	oldTheta = globalRobotPosition.th;

	std::freopen("spreadsheet.csv", "w", stdout);
	noParticles = 20000;
	particles.resize(noParticles);
	boundingBox = 1500;
	//if (globalRobotPosition.x < )
	for (int p = 0; p < noParticles; p++) {		
		/*****************GLOBAL LOCALISAION - TRICKY TO GET CORRECT CONVERGENCE *******************/
		float x = (std::rand() % (xMax - xMin) + xMin); // allocated particle a random x cordinate ranging from zero to the total no of x cells
		float y = (std::rand() % (yMax - yMin) + yMin);

		/**********LOCALISTION BUT WITH TRACKING ELEMENT FROM VERY ROUGH INITIAL POSITION***********
		********************* ACCURATE WITH A BOUNDING BOX OF AROUND 1500X1500 *********************
		********************** UP TO 3000X 3000 DEPENDING ON STARTING LOCATION *********************/
		//float x = rand() % (boundingBox * 2) + (globalRobotPosition.x - boundingBox);
		//float y = rand() % (boundingBox * 2) + (globalRobotPosition.y - boundingBox);
		particles[p].x = x;
		particles[p].y = y;
		particles[p].theta = 360 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		particles[p].w = 1.0 / noParticles;

	}
	std::cout << "localisation setup finished" << std::endl;
}

void localisation::update(globalRobotPositionData globalRobotPosition, ArRobot& robot,  sfmlSim_occupancyMap sfmlOccupancyMap)
{
	//std::cout << "localisation update" << std::endl;
	// This is called every iteration (once a second)
	// Update the particles using system dynamics (the prediction phase algorithm in the slides)
	//find particle at previous time step
	float dX = globalRobotPosition.x - oldX;
	float dY = globalRobotPosition.y - oldY;
	float newTheta = globalRobotPosition.th - oldTheta;

	//wrapping theta and orientation to 0 -> 360 degrees
	while (newTheta < 0) {
		newTheta = newTheta + 360;
	}
	while (oldTheta < 0) {
		oldTheta = oldTheta + 360;
	}
	//ensuring robotTh is always greater (so we have positive oldTheta)
	if (oldTheta < newTheta) {
		oldTheta = oldTheta + 360;
	}
	int dTheta = oldTheta - newTheta;

	//setting old coords 
    oldX = globalRobotPosition.x;
	oldY = globalRobotPosition.y;
	oldTheta = globalRobotPosition.th;



	float alpha = 0;
	int count = 0;
	for (int p = 0; p < noParticles; p++) {
		//if (globalRobotPosition.x <= sfmlOccupancyMap.originX || globalRobotPosition.x >= sfmlOccupancyMap.maxX || globalRobotPosition.y <= -sfmlOccupancyMap.maxY || globalRobotPosition.y >= -sfmlOccupancyMap.originY) {
		if (globalRobotPosition.x <= sfmlOccupancyMap.originX || globalRobotPosition.x >= sfmlOccupancyMap.maxX || globalRobotPosition.y <= sfmlOccupancyMap.originY || globalRobotPosition.y >= sfmlOccupancyMap.maxY) {
			printf("ROBOT OUTSIDE OF MAP COORDINATES");
		}
		else if (particles[p].x <= sfmlOccupancyMap.originX - 200 || particles[p].x >= sfmlOccupancyMap.maxX + 200 || particles[p].y <= sfmlOccupancyMap.originY - 200 || particles[p].y >= sfmlOccupancyMap.maxY + 200) {
			particles[p].w = 0.0; // ready for reampling
		}
		else {	
			/**********************
			PREDICTION PHASE
			**********************/
			float i = dX - 1.0;
			float j = dX + 1.0;
			float sigmaL = 0.3;
			float sigmaU = 0.7;

			float sigmaX = (((sigmaU - sigmaL) / (j - i)) * dX) + sigmaL;
			float sigmaY = (((sigmaU - sigmaL) / (j - i)) * dY) + sigmaL;
			float sigmaTheta = (((sigmaU - sigmaL) / (j - i)) * dTheta) + sigmaL;

			float dxNorm = getNormalRand(dX, sigmaX);
			float dyNorm = getNormalRand(dY, sigmaY);
			float dThetaNorm = getNormalRand(dTheta, sigmaTheta);

			particles[p].x = particles[p].x + dxNorm;
			particles[p].y = particles[p].y + dyNorm;
			particles[p].theta = particles[p].theta + dThetaNorm;
	
			//printf("particles[p].theta %f \n, ", particles[p].theta);
			/**********************
			CORRECTION PHASE
			**********************/
			double leftAngle;
			float leftDistance = robot.checkRangeDevicesCurrentPolar(0.0, 90.0, &leftAngle);
				
			double rightAngle; // (in degrees)
			float rightDistance = robot.checkRangeDevicesCurrentPolar(-90.0, 0.0, &rightAngle);
			// check the sensor readings are in range
			coord obsCoords;

			if ((leftDistance > 0) && (leftDistance < 3000) && (leftDistance < rightDistance) && (leftAngle < 90.0)) {				
				obsCoords = getObsCoords(particles[p].x, particles[p].y, oldTheta, leftAngle, leftDistance);
				int obsDist = sqrt((pow((obsCoords.x - particles[p].x), 2.0)) + pow((obsCoords.y - particles[p].y), 2.0));
				if (obsDist < 4000) {					
					float obstProb = sfmlOccupancyMap.getProbability(obsCoords.x, obsCoords.y);

					if (obstProb > 0.0) {

						float prob = 1 - fabs(particles[p].w - obstProb);
						particles[p].w = prob;

					}
					else {
						if (particles[p].w < 0.5) {
							particles[p].w = particles[p].w = 0.0;
						}
						else {
							particles[p].w = 1 - fabs(particles[p].w - obstProb);
						}
					}
				}
			}
				
			else if ((rightDistance > 0) && (rightDistance < 3000) && (rightAngle < 90.0)) {			
				obsCoords = getObsCoords(particles[p].x, particles[p].y, oldTheta, rightAngle, rightDistance);
				int obsDist = sqrt((pow((obsCoords.x - particles[p].x), 2.0)) + pow((obsCoords.y - particles[p].y), 2.0));
				if (obsDist < 4000) {			
					float obstProb = sfmlOccupancyMap.getProbability(obsCoords.x, obsCoords.y);
					if (obstProb > 0.0) {
						float prob = 1 - fabs(particles[p].w - obstProb);					
						particles[p].w = prob;						
					}
					else {
						// the previous particle was of low weight anyway, and the occ grid is zero, 
						// therefore set weight to zero ready for resampling
						if (particles[p].w < 0.5) {
							particles[p].w = particles[p].w = 0.0;
						}
						// the previous particle weight was quite, but there no observation,
						// therefore reduce weight but only by an inverse of the original weight
						else {
							particles[p].w = 1 - fabs(particles[p].w - obstProb);
						}
					}
				}
					
			}
			// the current particle position is likely to be an obstruction, set weight to 0 
			// ready for resampling
			if (sfmlOccupancyMap.getProbability((int)particles[p].x, (int)particles[p].y) > 0.60) {
				particles[p].w = 0;
				count++;
			}

			alpha = alpha + particles[p].w;
		
		}
	}
	//}
	//Ensuring particles are randomly distributed across map / bounding box 
	//if there are no particles left to be sampled
	//i.e. There's no convergence!!!!
	if (count == noParticles) {
		reassignParticles();
	}	

	// normalise weights for the particles
	float maxW = 0;
	for (int i = 0; i < noParticles; i++)
	{
		//particles[i].w = particles[i].w / alpha;
		if (particles[i].w > maxW) {
			maxW = particles[i].w;
		}
	}
	int maxCount = 0;
	float xm = 0;
	float ym = 0;
	int index = 0;
	float estimatedPositionX = 0.0;
	float estimatedPositionY = 0.0;
	
	//Finding the estimated postion (ready for Resample Method 1) and make a list 
	//(well a vector) maxPart of indexes associated with particles which have the 
	//max weight (ready for Resample Method 2)
	std::vector<int> maxPart;
	int maxPartSize = 0;
	for (int i = 0; i < noParticles; i++)
	{
		if (particles[i].w == maxW ) {
			xm = particles[i].x + xm;
			ym = particles[i].y + ym;
			maxCount++;
			index = i;
			maxPart.push_back(index);
			maxPartSize++;
		}
		estimatedPositionX = xm / maxCount; // USED FOR REAMPLING METHOD 1
		estimatedPositionY = ym / maxCount; // USED FOR REAMPLING METHOD 1
	}

	/**********************
	 RESAMPLING PHASE
	**********************/

	float threshold = 1.0 / (float)noParticles;
	for (int p = 0; p < noParticles; p++)
	{	
		int partIndex = 0;
		if (particles[p].w < threshold) {
			/***************** RESAMPLING METHOD 2*******************/
			//--------------------------------------------------------
			if (maxPartSize > 1) {
				int maxIndex = rand() % (maxPartSize - 1);
				partIndex = maxPart[maxIndex];
			}
			else {
				partIndex = rand() % (noParticles - 1);
			}
			float x = rand() % 300 + (particles[partIndex].x - 150);
			float y = rand() % 300 + (particles[partIndex].y - 150);
			//--------------------------------------------------------
			/***************** RESAMPLING METHOD 1*******************/
			//--------------------------------------------------------
			//float x = rand() % 2000 + (estimatedPositionX - 1000);
			//float y = rand() % 2000 + (estimatedPositionY - 1000);
			//--------------------------------------------------------

			//assign particle a new relavant position and resample
			particles[p].x = x;
			particles[p].y = y;
			particles[p].w = 1 / noParticles;
		}		
	}
	float diffX = (float)globalRobotPosition.x - estimatedPositionX;
	float diffY = (float)globalRobotPosition.y - estimatedPositionY;
	float dist = sqrt((diffX*diffX) + (diffY* diffY));
	//printf("dist, %f \n", dist);
	printf("globalRobotPosition.x, %d, globalRobotPosition.y, %d, estimatedPositionX, %f, estimatedPositionY, %f, dist, %f \n", globalRobotPosition.x, globalRobotPosition.y, estimatedPositionX, estimatedPositionY, dist);
}

/* this method take the particle coords (x and y) and angle (theta), as well as the obstruction distance and 
angle given by the sonar sensor; calculates the coords from the local pose, and rotates and transforms this 
to obtain a global position for the perceived object. This is returned as a coord struct*/
coord localisation::getObsCoords(int x, int y, int theta, float sonarAngle, float sonarDistance) {
	float objectX = cos(radians(sonarAngle)) * sonarDistance;
	float objectY = sin(radians(sonarAngle)) * sonarDistance;
	float thetaRads = radians(theta);
	float rotX = (objectX * cos(thetaRads)) - (objectY * sin(thetaRads));
	float rotY = (objectX * sin(thetaRads)) + (objectY * cos(thetaRads));
	coord c;
	c.x = (int)(rotX + x);
	c.y = (int)(rotY + y);
	return c;
}

void localisation::draw(sf::RenderTarget & target, sf::RenderStates states) const
{
	// Look at sfml to see how you can represent particles then draw them to the screen, the "trail" files may help
	sf::RectangleShape rect;
	sf::CircleShape circle;
	states.transform *= getTransform();
	
	
	for (int p = 0; p < noParticles; p++) {	
	
		if (particles[p].w > 0.00) {
			rect.setSize(sf::Vector2f(100.0, 100.0));
			rect.setOutlineThickness(10);
			rect.setOrigin(sf::Vector2f(1.0 / 2, 1.0 / 2));			
			unsigned int pixelValue = (unsigned int)(255 - (particles[p].w * 255));
			sf::Color colour = sf::Color(pixelValue);
			rect.setFillColor(colour);
			rect.setFillColor(sf::Color::Green);
			rect.setPosition(sf::Vector2f(particles[p].x, particles[p].y));
			rect.setRotation(particles[p].theta);			
			target.draw(rect, states);			
		}
	}
}

void localisation::reassignParticles() {
	for (int p = 0; p < noParticles; p++) {
		float x = rand() % (boundingBox * 2) + (oldX - boundingBox);
		float y = rand() % (boundingBox * 2) + (oldY - boundingBox);
		particles[p].x = x;
		particles[p].y = y;
		particles[p].theta = 360 * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		particles[p].w = 1.0 / noParticles;
	}
}

coord localisation::convertToGlobalCoords(int x, int y, int theta) {
	coord coords = { 0,0,0 };
	return coords;
}

float localisation::getNormalRand(float c, float sigma)
{
	float x1, x2, w;
	x1 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
	x2 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
	while (sqrt(x1*x1 + x2 * x2) > 1.0)
	{
		x1 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
		x2 = (float)((double)rand() / (double)RAND_MAX) * 2.0 - 1.0;
	}
	w = sqrt((-2.0 * log(x1*x1 + x2 * x2)) / (x1*x1 + x2 * x2));

	return c + (x1 * w) * sigma;
}

float localisation::radians(float angle)
{
	return (M_PI / 180.0) * angle;
}

float localisation::degrees(float angle)
{
	return (180.0 / M_PI) * angle;
}