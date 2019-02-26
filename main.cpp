#define _USE_MATH_DEFINES
#include <cmath>
#include <GL/glut.h>
#include <iostream>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <cstring>
#include <cstdint>
#include <queue>
#include <vector>
#include <random>
#include "SimplexNoise.h" //Perlin Noise.
#include "Draw.h" //Draw functions.
#include "Cell.h" //Cell struct.
#include "Visuals.h" //Lighting and Materials.
#include "Drone.h" //Drone object and functions.
#include "Config.h" //Custom preset configurations.
#include "MapCell.h" //Cave cell type.
#include "CommunicationMethod.h" //Communication method enum.
using namespace std;

//Cave Properties.
const int caveWidth = 250; //Number of cells making the width of the cave.
const int caveHeight = 180; //Number of cells making the height of the cave.
const int border = 3; //Padding of the cave border on the x-axis.

//Generation Parameters.
const int birthThreshold = 4;
const int deathThreshold = 4;
const int deathChance = 100;
const int birthChance = 100;
const float depth = -1.0f;
vector<vector<int>> presets; //List of cave presets obtained from the config file.

//Cave.
int currentCave[caveWidth][caveHeight];
int tempCave[caveWidth][caveHeight];
Cell startCell;
vector<string> caveStats;

//Camera.
float cameraPanX = 125.0f; //Camera translation along the x-axis.
float cameraPanY = 90.0f; //Camera translation along the y-axis.
float cameraFOV = 150.0f; //Field of View.
bool caveSmooth = false; //Determines if smoothing is enabled when rendering the cave.
int cameraView = -1; //Overview camera view.

//Drone.
vector<Drone> droneList;
bool paused = true;
CommunicationMethod commMethod = Local;
bool showPath = true;

//Controls.
bool ctrlHidden = false;


//Using a chance value, outputs true if a random value is smaller than the chance.
bool thresholdRandom(int chance) {
	return rand() % 100 < chance;
}

int getNeighbourCount(int x, int y) {
	int count = 0;
	for (size_t i = x - 1; i <= x + 1; i++) {
		for (size_t j = y - 1; j <= y + 1; j++) {
			if (!(i == x && j == y) && currentCave[i][j] == Free) {
				count++;
			}
		}
	}
	return count;
}

//Performs one pass of a given ruleset of cellular automata to smooth the cave.
void smoothCave(int iterations) {
	for (size_t i = 0; i < iterations; i++) { //Smooth iterations.
		for (size_t y = border; y < caveHeight - border; y++) { //For each row.
			for (size_t x = border; x < caveWidth - border; x++) { //For each column.
				int neighbours = getNeighbourCount(x, y);
				if (neighbours > birthThreshold && thresholdRandom(birthChance)) {
					tempCave[x][y] = Free; //Cell is born.
				}
				else if (neighbours < deathThreshold && thresholdRandom(deathChance)) {
					tempCave[x][y] = Occupied; //Cell dies.
				}
				else {
					tempCave[x][y] = currentCave[x][y]; //Maintains the cell state.
				}
			}
		}
		//Copies the contents of next cave into current cave.
		memcpy(currentCave, tempCave, sizeof(currentCave));
	}
}

//Generates the initial cave using Simplex noise.
void randomiseCave(float noiseOffsetX, float noiseOffsetY, float noiseScale, float fillPercentage) {

	//Iterates through each cell in the cave.
	for (size_t y = 0; y < caveHeight; y++) { //For each column in the cave.
		for (size_t x = 0; x < caveWidth; x++) { //For each row in the cave.
			 //Cave border.
			 if (x < border || x > caveWidth - border - 1 || y < border || y > caveHeight - border - 1) {
				currentCave[x][y] = Occupied;
				tempCave[x][y] = Occupied;
			}
			else {
				//Maps each x,y coordinate to a scaled and offset coordinate.
				float mappedX = (float)x / caveWidth * noiseScale + noiseOffsetX;
				float mappedY = (float)y / caveHeight * noiseScale + noiseOffsetY;
				//Gets the noise value for the coordinate.
				float noise = SimplexNoise::noise(mappedX, mappedY);
				//Thresholds the noise value into either a free or occupied cell.
				float noiseThreshold = (fillPercentage / 50.0f) - 1.0f;
				currentCave[x][y] = (noise <= noiseThreshold) ? Occupied : Free;
			}
		}
	}
}

//Uses flood fill to target certain cells and modify cells in the temp and current cave accordingly.
void floodFillReplace(int x, int y, int criteria, int currentTarget, int tempTarget) {
	//Current cell is free.
	if (currentCave[x][y] == currentTarget) { return; }

	//Sets current cell to the target state.
	tempCave[x][y] = tempTarget;

	queue<Cell> cellQueue; //Queue of cells to be checked.
	Cell n = Cell(x,y); //Current cell.
	cellQueue.push(n);

	while (!cellQueue.empty()) {
		n = cellQueue.front();
		cellQueue.pop();

		//If the West cell is not part of the border and is free.
		if (n.x-1 >= border && currentCave[n.x-1][n.y] == criteria) {
			currentCave[n.x-1][n.y] = currentTarget;
			tempCave[n.x-1][n.y] = tempTarget;
			Cell west = Cell(n.x-1, n.y);
			cellQueue.push(west);
		}
		//If the East cell is not part of the border and is free.
		if (n.x+1 <= caveWidth - border - 1 && currentCave[n.x+1][n.y] == criteria) {
			currentCave[n.x+1][n.y] = currentTarget;
			tempCave[n.x+1][n.y] = tempTarget;
			Cell east = Cell(n.x+1, n.y);
			cellQueue.push(east);
		}
		//If the South cell is not part of the border and is free.
		if (n.y-1 >= border && currentCave[n.x][n.y-1] == criteria) {
			currentCave[n.x][n.y-1] = currentTarget;
			tempCave[n.x][n.y-1] = tempTarget;
			Cell south = Cell(n.x, n.y-1);
			cellQueue.push(south);
		}
		//If the North cell is not part of the border and is free.
		if (n.y+1 <= caveHeight - border - 1 && currentCave[n.x][n.y+1] == criteria) {
			currentCave[n.x][n.y+1] = currentTarget;
			tempCave[n.x][n.y+1] = tempTarget;
			Cell north = Cell(n.x, n.y+1);
			cellQueue.push(north);
		}
	}
}

//Performs a flood fill at a given position and counts how many cells are captured in the fill.
int floodFillCount(int x, int y, int target) {

	//Current cell is occupied.
	if (tempCave[x][y] == Occupied) { return 0; }

	//Sets current cell to the target state.
	tempCave[x][y] = target;

	int count = 0; //Number of cells in the same region.
	queue<Cell> cellQueue; //Queue of cells to be checked.
	Cell n = Cell(x,y); //Current cell.
	cellQueue.push(n);

	while (!cellQueue.empty()) {
		n = cellQueue.front();
		cellQueue.pop();

		//If the West cell is not part of the border and is free.
		if (n.x-1 >= border && tempCave[n.x-1][n.y] == Free) {
			tempCave[n.x-1][n.y] = target; //Sets the west cell to target state.
			Cell west = Cell(n.x-1, n.y);
			cellQueue.push(west);
			count++;
		}
		//If the East cell is not part of the border and is free.
		if (n.x+1 <= caveWidth - border - 1 && tempCave[n.x+1][n.y] == Free) {
			tempCave[n.x+1][n.y] = target; //Sets the east cell to target state.
			Cell east = Cell(n.x+1, n.y);
			cellQueue.push(east);
			count++;
		}
		//If the South cell is not part of the border and is free.
		if (n.y-1 >= border && tempCave[n.x][n.y-1] == Free) {
			tempCave[n.x][n.y-1] = target; //Sets the south cell to target state.
			Cell south = Cell(n.x, n.y-1);
			cellQueue.push(south);
			count++;
		}
		//If the North cell is not part of the border and is free.
		if (n.y+1 <= caveHeight - border - 1 && tempCave[n.x][n.y+1] == Free) {
			tempCave[n.x][n.y+1] = target; //Sets the north cell to target state.
			Cell north = Cell(n.x, n.y+1);
			cellQueue.push(north);
			count++;
		}
	}

	return count;
}

//Uses a series of flood fills to find an appropriate starting cell.
Cell findStartCell() {

	memcpy(tempCave, currentCave, sizeof(currentCave));
	Cell startCell = Cell(0,0);
	int max = 0;

	//Iterates over all cells in the cave.
	for (size_t x = border; x < caveWidth - border; x++) {
		for (size_t y = border; y < caveHeight - border; y++) {
			//Uses flood fill to see how many cells occupy the same free space.
			int count = floodFillCount(x,y,Occupied);
			if (count > max) {
				max = count;
				startCell.x = x;
				startCell.y = y;
			}
		}
	}
	cout << "[Start] - (" << startCell.x << "," << startCell.y << ") - Count: " << max << "." << endl;
	return startCell;
}

//Changes all inaccessible free cells to occupied cells.
void fillInaccessibleAreas(Cell startCell) {

	//Sets the temporary cave to be fully occupied.
  for (size_t i = 0; i < caveWidth; i++) {
		for (size_t j = 0; j < caveHeight; j++) {
			tempCave[i][j] = Occupied;
		}
	}

	//Converts inaccessible free cells to occupied cells.
	floodFillReplace(startCell.x, startCell.y, Free, Occupied, Free);
	//Copies the improved cave to the current cave structure.
	memcpy(currentCave, tempCave, sizeof(currentCave));
}

//Removes occupied areas not connected to the cave border.
void removeNonBorderOccupiedAreas() {

	//Sets the temporary cave to be fully occupied.
  for (size_t i = 0; i < caveWidth; i++) {
		for (size_t j = 0; j < caveHeight; j++) {
			tempCave[i][j] = Free;
		}
	}

	//Iterates over cells in the cave border.
	//Uses flood fill to find all occupied cells connected to a given border cell
	//and dismisses occupied cells not connected to a border cell.
	for (size_t y = border; y < caveHeight - border; y++) {
		floodFillReplace(border - 1, y, Occupied, Free, Occupied);
		floodFillReplace(caveWidth - border, y, Occupied, Free, Occupied);
	}
	for (size_t x = border; x < caveWidth - border; x++) {
		floodFillReplace(x, border - 1, Occupied, Free, Occupied);
		floodFillReplace(x, caveHeight - border, Occupied, Free, Occupied);
	}

	//Re-adds the borders into the cave.
	for (size_t y = 0; y < caveHeight; y++) {
		//Left Border.
		for (size_t x = 0; x < border; x++) {
			tempCave[x][y] = Occupied;
		}
		//Right Border.
		for (size_t x = caveWidth - border; x < caveWidth; x++) {
			tempCave[x][y] = Occupied;
		}
	}
	for (size_t x = border; x < caveWidth - border; x++) {
		//Bottom Border.
		for (size_t y = 0; y < border; y++) {
			tempCave[x][y] = Occupied;
		}
		//Top Border.
		for (size_t y = caveHeight - border; y < caveHeight; y++) {
			tempCave[x][y] = Occupied;
		}
	}

	memcpy(currentCave, tempCave, sizeof(tempCave));
}

//Generates a cave with no inaccessible areas, no non-border connected occupied cells and smoothed.
void generateCave(float noiseOffsetX, float noiseOffsetY, float fillPercentage, float noiseScale, float smoothIt) {

	//Updates Cave Statistics vector.
	caveStats.clear();
	caveStats.push_back(to_string((int)noiseOffsetX));
	caveStats.push_back(to_string((int)noiseOffsetY));
	caveStats.push_back(to_string((int)fillPercentage));
	caveStats.push_back(to_string((int)noiseScale));
	caveStats.push_back(to_string((int)smoothIt));

	Drone::droneCount = -1;
	cameraView = -1;
	paused = true;

	//Seed output to console.
	cout << "[Seed] - Offset: (" << noiseOffsetX << "," << noiseOffsetY << ") - Scale: " << noiseScale << " - Fill: " << fillPercentage << "% - Iterations: " << smoothIt << endl;

	randomiseCave(noiseOffsetX, noiseOffsetY, noiseScale, fillPercentage); //Uses simplex noise to create a random cave.
	smoothCave(smoothIt); //Uses cellular automata to smooth the cave cells.
	startCell = findStartCell(); //Finds an appropraite starting location.
	fillInaccessibleAreas(startCell); //Removes inaccessible free cells.
	removeNonBorderOccupiedAreas(); //Removes occupied cells not connected to the cave border.

	//Converts the 2D array version of the cave into a vector of vector of ints.
	vector<vector<int>> caveVector;
	for (size_t i = 0; i < caveWidth; i++) {
		vector<int> caveColumn;
		for (size_t j = 0; j < caveHeight; j++) {
			caveColumn.push_back(currentCave[i][j]);
		}
		caveVector.push_back(caveColumn);
	}

	//Initialises the cave dimensions and contents.
	Drone::setParams(caveWidth, caveHeight, caveVector);
}

//Generates a cave from a preset read from a config file.
void generatePresetCave(vector<int> preset) {
	generateCave(preset[0], preset[1], preset[2], preset[3], preset[4]);
}

//Generates a cave with no inaccessible areas, no non-border connected occupied cells and smoothed.
void generateRandomCave() {

	//Uses normal distributions to get random values.
	random_device dev;
	default_random_engine generator(dev());
	normal_distribution<float> fillDistribution(50,5);
	normal_distribution<float> noiseDistribution(55,20);
	normal_distribution<float> smoothDistribution(10,5);

	//Fill percentage. Mean: 50, Std: 5.
	//Percentage of the randomised environment that will be filled.
	float fillPercentage = (int)fillDistribution(generator);
	if (fillPercentage < 40) { fillPercentage = 40; }
	if (fillPercentage > 60) { fillPercentage = 60; }

	//Noise Scale. Mean: 55, Std: 20.
	float noiseScale = (int)noiseDistribution(generator);
	if (noiseScale < 10) { noiseScale = 10; }
	if (noiseScale > 100) { noiseScale = 100; }

	//Smoothing Iterations.
	int smoothIt = (int)smoothDistribution(generator);
	if (smoothIt < 2) { smoothIt = 2; }
	if (smoothIt > 25) { smoothIt = 25; }

	//Gets a random offset value for each direction for simplex noise.
	float noiseOffsetX = rand() % 100000;
	float noiseOffsetY = rand() % 100000;

	generateCave(noiseOffsetX, noiseOffsetY, fillPercentage, noiseScale, smoothIt);
}

//Checks for obstructions between two points in the cave.
bool lineOfSightCheck(int ax, int ay, int bx, int by) {

	//Two points have the same x value.
	if (ax == bx) {
		for (size_t i = min(ay,by); i <= max(ay,by); i++) {
			if (currentCave[ax][i] == Occupied) { return false; }
		}
	}
	//Two points have the same y value.
	else if (ay == by) {
		for (size_t j = min(ax,bx); j <= max(ax,bx); j++) {
			if (currentCave[j][ay] == Occupied) { return false; }
		}
	}
	//Two points are not alligned by either axis.
	else {
		for (size_t x = min(ax,bx); x <= max(ax,bx); x++) {
			float t0 = (x - 0.5f - ax) / (bx - ax);
			float t1 = (x + 0.5f - ax) / (bx - ax);
			float y0 = ay + (t0 * (by - ay));
			float y1 = ay + (t1 * (by - ay));
			float ymin = max((int)floor(floor((min(y0,y1) * 2.0f) + 0.5f) / 2.0f), min(ay,by));
			float ymax = min((int)ceil(floor((max(y0,y1) * 2.0f) + 0.5f) / 2.0f), max(ay,by));
			for (size_t y = ymin; y <= ymax; y++) {
				if (currentCave[x][y] == Occupied) { return false; }
			}
		}
	}

	return true;
}

//If enough time has elapsed between last communication then the interla maps of the drones are combined.
void communicate(int a, int b) {
	//Informs the drone of nearby drones to help frontier selection.
	droneList[a].addNearDrone(droneList[b].posX, droneList[b].posY);
	droneList[b].addNearDrone(droneList[a].posX, droneList[a].posY);

	//Check to see if enough time has elapsed between communications with drones a and b.
	if (droneList[a].allowCommunication(b)) {
		droneList[a].combineMaps(droneList[b].internalMap, droneList[b].frontierCells, b);
		droneList[b].combineMaps(droneList[a].internalMap, droneList[a].frontierCells, a);
	}
}

//Checks all drones if they are in communication distance and can view each other with no obstruction.
//If two drones satisfy these conditions they communicate their internal maps.
void pollLocalCommunication() {

	//Skip polling communication if there arn't enough drones to communicate.
	if (Drone::droneCount <= 1) { return; }

	//For each unique pair of drones in communication range.
	for (size_t i = 0; i < Drone::droneCount - 1; i++) {
		for (size_t j = i + 1; j < Drone::droneCount; j++) {
			float dx = droneList[i].posX - droneList[j].posX;
			float dy = droneList[i].posY - droneList[j].posY;
			float dist = pow(pow(dx, 2.0f) + pow(dy, 2.0f), 0.5f);
			//If distance is small then there can be no obstructions.
			if (dist <= 1) {
				communicate(i,j);
			}
			else if (dist < Drone::communicationRadius) {
				int ix = (int)droneList[i].posX;
				int iy = (int)droneList[i].posY;
				int jx = (int)droneList[j].posX;
				int jy = (int)droneList[j].posY;
				//Checks that there are no obstructions in the path from drone I to J.
				if (lineOfSightCheck(ix, iy, jx, jy)) {
					communicate(i ,j);
				}
			}
		}
	}
}

//All drones can communicate without being in view distance of each other.
void pollGlobalCommunication() {

	//Skip polling communication if there arn't enough drones to communicate.
	if (Drone::droneCount <= 1) { return; }

	//For each unique pair of drones in communication range.
	for (size_t i = 0; i < Drone::droneCount - 1; i++) {
		for (size_t j = i + 1; j < Drone::droneCount; j++) {
			communicate(i,j);
		}
	}
}

//Displays the camera mode in the bottom-left corner as well as other useful statistics.
void displayStatistics(const float* textColour) {

	//Text parameters.
	const int yPad = 20;
	const int xPad = 30;
	const int segments = 6;
	const int windowW = glutGet(GLUT_WINDOW_WIDTH) - (2 * xPad);
	const int windowH = glutGet(GLUT_WINDOW_HEIGHT) - (2 * yPad);
	const float statSize = 0.13f;
	const float textSize = 0.11f;

	//Statistics.
	Draw::drawText(xPad, windowH, textSize, "Cave Stats" , textColour);
	Draw::drawText(xPad, windowH - yPad, statSize, ("X Offset - " + caveStats[0]).c_str(), textColour);
	Draw::drawText(xPad, windowH - (yPad * 2), statSize, ("Y Offset - " + caveStats[1]).c_str(), textColour);
	Draw::drawText(xPad, windowH - (yPad * 3), statSize, ("Fill - " + caveStats[2] + "%").c_str(),  textColour);
	Draw::drawText(xPad, windowH - (yPad * 4), statSize, ("Noise Scale - " + caveStats[3]).c_str(), textColour);
	Draw::drawText(xPad, windowH - (yPad * 5), statSize, ("Smooth Iterations - " + caveStats[4]).c_str(), textColour);

	//Other Statistics.
	string state = paused ? "Paused" : "Running";
	string comm = (commMethod == Local) ? "Local" : "Global";
	string drone = (Drone::droneCount == -1) ? "0" : to_string(Drone::droneCount);
	Draw::drawText(xPad, windowH - (yPad * 7), textSize, "Other Stats" , textColour);
	Draw::drawText(xPad, windowH - (yPad * 8), statSize, ("State - " + state).c_str(), textColour);
	Draw::drawText(xPad, windowH - (yPad * 9), statSize, ("No. Drones - " + drone).c_str(), textColour);
	Draw::drawText(xPad, windowH - (yPad * 10), statSize, ("Communication - " + comm).c_str(), textColour);

	//Overview.
	if (cameraView == -1) {
		//Camera Mode.
		Draw::drawText(30, 30, 0.15f, (char *)"Overview", textColour);
	}
	//Drone follow.
	else {
		vector<string> stats = droneList[cameraView].getStatistics();
		//Camera Mode.
		Draw::drawText(xPad, yPad, 0.15f, droneList[cameraView].name.c_str(), textColour);
		//Header Text.
		Draw::drawText(xPad + (1 * windowW / segments), 2 * yPad, textSize, "Distance Travelled", textColour);
		Draw::drawText(xPad + (2 * windowW / segments), 2 * yPad, textSize, "Free Cells", textColour);
		Draw::drawText(xPad + (3 * windowW / segments), 2 * yPad, textSize, "Occupied Cells", textColour);
		Draw::drawText(xPad + (4 * windowW / segments), 2 * yPad, textSize, "Free from Comm.", textColour);
		Draw::drawText(xPad + (5 * windowW / segments), 2 * yPad, textSize, "Occupied from Comm.", textColour);
		//Statistics.
		Draw::drawText(xPad + (1 * windowW / segments), yPad, statSize, stats[0].c_str(), textColour);
		Draw::drawText(xPad + (2 * windowW / segments), yPad, statSize, stats[1].c_str(), textColour);
		Draw::drawText(xPad + (3 * windowW / segments), yPad, statSize, stats[2].c_str(),  textColour);
		Draw::drawText(xPad + (4 * windowW / segments), yPad, statSize, stats[3].c_str(), textColour);
		Draw::drawText(xPad + (5 * windowW / segments), yPad, statSize, stats[4].c_str(), textColour);
	}
}

//Displays the control text at the top-left of the window.
void displayControls() {

	int leftPad = glutGet(GLUT_WINDOW_WIDTH) / 2 - 200;
	int topPad = glutGet(GLUT_WINDOW_HEIGHT) - 100;
	Draw::drawText(glutGet(GLUT_WINDOW_WIDTH) / 2 - 55, topPad, 0.25f, (char *)"Controls", textColour);
	Draw::drawText(leftPad, topPad - 100, 0.15f, (char *)"q - Quit", textColour);
	Draw::drawText(leftPad, topPad - 150, 0.15f, (char *)"[ - Zoom In", textColour);
	Draw::drawText(leftPad, topPad - 200, 0.15f, (char *)"] - Zoom Out", textColour);
	Draw::drawText(leftPad, topPad - 250, 0.15f, (char *)"Scroll - Zoom", textColour);
	Draw::drawText(leftPad, topPad - 300, 0.15f, (char *)"r - Generate Cave", textColour);
	Draw::drawText(leftPad, topPad - 350, 0.15f, (char *)"t - Toggle Smooth Cells.", textColour);
	Draw::drawText(leftPad, topPad - 400, 0.15f, (char *)"p - Toggle Drone Paths.", textColour);
	Draw::drawText(leftPad, topPad - 450, 0.15f, (char *)"v - Change Camera View.", textColour);
	Draw::drawText(leftPad, topPad - 500, 0.15f, (char *)"SPACE - Resume/Pause Simulation.", textColour);
	Draw::drawText(leftPad, topPad - 550, 0.15f, (char *)"F1-F5 - Load Cave Presets.", textColour);
	Draw::drawText(leftPad, topPad - 600, 0.15f, (char *)"1-9 - Start simulation with N drones.", textColour);
	Draw::drawText(leftPad, topPad - 750, 0.15f, (char *)"H - Show/Hide Controls", textColour);
	displayStatistics(textColour);
}

//Draws the cave structure, features no smoothing.
void renderCaveNormal() {

	//For each cell in the cave.
	for (size_t i = 0; i < caveWidth; i++) {
		for (size_t j = 0; j < caveHeight; j++) {
			if (currentCave[i][j] == Occupied) {
				glPushMatrix();
				//Translate to cell position.
				glTranslatef((float)i, (float)j, 0);

				//Main face.
				glColor4fv(caveFaceColour);
				if (currentCave[i][j] == Occupied) {
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3f(0.0f, 0.0f, 1.0f);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(0.5f, 0.5f, 0);
					glEnd();
				}

				glColor4fv(caveDepthColour);
				//Left Depth face.
				if (i - 1 >= 0 && currentCave[i-1][j] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(-1.0f, 0.0f, 0.0f);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(-0.5f, -0.5f, depth);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(-0.5f, 0.5f, depth);
					glEnd();
				}
				//Right Depth face.
				if (i + 1 < caveWidth && currentCave[i+1][j] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(1.0f, 0.0f, 0.0f);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(0.5f, -0.5f, depth);
					glVertex3f(0.5f, 0.5f, 0);
					glVertex3f(0.5f, 0.5f, depth);
					glEnd();
				}
				//Bottom Depth face.
				if (j - 1 >= 0 && currentCave[i][j-1] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(0.0f, -1.0f, 0.0f);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(-0.5f, -0.5f, depth);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(0.5f, -0.5f, depth);
					glEnd();
				}
				//Top Depth face.
				if (j + 1 < caveHeight && currentCave[i][j+1] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(0.0f, 1.0f, 0.0f);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(-0.5f, 0.5f, depth);
					glVertex3f(0.5f, 0.5f, 0);
					glVertex3f(0.5f, 0.5f, depth);
					glEnd();
				}

				glPopMatrix();
			}
		}
	}
}

//Draws the cave structure, uses the marching squares algorithm to smooth edges.
void renderCaveSmooth() {

	//Normals.
	const float nl[3] = {-1.0f, 0.0f, 0.0f};
	const float nr[3] = {1.0f, 0.0f, 0.0f};
	const float nt[3] = {0.0f, 1.0f, 0.0f};
	const float nb[3] = {0.0f, -1.0f, 0.0f};
	const float nf[3] = {0.0f, 0.0f, 1.0f};
	const float ntl[3] = {-1.0f, 1.0f, 0.0f};
	const float ntr[3] = {1.0f, 1.0f, 0.0f};
	const float nbl[3] = {-1.0f, -1.0f, 0.0f};
	const float nbr[3] = {1.0f, -1.0f, 0.0f};

	//Vertices.
	const float near[8][3] = {
		{-0.5f, 0.5f, 0}, //0-Top-Left.
		{0, 0.5f, 0}, //1-Top-Middle.
		{0.5f, 0.5f, 0}, //2-Top-Right.
		{0.5f, 0, 0}, //3-Middle-Right.
		{0.5f, -0.5f, 0}, //4-Bottom-Right.
		{0, -0.5f, 0}, //5-Bottom-Middle.
		{-0.5f, -0.5f, 0}, //6-Bottom-Left.
		{-0.5f, 0, 0} //7-Middle-Left.
	};

	const float far[8][3] = {
		{-0.5f, 0.5f, depth}, //0-Top-Left.
		{0, 0.5f, depth}, //1-Top-Middle.
		{0.5f, 0.5f, depth}, //2-Top-Right.
		{0.5f, 0, depth}, //3-Middle-Right.
		{0.5f, -0.5f, depth}, //4-Bottom-Right.
		{0, -0.5f, depth}, //5-Bottom-Middle.
		{-0.5f, -0.5f, depth}, //6-Bottom-Left.
		{-0.5f, 0, depth} //7-Middle-Left.
	};

	glPushMatrix();
	glTranslatef(0.5f, 0.5f, 0.0f);

	//Iterates over each 2x2 block of cells in the cave.
	for (size_t i = 0; i < caveWidth - 1; i++) {
		for (size_t j = 0; j < caveHeight - 1; j++) {
			glPushMatrix();
			//Translate to cell position.
			glTranslatef((float)i, (float)j, 0);
			glColor4fv(caveFaceColour);

			//Gets a 4-bit value based on occupied cells in the block for use by the marching squares algorithm.
			int tr = currentCave[i+1][j+1] == Occupied;
			int tl = currentCave[i][j+1] == Occupied;
			int bl = currentCave[i][j] == Occupied;
			int br = currentCave[i+1][j] == Occupied;

			int vertexInd = (tl << 3) + (tr << 2) + (br << 1) + bl;

			//Look-up table contour lines.
			switch (vertexInd) {
				case 1:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[5]);
					glVertex3fv(near[7]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(ntr);
					glVertex3fv(near[5]);
					glVertex3fv(near[7]);
					glVertex3fv(far[7]);
					glVertex3fv(far[5]);
					glEnd();
					break;
				case 2:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
				  glVertex3fv(near[5]);
					glVertex3fv(near[4]);
					glVertex3fv(near[3]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(ntl);
					glVertex3fv(near[5]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[5]);
					glEnd();
					break;
				case 3:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[4]);
					glVertex3fv(near[7]);
					glVertex3fv(near[3]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nt);
					glVertex3fv(near[7]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[7]);
					glEnd();
					break;
				case 4:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[3]);
					glVertex3fv(near[2]);
					glVertex3fv(near[1]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nbl);
					glVertex3fv(near[3]);
					glVertex3fv(near[1]);
					glVertex3fv(far[1]);
					glVertex3fv(far[3]);
					glEnd();
					break;
				case 5:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[5]);
					glVertex3fv(near[7]);
					glVertex3fv(near[3]);
					glVertex3fv(near[1]);
					glVertex3fv(near[2]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nbr);
					glVertex3fv(near[5]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[5]);
					glNormal3fv(ntl);
					glVertex3fv(near[7]);
					glVertex3fv(near[1]);
					glVertex3fv(far[1]);
					glVertex3fv(far[7]);
					glEnd();
					break;
				case 6:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[5]);
					glVertex3fv(near[4]);
					glVertex3fv(near[1]);
					glVertex3fv(near[2]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nl);
					glVertex3fv(near[1]);
					glVertex3fv(near[5]);
					glVertex3fv(far[5]);
					glVertex3fv(far[1]);
					glEnd();
					break;
				case 7:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[4]);
					glVertex3fv(near[7]);
					glVertex3fv(near[2]);
					glVertex3fv(near[1]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(ntl);
					glVertex3fv(near[1]);
					glVertex3fv(near[7]);
					glVertex3fv(far[7]);
					glVertex3fv(far[1]);
					glEnd();
					break;
				case 8:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[7]);
					glVertex3fv(near[1]);
					glVertex3fv(near[0]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nbr);
					glVertex3fv(near[1]);
					glVertex3fv(near[7]);
					glVertex3fv(far[7]);
					glVertex3fv(far[1]);
					glEnd();
					break;
				case 9:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[5]);
					glVertex3fv(near[0]);
					glVertex3fv(near[1]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nr);
					glVertex3fv(near[1]);
					glVertex3fv(near[5]);
					glVertex3fv(far[5]);
					glVertex3fv(far[1]);
					glEnd();
					break;
				case 10:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[5]);
					glVertex3fv(near[4]);
					glVertex3fv(near[7]);
					glVertex3fv(near[3]);
					glVertex3fv(near[0]);
					glVertex3fv(near[1]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nbl);
					glVertex3fv(near[5]);
					glVertex3fv(near[7]);
					glVertex3fv(far[7]);
					glVertex3fv(far[5]);
					glNormal3fv(ntr);
					glVertex3fv(near[1]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[1]);
					glEnd();
					break;
				case 11:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[4]);
					glVertex3fv(near[0]);
					glVertex3fv(near[3]);
					glVertex3fv(near[1]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(ntr);
					glVertex3fv(near[1]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[1]);
					glEnd();
					break;
				case 12:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[7]);
					glVertex3fv(near[3]);
					glVertex3fv(near[0]);
					glVertex3fv(near[2]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nb);
					glVertex3fv(near[7]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[7]);
					glEnd();
					break;
				case 13:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[5]);
					glVertex3fv(near[6]);
					glVertex3fv(near[3]);
					glVertex3fv(near[0]);
					glVertex3fv(near[2]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nbr);
					glVertex3fv(near[5]);
					glVertex3fv(near[3]);
					glVertex3fv(far[3]);
					glVertex3fv(far[5]);
					glEnd();
					break;
				case 14:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[5]);
					glVertex3fv(near[4]);
					glVertex3fv(near[7]);
					glVertex3fv(near[2]);
					glVertex3fv(near[0]);
					glEnd();
					glColor4fv(caveDepthColour);
					glBegin(GL_QUADS);
					glNormal3fv(nbl);
					glVertex3fv(near[5]);
					glVertex3fv(near[7]);
					glVertex3fv(far[7]);
					glVertex3fv(far[5]);
					glEnd();
					break;
				case 15:
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3fv(nf);
					glVertex3fv(near[6]);
					glVertex3fv(near[4]);
					glVertex3fv(near[0]);
					glVertex3fv(near[2]);
					glEnd();
					break;
			}
			glPopMatrix();
		}
	}
	glPopMatrix();
}

//Draws all the available drones onto the screen.
void renderDrone() {
	//Draws a drone at the starting location.
	for (size_t i = 0; i < droneList.size(); i++) {
		Draw::drawDrone(droneList[i].posX, droneList[i].posY, depth, Drone::searchRadius, droneList[i].name, droneList[i].bearing);
	}
}

//Positions the camera according to which drone it is following.
void setCameraView() {

	//Camera View.
	if (cameraView == -1) {
		//Eye Position, Reference Point, Up Vector.
		gluLookAt(cameraPanX, cameraPanY, 25, cameraPanX, cameraPanY, 0, 0, 1, 0);
	}
	else {
		//Eye Position, Reference Point, Up Vector.
		gluLookAt(droneList[cameraView].posX, droneList[cameraView].posY, 25, droneList[cameraView].posX, droneList[cameraView].posY, 0, 0, 1, 0);
	}
}

//Initialises the set of drones.
void droneListInit() {
	droneList.clear();
	string droneNames[9] = {"Alpha", "Beta", "Gamma", "Delta", "Epsilon", "Zeta", "Eta", "Theta", "Iota"};
	for (size_t i = 0; i < Drone::droneCount; i++) {
		Drone newDrone;
		newDrone.init(i, startCell.x, startCell.y, droneNames[i]);
		droneList.push_back(newDrone);
	}
}

//Draws the discovered cells of all drones in overview mode or one particular drone.
void drawDiscoveredCells() {
	//If there no drones present return.
	if (Drone::droneCount == -1) { return; };

	renderDrone(); //Renders the drones.
	if (cameraView == -1) { //Overview mode.
		 //Free, Occupied, Frontier colours.
		float colours[3][4] = {{0.0f, 1.0f, 0.0f, 0.5f}, {1.0f, 0.0f, 0.0f, 0.5f}, {0.0f, 1.0f, 0.0f, 0.5f}};
		//Draws the free and occupied cells of every drone.
		for (size_t i = 0; i < Drone::droneCount; i++) {
			Draw::drawDiscoveredCells(caveWidth, caveHeight, depth, droneList[i].internalMap, colours);
		}
	}
	else { //Following particular drone.
		//Free, Occupied, Frontier colours.
		float colours[3][4] = {{0.0f, 1.0f, 0.0f, 0.5f}, {1.0f, 0.0f, 0.0f, 0.5f}, {0.0f, 1.0f, 1.0f, 0.5f}};
		//Draws the free, occupied and frontier cells of the selected drone.
		Draw::drawDiscoveredCells(caveWidth, caveHeight, depth, droneList[cameraView].internalMap, colours);
	}

}

//Draws the discovered cells of all drones in overview mode or one particular drone.
void drawDronePath() {
	//If there exists at least one drone.
	if (Drone::droneCount != -1) {
		if (cameraView != -1) { //Draws only the path of the drone being followed.
 			Draw::drawDronePath(droneList[cameraView].pathList, depth / 2.0f, 0.25f, colourMask[cameraView]);
 		}
		else { //Draws all drone paths.
			for (size_t i = 0; i < Drone::droneCount; i++) {
				Draw::drawDronePath(droneList[i].pathList, depth / 2.0f, 0.25f, colourMask[i]);
			}
		}
	}
}

//Idle loop. Processes drone functions every timestep.
void idle() {
	if (!paused) {
		//2500 Microsecond pause.
		usleep(2500);
		//Communication between drones.
		(commMethod == Local) ? pollLocalCommunication() : pollGlobalCommunication();
		//Processes each drone.
		for (size_t i = 0; i < Drone::droneCount; i++) {
			if (!droneList[i].complete) {
				droneList[i].process();
			}
		}
		glutPostRedisplay();
	}
}

//Displays the cave, drones, text, etc on screen.
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(cameraFOV, (GLdouble)glutGet(GLUT_WINDOW_WIDTH) / (GLdouble)glutGet(GLUT_WINDOW_HEIGHT), 0.1f, 250.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Applies transformations for the camera.
	setCameraView();

	glPushMatrix();

	//Lighting.
	setLight(globalLight);
	glEnable(GL_LIGHTING);


	//Draws Cave Background then Border then finally the cave structure.
	Draw::drawBackground(depth, caveWidth, caveHeight);
	Draw::drawBorder(depth, caveWidth, caveHeight);
	caveSmooth ? renderCaveSmooth() : renderCaveNormal();
	//Draws discovered cells by the drones and their paths.
	drawDiscoveredCells();
	//Draws the drone paths.
	if (showPath) { drawDronePath(); }

	//Dark translucent overlay onto cave to make controls mode visible.
	if (!ctrlHidden) {
		const float l = 1000.0f;
		glColor4f(0.0f, 0.0f, 0.0f, 0.90f);
		glBegin(GL_QUADS);
		glVertex3f(l,l,1.0f);
		glVertex3f(-l,l,1.0f);
		glVertex3f(-l,-l,1.0f);
		glVertex3f(l,-l,1.0f);
		glEnd();
	}

	glPopMatrix();

	//Displays control text on the screen.
	if (ctrlHidden) {
		displayStatistics(textColour);
	}
	else { //Display control screen.
		displayControls();
	}

	glutSwapBuffers();
}

//Reshapes the view window.
void reshape(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(cameraFOV, (GLdouble)w / (GLdouble)h, 0.1f, 250.0f);
}

//Mouse event functions.
void mouseInput(int button, int state, int x, int y) {

	switch (button) {
		//Scroll Up / Zoom Out.
		case 3: cameraFOV = (cameraFOV <= 25.0f) ? 25.0f : cameraFOV - 2.5f; break;
		//Scroll Down / Zoom In.
		case 4: cameraFOV = (cameraFOV >= 170.0f) ? 170.0f : cameraFOV + 2.5f; break;
	}

	//Reshapes the display.
	reshape(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	glutPostRedisplay();
}

//Keyboard event functions.
void keyboardInput(unsigned char key, int, int) {
	switch (key) {
		//Exits the program.
		case 'Q':
		case 'q': exit(1); break;
		//Zoom Out.
		case ']': cameraFOV = (cameraFOV <= 25.0f) ? 25.0f : cameraFOV - 2.5f; break;
		//Zoom In.
		case '[': cameraFOV = (cameraFOV >= 170.0f) ? 170.0f : cameraFOV + 2.5f; break;
		//Generates an improved cave.
		case 'R':
		case 'r':
			cout << "[Random]" << endl;
			generateRandomCave();
			break;
		case 'P':
		case 'p': showPath = !showPath; break;
		//Pauses the simulation.
		case ' ': if (Drone::droneCount != -1) { paused = !paused; } break;
		//Smoothing.
		case 'T':
		case 't': caveSmooth = !caveSmooth; break;
		//Switch Camera View.
		case 'v':
		case 'V': cameraView = (++cameraView >= Drone::droneCount) ? -1 : cameraView; break;
		//Start simulation with n drones.
		case '1': Drone::droneCount = 1; droneListInit(); break;
		case '2':	Drone::droneCount = 2;	droneListInit();break;
		case '3':	Drone::droneCount = 3;	droneListInit(); break;
		case '4':	Drone::droneCount = 4;	droneListInit(); break;
		case '5':	Drone::droneCount = 5;	droneListInit(); break;
		case '6':	Drone::droneCount = 6;	droneListInit(); break;
		case '7':	Drone::droneCount = 7;	droneListInit(); break;
		case '8':	Drone::droneCount = 8;	droneListInit(); break;
		case '9':	Drone::droneCount = 9;	droneListInit(); break;
		//Show/Hide Controls.
		case 'h':
		case 'H': ctrlHidden = !ctrlHidden; break;
	}
	glutPostRedisplay();
}

//Special Key event functions.
void specialKeyInput(int key, int x, int y) {
	switch (key) {
		//Pan Up.
		case GLUT_KEY_UP:
			cameraPanY += 0.2f * cameraFOV;
			if (cameraPanY > caveHeight + 50.0f) { cameraPanY = caveHeight + 50.0f; }
			break;
		//Pan Down.
		case GLUT_KEY_DOWN:
			cameraPanY -= 0.2f * cameraFOV;
			if (cameraPanY < -50.0f) { cameraPanY = -50.0f; }
			break;
		//Pan Left.
		case GLUT_KEY_LEFT:
			cameraPanX -= 0.2f * cameraFOV;
			if (cameraPanX < -50.0f) { cameraPanX = -50.0f; }
			break;
		//Pan Right.
		case GLUT_KEY_RIGHT:
			cameraPanX += 0.2f * cameraFOV;
			if (cameraPanX > caveWidth + 50.0f) { cameraPanX = caveWidth + 50.0f; }
			break;
		//Cave Generation Presets.
		case GLUT_KEY_F1: //Jagged cave.
			cout << "[Preset 1]" << endl;
			generatePresetCave(presets[0]);
			break;
		case GLUT_KEY_F2:
			cout << "[Preset 2]" << endl;
			generatePresetCave(presets[1]);
			break;
		case GLUT_KEY_F3:
			cout << "[Preset 3]" << endl;
			generatePresetCave(presets[2]);
			break;
		case GLUT_KEY_F4:
			cout << "[Preset 4]" << endl;
			generatePresetCave(presets[3]);
			break;
		case GLUT_KEY_F5:
			cout << "[Preset 5]" << endl;
			generatePresetCave(presets[4]);
			break;
	}
	glutPostRedisplay();
}

void init() {
	//Material.
	setMaterial(globalMaterial);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	//Blending.
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA , GL_ONE_MINUS_SRC_ALPHA);

	//Light.
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//Misc.
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);

	//Reads config file for default values.
	vector<int> presetSing;
	presetSing.push_back(0);
	presetSing.push_back(0);
	presetSing.push_back(50);
	presetSing.push_back(50);
	presetSing.push_back(10);
	presets.push_back(presetSing);
	presets.push_back(presetSing);
	presets.push_back(presetSing);
	presets.push_back(presetSing);
	presets.push_back(presetSing);

	Config::readConfig(presets, commMethod);
}

int main(int argc, char* argv[]) {

	//Random.
	srand(time(NULL));

	//Window Properties.
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(1600, 900);
	glutInitWindowPosition(25, 25);
	glutCreateWindow("Cave Generation");

	//Event Functions.
	glutMouseFunc(mouseInput);
	glutKeyboardFunc(keyboardInput);
	glutSpecialFunc(specialKeyInput);
	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutIdleFunc(idle);

	//Cave Generation.
	generateRandomCave();

	init();
	glutMainLoop();
	return 0;
}
