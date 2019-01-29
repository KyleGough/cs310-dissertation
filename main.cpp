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
#include "SimplexNoise.h" //Perlin Noise.
#include "Draw.h" //Draw functions.
#include "Cell.h" //Cell struct.
#include "Drone.h" //Drone object and functions.
using namespace std;

/*@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@*/

//Light Struct.
struct Light {
	size_t name;
	float ambient[4];
	float diffuse[4];
	float specular[4];
	float position[4];
};

//Material Struct.
struct Material {
	float ambient[4];
	float diffuse[4];
	float specular[4];
	float shininess;
};

//Global Light Source.
const Light globalLight = {
	GL_LIGHT0,
	{0.3f, 0.3f, 0.3f, 1.0f},
	{2.0f, 2.0f, 2.0f, 1.0f},
	{0.5f, 0.5f, 0.5f, 1.0f},
	{120.0f, 90.0f, 50.0f, 1.0f}
};

//Global Material.
const Material globalMaterial = {
	{0.02f, 0.02f, 0.02f, 1.0f},
  {0.01f, 0.01f, 0.01f, 1.0f},
	{0.4f, 0.4f, 0.4f, 1.0f},
	0.078125f

	/*###{0.15f, 0.15f, 0.15f, 1.0f},
  {0.4f, 0.4f, 0.4f, 1.0f},
	{0.7746f, 0.7746f, 0.7746f, 1.0f},
	76.8f*/
};

//Sets the properties of a given material.
void setMaterial(const Material& material) {
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material.ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material.diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material.specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material.shininess);
}

//Sets the properties of a given light.
void setLight(const Light& light) {
	glLightfv(light.name, GL_AMBIENT, light.ambient);
	glLightfv(light.name, GL_DIFFUSE, light.diffuse);
	glLightfv(light.name, GL_SPECULAR, light.specular);
	glLightfv(light.name, GL_POSITION, light.position);
	//glLightf(light.name, GL_SPOT_CUTOFF, 60.0f);
}

/*@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@*/

//Cave Properties.
const int caveWidth = 250; //Number of cells making the width of the cave.
const int caveHeight = 180; //Number of cells making the height of the cave.
const int border = 3; //Padding of the cave border on the x-axis.
enum MapCell { Free, Occupied, Unknown, Frontier };

//Generation Parameters.
int fillPercentage = 50; //Percentage of the randomised environment that will be filled.
const int birthThreshold = 4;
const int deathThreshold = 4;
const int deathChance = 100;
const int birthChance = 100;
const float depth = -1.0f;

//Simplex Noise.
float noiseScale = 40.0f;
float noiseOffsetX = 100.0f;
float noiseOffsetY = 100.0f;

//Cave.
int currentCave[caveWidth][caveHeight];
int tempCave[caveWidth][caveHeight];
Cell startCell = Cell(0,0);

//Camera.
float cameraPanX = 125.0f; //Camera translation along the x-axis.
float cameraPanY = 90.0f; //Camera translation along the y-axis.
float cameraFOV = 150.0f; //Field of View.
bool caveSmooth = true;

//Drone.
Drone droneA;

/*@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@*/

//Using a chance value, outputs true if a random value is smaller than the chance.
bool thresholdRandom(int chance) {
	return rand() % 100 < chance;
}

int getNeighbourCount(int x, int y) {
	int count = 0;
	for (int i = x - 1; i <= x + 1; i++) {
		for (int j = y - 1; j <= y + 1; j++) {
			if (!(i == x && j == y) && currentCave[i][j] == Free) {
				count++;
			}
		}
	}
	return count;
}

//Performs one pass of a given ruleset of cellular automata to smooth the cave.
void smoothCave(int iterations) {
	for (int i = 0; i < iterations; i++) { //Smooth iterations.
		for (int y = border; y < caveHeight - border; y++) { //For each row.
			for (int x = border; x < caveWidth - border; x++) { //For each column.
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
void randomiseCave() {

	//Gets a random offset value for each direction for simplex noise.
	noiseOffsetX = rand() % 100000;
	noiseOffsetY = rand() % 100000;

	//Iterates through each cell in the cave.
	for (int y = 0; y < caveHeight; y++) { //For each column in the cave.
		for (int x = 0; x < caveWidth; x++) { //For each row in the cave.
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


/*@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@*/

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
	for (int x = border; x < caveWidth - border; x++) {
		for (int y = border; y < caveHeight - border; y++) {
			//Uses flood fill to see how many cells occupy the same free space.
			int count = floodFillCount(x,y,Occupied);
			if (count > max) {
				max = count;
				startCell.x = x;
				startCell.y = y;
			}
		}
	}
	cout << " + Start: (" << startCell.x << "," << startCell.y << ") with count: " << max << "." << endl; //###DEBUG
	return startCell;
}

//Changes all inaccessible free cells to occupied cells.
void fillInaccessibleAreas(Cell startCell) {

	//Sets the temporary cave to be fully occupied.
  for (int i = 0; i < caveWidth; i++) {
		for (int j = 0; j < caveHeight; j++) {
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
  for (int i = 0; i < caveWidth; i++) {
		for (int j = 0; j < caveHeight; j++) {
			tempCave[i][j] = Free;
		}
	}

	//Iterates over cells in the cave border.
	//Uses flood fill to find all occupied cells connected to a given border cell
	//and dismisses occupied cells not connected to a border cell.
	for (int y = border; y < caveHeight - border; y++) {
		floodFillReplace(border - 1, y, Occupied, Free, Occupied);
		floodFillReplace(caveWidth - border, y, Occupied, Free, Occupied);
	}
	for (int x = border; x < caveWidth - border; x++) {
		floodFillReplace(x, border - 1, Occupied, Free, Occupied);
		floodFillReplace(x, caveHeight - border, Occupied, Free, Occupied);
	}

	//Re-adds the borders into the cave.
	for (int y = 0; y < caveHeight; y++) {
		//Left Border.
		for (int x = 0; x < border; x++) {
			tempCave[x][y] = Occupied;
		}
		//Right Border.
		for (int x = caveWidth - border; x < caveWidth; x++) {
			tempCave[x][y] = Occupied;
		}
	}
	for (int x = border; x < caveWidth - border; x++) {
		//Bottom Border.
		for (int y = 0; y < border; y++) {
			tempCave[x][y] = Occupied;
		}
		//Top Border.
		for (int y = caveHeight - border; y < caveHeight; y++) {
			tempCave[x][y] = Occupied;
		}
	}

	memcpy(currentCave, tempCave, sizeof(tempCave));
}

//Generates a cave with no inaccessible areas, no non-border connected occupied cells and smoothed.
void generateCave() {
	fillPercentage = (rand() % 20) + 40; //Range: 40 -> 60.
	noiseScale = (rand() % 90) + 10; //Range: 10 -> 100.
	randomiseCave(); //Uses simplex noise to create a random cave.
	smoothCave(25); //Uses cellular automata to smooth the cave cells.
	startCell = findStartCell(); //Finds an appropraite starting location.
	fillInaccessibleAreas(startCell); //Removes inaccessible free cells.
	removeNonBorderOccupiedAreas(); //Removes occupied cells not connected to the cave border.

	//###
	vector<vector<int>> caveVector;
	for (int i = 0; i < caveWidth; i++) {
		vector<int> caveColumn;
		for (int j = 0; j < caveHeight; j++) {
			caveColumn.push_back(currentCave[i][j]);
		}
		caveVector.push_back(caveColumn);
	}

	Drone::setParams(caveWidth, caveHeight, caveVector);
	droneA.setPosition(startCell.x, startCell.y);
}

/*@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@*/

//Displays the control text at the top-left of the window.
void displayControls() {
	float textColour[3] = {1.0f, 1.0f, 1.0f};
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 30, 0.15f, (char *)"Controls:", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 60, 0.15f, (char *)"'q' - Quit", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 90, 0.15f, (char *)"'['/']' - Zoom In/Out:", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 120, 0.15f, (char *)"'n' - Scale Noise Up", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 150, 0.15f, (char *)"'m' - Scale Noise Down", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 180, 0.15f, (char *)"' ' - Generate Cave", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 210, 0.15f, (char *)"'k' - Decrease Fill Percentage", textColour);
	Draw::drawText(10, glutGet(GLUT_WINDOW_HEIGHT) - 240, 0.15f, (char *)"'l' - Increase Fill Percentage", textColour);
}

//Draws the cave structure, features no smoothing.
void renderCaveNormal() {
	//Colours.
	float caveFaceColour[3] = {0.2f, 0.1f, 0.0f};
	float caveDepthColour[3] = {0.2f, 0.1f, 0.05f};

	for (int i = 0; i < caveWidth; i++) {
		for (int j = 0; j < caveHeight; j++) {
			if (currentCave[i][j] == Occupied) {
				glPushMatrix();
				//Translate to cell position.
				glTranslatef((float)i, (float)j, 0);

				glColor3fv(caveFaceColour);
				if (currentCave[i][j] == Occupied) {
					glBegin(GL_TRIANGLE_STRIP);
					glNormal3f(0.0f, 0.0f, 1.0f);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(0.5f, 0.5f, 0);
					glEnd();
				}

				glColor3fv(caveDepthColour);
				if (i - 1 >= 0 && currentCave[i-1][j] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(-1.0f, 0.0f, 0.0f);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(-0.5f, -0.5f, depth);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(-0.5f, 0.5f, depth);
					glEnd();
				}
				if (i + 1 < caveWidth && currentCave[i+1][j] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(1.0f, 0.0f, 0.0f);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(0.5f, -0.5f, depth);
					glVertex3f(0.5f, 0.5f, 0);
					glVertex3f(0.5f, 0.5f, depth);
					glEnd();
				}
				if (j - 1 >= 0 && currentCave[i][j-1] == Free) {
					glBegin(GL_QUAD_STRIP);
					glNormal3f(0.0f, -1.0f, 0.0f);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(-0.5f, -0.5f, depth);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(0.5f, -0.5f, depth);
					glEnd();
				}
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

	//Colours.
	float caveFaceColour[3] = {0.2f, 0.1f, 0.0f};
	float caveDepthColour[3] = {0.2f, 0.1f, 0.05f};

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
	for (int i = 0; i < caveWidth - 1; i++) {
		for (int j = 0; j < caveHeight - 1; j++) {
			glPushMatrix();
			//Translate to cell position.
			glTranslatef((float)i, (float)j, 0);
			glColor3fv(caveFaceColour);

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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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
					glColor3fv(caveDepthColour);
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

//###
void renderDrone() {
	//Draws a drone at the starting location. //###
	Draw::drawDrone((float)startCell.x, (float)startCell.y, depth, 5.0f);
}


/*@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@~#~@*/

void idle() {
	//usleep(2500); // in microseconds
	//glutPostRedisplay();
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//###
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(cameraFOV, (GLdouble)glutGet(GLUT_WINDOW_WIDTH) / (GLdouble)glutGet(GLUT_WINDOW_HEIGHT), 0.1f, 250.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Eye Position, Reference Point, Up Vector.
	gluLookAt(cameraPanX, cameraPanY, 25, cameraPanX, cameraPanY, 0, 0, 1, 0);
	setLight(globalLight);

	//###DEBUG.
	cout << " + Camera Pan: (" << cameraPanX << ":" << cameraPanY << ")" << endl;
	cout << " + FOV: " << cameraFOV << endl;
	cout << " + Seed: " << noiseOffsetX << ":" << noiseOffsetY << " ~ Scale: " << noiseScale << endl;
	cout << " + Fill Percentage: " << fillPercentage << "%" << endl;
	cout << "[===================================================]" << endl;

	glPushMatrix();
	glEnable(GL_LIGHTING);

	//Draws Cave Background then Border then finally the cave structure.
	Draw::drawBackground(depth, caveWidth, caveHeight);
	Draw::drawBorder(depth, caveWidth, caveHeight);
	caveSmooth ? renderCaveSmooth() : renderCaveNormal();
	renderDrone();

	glPopMatrix();
	displayControls();
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
		case 'q': exit(1); break;
		//Zoom Out.
		case ']': cameraFOV = (cameraFOV <= 25.0f) ? 25.0f : cameraFOV - 2.5f; break;
		//Zoom In.
		case '[': cameraFOV = (cameraFOV >= 170.0f) ? 170.0f : cameraFOV + 2.5f; break;
		//###Noise Scale Up.
		case 'n': noiseScale += 1.0f; randomiseCave(); break;
		//###Noise Scale Down.
		case 'm': noiseScale -= 1.0f; randomiseCave(); break;
		//###Fill Percentage Decrease.
		case 'k': fillPercentage = (fillPercentage <= 0) ? 0 : fillPercentage - 1; randomiseCave(); break;
		//###Fill Percentage Increase.
		case 'l': fillPercentage = (fillPercentage >= 100) ? 100 : fillPercentage + 1; randomiseCave(); break;
		//Generates an improved cave.
		case ' ': generateCave(); break;
		//Drone debug###
		case 'p': droneA.sense(); break;
		//###Smoothing.
		case 't': caveSmooth = !caveSmooth; break;
	}
	glutPostRedisplay();
}

//Special Key event functions.
void specialKeyInput(int key, int x, int y) {
	switch (key) {
		//Pan Up.
		case GLUT_KEY_UP: cameraPanY += 0.2f * cameraFOV; break;
		//Pan Down.
		case GLUT_KEY_DOWN: cameraPanY -= 0.2f * cameraFOV; break;
		//Pan Left.
		case GLUT_KEY_LEFT: cameraPanX -= 0.2f * cameraFOV; break;
		//Pan Right.
		case GLUT_KEY_RIGHT: cameraPanX += 0.2f * cameraFOV; break;
	}
	glutPostRedisplay();
}

void init() {
	//Material.
	setMaterial(globalMaterial);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	//Light.
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//Misc.
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);
}

int main(int argc, char* argv[]) {

	//Random.
	srand(time(NULL));

	//Window Properties.
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(1000, 1000);
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
	generateCave();

	init();
	glutMainLoop();
	return 0;
}
