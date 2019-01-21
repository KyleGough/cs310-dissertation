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
#include "SimplexNoise.h"
using namespace std;

const int caveWidth = 250; //Number of cells making the width of the cave.
const int caveHeight = 250; //Number of cells making the height of the cave.
const int border = 3; //Padding of the cave border on the x-axis.

//Generation Parameters.
const int smoothIterations = 10;
const int fillPercentage = 50; //Percentage of the randomised environment that will be filled.
const int birthThreshold = 4;
const int deathThreshold = 4;
const int deathChance = 75;
const int birthChance = 100;

int currentCave[caveHeight][caveWidth];
int nextCave[caveHeight][caveWidth];

//Cave 1: FP: 50, BT: 4, DT: 4, DC: 75, BC: 100, Iter: 3.

//Camera.
float cameraZoom = 0.33f;
float cameraPanX = 0.0f;
float cameraPanY = 0.0f;
float cameraFOV = 40.0f; //Field of View.

//Using a random number uses the chance to threshold the number.
bool thresholdRandom(int chance) {
	return rand() % 100 < chance;
}

void randomiseCave() {

	for (int y = 0; y < caveHeight; y++) { //For each column in the cave.
		for (int x = 0; x < caveWidth; x++) { //For each row in the cave.
			if (x < border || x > caveWidth - border - 1 || y < border || y > caveHeight - border - 1) {
				currentCave[y][x] = 0; //Cave border.
				nextCave[y][x] = 0;
			}
			else {
				float noise = SimplexNoise::noise(x, y);   // Get the noise value for the coordinate
				currentCave[y][x] = (noise <= 0.0f) ? 0 : 1;
				//###currentCave[y][x] = thresholdRandom(fillPercentage) ? 0 : 1; //Cave body.
			}
		}
	}
}


int getNeighbourCount(int x, int y) {
	int count = 0;
	for (int i = x - 1; i <= x + 1; i++) {
		for (int j = y - 1; j <= y + 1; j++) {
			if (!(i == x && j == y) && currentCave[j][i] == 1) {
				count++;
			}
		}
	}
	return count;
}

void smoothCave() {
	for (int y = border; y < caveHeight - border; y++) {
		for (int x = border; x < caveWidth - border; x++) {
			int neighbours = getNeighbourCount(x, y);
			if (neighbours > birthThreshold && thresholdRandom(birthChance)) {
				nextCave[y][x] = 1; //Cell is born.
			}
			else if (neighbours < deathThreshold && thresholdRandom(deathChance)) {
				nextCave[y][x] = 0; //Cell dies.
			}
			else {
				nextCave[y][x] = currentCave[y][x]; //Maintains the cell state.
			}
		}
	}
}


void idle() {
	//usleep(2500); // in microseconds
	//glutPostRedisplay();
}


void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Eye Position, Reference Point, Up Vector.
	gluLookAt(cameraPanX, cameraPanY, 25, cameraPanX, cameraPanY, 0, 0, 1, 0);

	//###DEBUG.
	std::cout << "Pan_x: " << cameraPanX << " ~ Pan_y: " << cameraPanY << " ~ FOV: " << cameraFOV << std::endl;

	glPushMatrix();
	glDisable(GL_LIGHTING);

	float depth = -1.0f; //###

	//Draw Background.
	glColor3f(0.3f, 0.2f, 0.3f);
	glBegin(GL_POLYGON);
	glVertex3f(0, 0, depth);
	glVertex3f(caveWidth, 0, depth);
	glVertex3f(caveWidth, caveHeight, depth);
	glVertex3f(0, caveHeight, depth);
	glEnd();

	//Draw Cave.
	glColor3f(1.0f, 1.0f, 1.0f);

	for (int i = 0; i < caveWidth; i++) { //For each cave column.
		for (int j = 0; j < caveHeight; j++) { //For each cave row.
			if (currentCave[j][i] == 0) { //If cell is free.
				glPushMatrix();
				//Translate here.
				glTranslatef((float)i, (float)j, 0);

				//Camera-Viewing polygon face.
				glColor3f(0.3f, 0.2f, 0.4f);
				glBegin(GL_POLYGON);
				glVertex3f(0.5f, 0.5f, 0);
				glVertex3f(0.5f, -0.5f, 0);
				glVertex3f(-0.5f, -0.5f, 0);
				glVertex3f(-0.5f, 0.5f, 0);
				glEnd();

				glColor3f(0.3f, 0.3f, 0.3f);
				//Depth Face: Top.
				if (currentCave[j+1][i] == 1) {
					glBegin(GL_POLYGON);
					glVertex3f(0.5f, 0.5f, 0);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(-0.5f, 0.5f, depth);
					glVertex3f(0.5f, 0.5f, depth);
					glEnd();
				}
				//Depth Face: Left.
				if (currentCave[j][i-1] == 1) {
					glBegin(GL_POLYGON);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(-0.5f, 0.5f, 0);
					glVertex3f(-0.5f, 0.5f, depth);
					glVertex3f(-0.5f, -0.5f, depth);
					glEnd();
				}
				//Depth Face: Bottom.
				if (currentCave[j-1][i] == 1) {
					glBegin(GL_POLYGON);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(-0.5f, -0.5f, 0);
					glVertex3f(-0.5f, -0.5f, depth);
					glVertex3f(0.5f, -0.5f, depth);
					glEnd();
				}
				//Depth Face: Right.
				if (currentCave[j][i+1] == 1) {
					glBegin(GL_POLYGON);
					glVertex3f(0.5f, -0.5f, 0);
					glVertex3f(0.5f, 0.5f, 0);
					glVertex3f(0.5f, 0.5f, depth);
					glVertex3f(0.5f, -0.5f, depth);
					glEnd();
				}

				glPopMatrix();
			}
		}
	}


	glPopMatrix();
	glutSwapBuffers();
}

void reshape(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(cameraFOV, (GLdouble)w / (GLdouble)h, 0.1f, 250.0f);
	//###glOrtho(-20, 20, -20, 20, -40, 40);
}

//Mouse event functions.
void mouseInput(int button, int state, int x, int y) {
	switch (button) {
		//Scroll Up / Zoom Out.
		case 3: cameraFOV = (cameraFOV <= 25.0f) ? 25.0f : cameraFOV - 1.0f; break;
		//Scroll Down / Zoom In.
		case 4: cameraFOV = (cameraFOV >= 150.0f) ? 150.0f : cameraFOV + 1.0f; break;
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
		case '[': cameraFOV = (cameraFOV <= 25.0f) ? 25.0f : cameraFOV - 1.0f; cameraPanX *= cameraFOV; break; //###
		//Zoom In.
		case ']': cameraFOV = (cameraFOV >= 150.0f) ? 150.0f : cameraFOV + 1.0f; cameraPanX *= cameraFOV; break; //###
		//Smooth 1 Iteration.
		case ' ':
			smoothCave();
			memcpy(currentCave, nextCave, sizeof(currentCave));
			break;
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
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
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
	randomiseCave();
	//init();
	//glutMainLoop();
	/*for (int i = 0; i < smoothIterations; i++) {
		smoothCave();
		memcpy(currentCave, nextCave, sizeof(currentCave));
	}*/


	init();
	glutMainLoop();
	return 0;
}
