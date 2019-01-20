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

//###
//g++ -o main main.cpp SimplexNoise.cpp -I -L/usr/X11R6/lib -lglut -lGL -lGLU -lX11 -lm -lpng -lpthread

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
float cameraZoom = 1.0f;
float cameraPanX = 0.0f;
float cameraPanY = 0.0f;

//Using a random number uses the chance to threshold the number.
bool thresholdRandom(int chance) {
	return rand() % 100 < chance;
}

void randomiseCave() {

	for (int y = 0; y < caveHeight; y++) {
		for (int x = 0; x < caveWidth; x++) {
			if (x < border || x > caveWidth - border - 1 || y < border || y > caveHeight - border - 1) {
				currentCave[y][x] = 0; //Cave border.
				nextCave[y][x] = 0;
			}
			else {
				float z = 0.123f; // Define a float coordinate
    		float noise = SimplexNoise::noise(x, y);   // Get the noise value for the coordinate

				currentCave[y][x] = (noise <= 0) ? 0 : 1;
				std::cout << noise << std::endl;

				(void)noise;
				//currentCave[y][x] = SimplexNoise::noise(x, y);
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
				nextCave[y][x] = 1;
			}
			else if (neighbours < deathThreshold && thresholdRandom(deathChance)) {
				nextCave[y][x] = 0;
			}
			else {
				nextCave[y][x] = currentCave[y][x];
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

	glScalef(cameraZoom, cameraZoom, 1.0f);
  glTranslatef(cameraPanX, cameraPanY, 0.0f);

	//Eye Position, Reference Point, Up Vector.
	//gluLookAt(cameraX, cameraY, 10, cameraX, cameraY, 0, 0, 1, 0);

	std::cout << cameraPanX << " : " << cameraPanY << " : " << cameraZoom << std::endl;

	glPushMatrix();
	glScalef(cameraZoom, cameraZoom, 1.0f);

	glDisable(GL_LIGHTING);
	glColor3f(1.0f, 1.0f, 1.0f);

	for (int i = 0; i < caveWidth; i++) {
		for (int j = 0; j < caveHeight; j++) {
			if (currentCave[j][i] == 1) {
				glPushMatrix();
				//Translate here.
				glTranslatef((float)i, (float)j, 0);
				glBegin(GL_POLYGON);
				glVertex3f(0.5f, 0.5f, 0);
				glVertex3f(0.5f, -0.5f, 0);
				glVertex3f(-0.5f, -0.5f, 0);
				glVertex3f(-0.5f, 0.5f, 0);
				glEnd();
				glPopMatrix();
			}
		}
	}


	glPopMatrix();
	glutSwapBuffers();
}

//Mouse event functions.
void mouseInput(int button, int state, int x, int y) {
	switch (button) {
		//Scroll Up / Zoom Out.
		case 3: cameraZoom = (cameraZoom >= 1.0f) ? 1.0f : cameraZoom *= 1.05f; cameraPanX *= cameraZoom; break;
		//Scroll Down / Zoom In.
		case 4: cameraZoom = (cameraZoom <= 0.25f) ? 0.25f : cameraZoom /= 1.05f; cameraPanX *= cameraZoom; break;
	}
	glutPostRedisplay();
}

//Keyboard event functions.
void keyboardInput(unsigned char key, int, int) {
	switch (key) {
		//Exits the program.
		case 'q': exit(1); break;
		//Zoom Out.
		case '[': cameraZoom = (cameraZoom >= 1.0f) ? 1.0f : cameraZoom *= 1.05f; cameraPanX *= cameraZoom; break;
		//Zoom In.
		case ']': cameraZoom = (cameraZoom <= 0.25f) ? 0.25f : cameraZoom /= 1.05f; cameraPanX *= cameraZoom; break;
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
		case GLUT_KEY_UP:
			cameraPanY += 1.0f / cameraZoom;
			break;
		case GLUT_KEY_DOWN:
			cameraPanY -= 1.0f / cameraZoom;
			break;
		case GLUT_KEY_LEFT:
			cameraPanX -= 1.0f / cameraZoom;
			break;
		case GLUT_KEY_RIGHT:
			cameraPanX += 1.0f / cameraZoom;
			break;
	}
	glutPostRedisplay();
}

void reshape(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//gluPerspective(fov, (GLdouble)w / (GLdouble)h, 0.1f, 250.0f);
	glOrtho(-20, 20, -20, 20, -40, 40);
}

void init() {
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
}

int main(int argc, char* argv[]) {

	//Random.
	srand(time(NULL));

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
