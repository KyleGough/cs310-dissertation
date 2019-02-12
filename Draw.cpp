#define _USE_MATH_DEFINES
#include <GL/glut.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include "MapCell.h"
#include "Draw.h"
#include "DroneConfig.h"
#include "Cell.h" //###
using namespace std;

//Draw Background.
void Draw::drawBackground(float depth, float caveWidth, float caveHeight) {

	//Components of border coordinates.
	const float minX = -0.5f;
	const float minY = -0.5f;
	const float maxX = caveWidth - 1.5f;
	const float maxY = caveHeight - 1.5f;

	//Draws the background.
	glColor4f(0.175f, 0.075f, 0.0f, 1.0f);
	glPushMatrix();
	glBegin(GL_POLYGON);
	glVertex3f(minX, minY, depth);
	glVertex3f(maxX, minY, depth);
	glVertex3f(maxX, maxY, depth);
	glVertex3f(minX, maxY, depth);
	glEnd();
	glPopMatrix();
}

//Draws the border polygons.
void Draw::drawBorder(float depth, float caveWidth, float caveHeight) {

	//Components of border coordinates.
	const float minX = -0.5f;
	const float minY = -0.5f;
	const float maxX = caveWidth - 1.5f;
	const float maxY = caveHeight - 1.5f;
	const float borderBuffer = 5.0f;

	//Draws the border around the cave edges orthogonal to the camera.
	glBegin(GL_QUAD_STRIP);
	glVertex3f(minX - borderBuffer, minY - borderBuffer + 0.5f, 0);
	glVertex3f(minX - borderBuffer, minY - borderBuffer + 0.5f, depth);
	glVertex3f(maxX + borderBuffer, minY - borderBuffer + 0.5f, 0);
	glVertex3f(maxX + borderBuffer, minY - borderBuffer + 0.5f, depth);
	glVertex3f(maxX + borderBuffer, maxY + borderBuffer - 0.5f, 0);
	glVertex3f(maxX + borderBuffer, maxY + borderBuffer - 0.5f, depth);
	glVertex3f(minX - borderBuffer, maxY + borderBuffer - 0.5f, 0);
	glVertex3f(minX - borderBuffer, maxY + borderBuffer - 0.5f, depth);
	glVertex3f(minX - borderBuffer, minY - borderBuffer + 0.5f, 0);
	glVertex3f(minX - borderBuffer, minY - borderBuffer + 0.5f, depth);
	glEnd();

	//Draws the border around the cave edges facing the camera.
	glColor4f(0.2f, 0.1f, 0.0f, 1.0f);
	glBegin(GL_POLYGON);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(minX - borderBuffer, minY + 0.5f, 0);
	glVertex3f(maxX + borderBuffer, minY + 0.5f, 0);
	glVertex3f(maxX + borderBuffer, minY - borderBuffer + 0.5f, 0);
	glVertex3f(minX - borderBuffer, minY - borderBuffer + 0.5f, 0);
	glEnd();
	glBegin(GL_POLYGON);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(minX - borderBuffer, maxY - 0.5f, 0);
	glVertex3f(maxX + borderBuffer, maxY - 0.5f, 0);
	glVertex3f(maxX + borderBuffer, maxY + borderBuffer - 0.5f, 0);
	glVertex3f(minX - borderBuffer, maxY + borderBuffer - 0.5f, 0);
	glEnd();
	glBegin(GL_POLYGON);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(minX + 0.5f, minY, 0);
	glVertex3f(minX - borderBuffer, minY, 0);
	glVertex3f(minX - borderBuffer, maxY, 0);
	glVertex3f(minX + 0.5f, maxY, 0);
	glEnd();
	glBegin(GL_POLYGON);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(maxX - 0.5f, minY, 0);
	glVertex3f(maxX + borderBuffer, minY, 0);
	glVertex3f(maxX + borderBuffer, maxY, 0);
	glVertex3f(maxX - 0.5f, maxY, 0);
	glEnd();
}

//Draws text onto the screen with a given colour at a given position and scale.
void Draw::drawText(int x, int y, float scale, const char* text, const float* textColour) {
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT));
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
  glDisable(GL_LIGHTING);
	glColor3fv(textColour);
	glTranslatef(x, y, 0.0f);
	glScalef(scale, scale, 1.0f);
	glLineWidth(2.0f);
	size_t len = strlen(text);
	for (size_t i = 0; i < len; i++) {
			glutStrokeCharacter(GLUT_STROKE_ROMAN, text[i]);
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

//Draws a drone at a given position.
void Draw::drawDrone(float x, float y, float depth, float searchRadius, string name, float bearing) {
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f); //Red.
	glTranslatef(x, y, 0); //Move the drone to its current position.

	//Draws the name of the drone.
	glPushMatrix();
	glLineWidth(2.0f);
	size_t len = name.size();
	glColor3f(1.0f, 1.0f, 1.0f);
	glTranslatef(-(len / 2.0f), 1.0f, 1.0f);
	glScalef(0.020f, 0.020f, 0.0f);
	for (size_t i = 0; i < len; i++) {
			glutStrokeCharacter(GLUT_STROKE_ROMAN, name[i]);
	}
	glPopMatrix();

	//Drone Object.
	const float gap = 0.15f;
	glPushMatrix();
	glTranslatef(-gap, -gap, 0.0f);
	drawCircle(0.1f, 32, depth / 2.0f);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(gap, -gap, 0.0f);
	drawCircle(0.1f, 32, depth / 2.0f);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(gap, gap, 0.0f);
	drawCircle(0.1f, 32, depth / 2.0f);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(-gap, gap, 0.0f);
	drawCircle(0.1f, 32, depth / 2.0f);
	glPopMatrix();
	glBegin(GL_POLYGON);
	glVertex3f(0.10f, 0.10f, depth / 2.0f);
	glVertex3f(-0.10f, 0.10f, depth / 2.0f);
	glVertex3f(-0.10f, -0.10f, depth / 2.0f);
	glVertex3f(0.10f, -0.10f, depth / 2.0f);
	glEnd();

	//Bounding Box.
	drawDroneBoundingBox(depth + 0.01f);

	//Search Range.
	drawDroneSearchingRange(searchRadius, depth / 2.0f);

	glEnable(GL_LIGHTING);
	glPopMatrix();
}

//Draws the bounding box of the drone.
void Draw::drawDroneBoundingBox(float depth) {
	float boxNeg = -0.49f;
	float boxPos = 0.49f;
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glLineWidth(3.0f);
	glEnable(GL_LINE_SMOOTH);
	glBegin(GL_LINE_STRIP);
	glVertex3f(boxNeg, boxNeg, 0); //Front Bottom-Left.
	glVertex3f(boxPos, boxNeg, 0); //Front Bottom-Right.
	glVertex3f(boxPos, boxPos, 0); //Front Top-Right.
	glVertex3f(boxNeg, boxPos, 0); //Front Top-Left.
	glVertex3f(boxNeg, boxNeg, 0); //Front Bottom-Left.
	glVertex3f(boxNeg, -0.5f, depth); //Back Bottom-Left.
	glVertex3f(boxPos, boxNeg, depth); //Back Bottom-Right.
	glVertex3f(boxPos, boxPos, depth); //Back Top-Right.
	glVertex3f(boxNeg, boxPos, depth); //Back Top-Left.
	glVertex3f(boxNeg, -0.5f, depth); //Back Bottom-Left.
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(boxPos, boxPos, 0); //Front Top-Right.
	glVertex3f(boxPos, boxPos, depth); //Back Top-Right.
	glVertex3f(boxNeg, boxPos, 0); //Front Top-Left.
	glVertex3f(boxNeg, boxPos, depth); //Back Top-Left.
	glVertex3f(boxPos, boxNeg, 0); //Front Bottom-Right.
	glVertex3f(boxPos, boxNeg, depth); //Back Bottom-Right.
	glEnd();
	glDisable(GL_LINE_SMOOTH);
}

//Draws a circle indicating the search range of the drone.
void Draw::drawDroneSearchingRange(float searchRadius, float depth) {
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_LINE_SMOOTH);
	glBegin(GL_LINE_LOOP);

	//Draws a segmented circle.
	drawCircle(searchRadius, 32, depth / 2.0f);

	glEnd();
	glDisable(GL_LINE_SMOOTH);
}

//Draws discovered cave cells in a specific colour to indicate type.
void Draw::drawDiscoveredCells(int caveWidth, int caveHeight, float depth, vector<vector<int>> cave, float colours[][4]) {
	//Iterates over each cell in the cave.
	for (size_t i = 0; i < caveWidth; i++) {
		for (size_t j = 0; j < caveHeight; j++) {
			float d;
			//Skip the cell if it is unknown.
			if (cave[i][j] == Unknown) { continue; }
			glPushMatrix();
			switch (cave[i][j]) {
				case Free:
					glColor4fv(colours[0]);
					d = depth;
					break;
				case Occupied:
					glColor4fv(colours[1]);
					d = 0.0f;
					break;
				case Frontier:
					glColor4fv(colours[2]);
					d = depth;
					break;
			}
			glTranslatef((float)i, (float)j, 0.0f);
			glBegin(GL_TRIANGLE_STRIP);
			glVertex3f(-0.5f, -0.5f, d + 0.01f);
			glVertex3f(0.5f, -0.5f, d + 0.01f);
			glVertex3f(-0.5f, 0.5f, d + 0.01f);
			glVertex3f(0.5f, 0.5f, d + 0.01f);
			glEnd();
			glPopMatrix();
		}
	}
}

//Draws a coloured path depicting communicatio between two drones.
void Draw::drawCommunication(vector<Cell> cellList, float radius, float depth) {

	for (auto& cell : cellList) {
		glPushMatrix();
		glTranslatef(cell.x, cell.y, 0.0f);
		glBegin(GL_QUADS);
		glVertex3f(radius, radius, depth);
		glVertex3f(-radius, radius, depth);
		glVertex3f(-radius, -radius, depth);
		glVertex3f(radius, -radius, depth);
		glEnd();
		glPopMatrix();
	}
}

//Draws a segmented circle.
void Draw::drawCircle(float radius, size_t segments, float depth) {
	glBegin(GL_LINE_LOOP);
	for (size_t i = 0; i < segments; i++) {
		float angle = 2.0f * M_PI * float(i) / float(segments);
		float x = radius * cos(angle);
		float y = radius * sin(angle);
		glVertex3f(x, y, depth);
	}
	glEnd();
}

//Draws a coloured path of the drone's previous positions.
void Draw::drawDronePath(vector<DroneConfig> pathList, float depth, float radius, const float mask[3]) {

	size_t size = pathList.size(); //Number of configurations.
	glDisable(GL_LIGHTING);

	//For each previous configuration.
	for (auto& config : pathList) {
		glPushMatrix();
		float intensity = config.timestep / (float)size; //Intensity based on timestep.
		glColor4f(intensity * mask[0], intensity * mask[1], intensity * mask[2], 0.75f); //Apply colour mask to intensity.
		glTranslatef(config.x, config.y, 0.0f);
		glBegin(GL_QUADS);
		glVertex3f(radius, radius, depth);
		glVertex3f(-radius, radius, depth);
		glVertex3f(-radius, -radius, depth);
		glVertex3f(radius, -radius, depth);
		glEnd();
		glPopMatrix();
	}

	glEnable(GL_LIGHTING);
}
