#include <GL/glut.h>
#include <string.h>
#include "Draw.h"

//Draw Background.
void Draw::drawBackground(float depth, float caveWidth, float caveHeight) {
	glColor3f(0.175f, 0.075f, 0.0f);
	glPushMatrix();
	glTranslatef(-0.5f, -0.5f, 0);
	glBegin(GL_POLYGON);
	glVertex3f(0, 0, depth);
	glVertex3f(caveWidth, 0, depth);
	glVertex3f(caveWidth, caveHeight, depth);
	glVertex3f(0, caveHeight, depth);
	glEnd();
	glPopMatrix();
}

//Draws the border polygons.
void Draw::drawBorder(float depth, float caveWidth, float caveHeight) {
	glColor3f(0.3f, 0.2f, 0.3f);
	glPushMatrix();
	glTranslatef(-0.5f, -0.5f, 0.0f);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, depth);
	glVertex3f(caveWidth, 0, 0);
	glVertex3f(caveWidth, 0, depth);
	glVertex3f(caveWidth, caveHeight, 0);
	glVertex3f(caveWidth, caveHeight, depth);
	glVertex3f(0, caveHeight, 0);
	glVertex3f(0, caveHeight, depth);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, depth);
	glEnd();
	glPopMatrix();
}

//Draws text onto the screen with a given colour at a given position and scale.
void Draw::drawText(int x, int y, float scale, char* text, float* textColour) {
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
void Draw::drawDrone(float x, float y, float depth) {
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glColor3f(1.0f, 0.0f, 0.0f); //Red.
	glTranslatef(x, y, 0); //Move the drone to its current position.

	//Drone Object.
	glBegin(GL_TRIANGLES);
	glVertex3f(-0.2f, -0.2f, depth / 2.0f);
	glVertex3f(0.2f, -0.2f, depth / 2.0f);
	glVertex3f(0, 0.2f, depth / 2.0f);
	glEnd();

	//Bounding Box.
	drawDroneBoundingBox(depth + 0.01f);

	glEnable(GL_LIGHTING);
	glPopMatrix();
}

void Draw::drawDroneBoundingBox(float depth) {
	float boxNeg = -0.49f;
	float boxPos = 0.49f;
	glColor3f(0.0f, 0.0f, 1.0f);
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
