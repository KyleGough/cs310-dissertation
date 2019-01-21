#include <GL/glut.h>
#include "Draw.h"

//Draw Background.
void Draw::drawBackground(float depth, float caveWidth, float caveHeight) {
	glColor3f(0.3f, 0.2f, 0.3f);
	glBegin(GL_POLYGON);
	glVertex3f(-0.5f, -0.5f, depth);
	glVertex3f(caveWidth + 0.5f, -0.5f, depth);
	glVertex3f(caveWidth + 0.5f, caveHeight + 0.5f, depth);
	glVertex3f(-0.5f, caveHeight + 0.5f, depth);
	glEnd();
}

//Draws the border polygons.
void Draw::drawBorder(float depth, float caveWidth, float caveHeight) {
	glColor3f(0.3f, 0.2f, 0.3f);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-0.5f, -0.5f, 0);
	glVertex3f(-0.5f, -0.5f, depth);
	glVertex3f(caveWidth + 0.5f, -0.5f, 0);
	glVertex3f(caveWidth + 0.5f, -0.5f, depth);
	glVertex3f(caveWidth + 0.5f, caveHeight + 0.5f, 0);
	glVertex3f(caveWidth + 0.5f, caveHeight + 0.5f, depth);
	glVertex3f(-0.5f, caveHeight + 0.5f, 0);
	glVertex3f(-0.5f, caveHeight + 0.5f, depth);
	glVertex3f(-0.5f, -0.5f, 0);
	glVertex3f(-0.5f, -0.5f, depth);
	glEnd();
}
