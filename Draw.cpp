#include <GL/glut.h>
#include <string.h>
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
	size_t len = strlen(text);
	for (size_t i = 0; i < len; i++) {
			glutStrokeCharacter(GLUT_STROKE_ROMAN, text[i]);
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}
