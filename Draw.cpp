#include <GL/glut.h>
#include "Draw.h"

//Draw Background.
void Draw::drawBackground(float depth, float caveWidth, float caveHeight) {
	glColor3f(0.3f, 0.2f, 0.3f);
	glBegin(GL_POLYGON);
	glVertex3f(0, 0, depth);
	glVertex3f(caveWidth, 0, depth);
	glVertex3f(caveWidth, caveHeight, depth);
	glVertex3f(0, caveHeight, depth);
	glEnd();
}

//Draws the border polygons.
void Draw::drawBorder(float depth, float caveWidth, float caveHeight) {
	glColor3f(0.3f, 0.2f, 0.3f);
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
}

/*###void Draw::drawCellDepth(float depth) {
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
}*/
