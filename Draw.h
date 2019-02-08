#pragma once
#include <vector>
#include "SenseCell.h"
using namespace std;

class Draw {
public:
  static void drawBackground(float depth, float caveWidth, float caveHeight);
  static void drawBorder(float depth, float caveWidth, float caveHeight);
  static void drawText(int x, int y, float scale, const char* text, float* textColour);
  static void drawDrone(float x, float y, float depth, float searchRadius, string name);
  static void drawDiscoveredCells(int caveWidth, int caveHeight, float depth, vector<vector<int>> cave, float colours[][4]);
private:
  static void drawDroneBoundingBox(float depth);
  static void drawDroneSearchingRange(float searchRadius, float depth);
};
