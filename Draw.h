#pragma once
#include <vector>
#include "SenseCell.h"
#include "DroneConfig.h"
using namespace std;

class Draw {
public:
  static void drawBackground(float depth, float caveWidth, float caveHeight);
  static void drawBorder(float depth, float caveWidth, float caveHeight);
  static void drawText(int x, int y, float scale, const char* text, const float* textColour);
  static void drawDrone(float x, float y, float depth, float searchRadius, string name, float bearing);
  static void drawDiscoveredCells(int caveWidth, int caveHeight, float depth, vector<vector<int>> cave, float colours[][4]);
  static void drawDronePath(vector<DroneConfig> pathList, float depth, float radius, const float mask[3]);
private:
  static void drawDroneBoundingBox(float depth);
  static void drawDroneSearchingRange(float searchRadius, float depth);
  static void drawCircle(float radius, size_t segments, float depth);
};
