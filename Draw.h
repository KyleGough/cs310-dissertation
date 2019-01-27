#pragma once

class Draw {
public:
  static void drawBackground(float depth, float caveWidth, float caveHeight);
  static void drawBorder(float depth, float caveWidth, float caveHeight);
  static void drawText(int x, int y, float scale, char* text, float* textColour);
  static void drawDrone(float x, float y, float depth, float searchRange);
private:
  static void drawDroneBoundingBox(float depth);
  static void drawDroneSearchingRange(float searchRange, float depth);
};
