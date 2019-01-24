#pragma once

class Draw {
public:
  static void drawBackground(float depth, float caveWidth, float caveHeight);
  static void drawBorder(float depth, float caveWidth, float caveHeight);
  static void drawText(int x, int y, float scale, char* text, float* textColour);
};