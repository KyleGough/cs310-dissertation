#include <vector>
using namespace std;
#pragma once

class Drone {
public:
  float x;
  float y;
  float orientation;
  static void setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave);
  void setPosition(int _x, int _y);
  void sense();
private:
};
