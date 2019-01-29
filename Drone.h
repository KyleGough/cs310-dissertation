#include <vector>
#include "SenseCell.h"
using namespace std;
#pragma once

class Drone {
public:
  //Data Members.
  float x;
  float y;
  float orientation;
  vector<SenseCell> freeCellBuffer;
  vector<SenseCell> occupiedCellBuffer;
  //Member Functions.
  static void setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave);
  void setPosition(int _x, int _y);
  void sense();
private:
  //Member functions.
  void updateInternalMap();
  void findFrontierCells();
};
