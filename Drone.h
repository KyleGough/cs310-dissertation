#include <vector>
#include "SenseCell.h"
#include "QuadTree.h"
using namespace std;
#pragma once

class Drone {
public:
  //Data Members.
  float posX;
  float posY;
  float orientation;
  vector<SenseCell> freeCellBuffer;
  vector<SenseCell> occupiedCellBuffer;
  Quad quadCave;
  static float searchRange;
  //Member Functions.
  static void setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave);
  void init();
  void setPosition(int _x, int _y);
  void sense();
private:
  //Member functions.
  void updateInternalMap();
  void findFrontierCells();
};
