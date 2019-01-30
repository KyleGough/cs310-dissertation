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
  vector<vector<int>> internalMap;
  static float searchRange;
  //Member Functions.
  static void setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave);
  void init(int x, int y);
  void setPosition(int x, int y);
  void sense();
private:
  //Member functions.
  void updateInternalMap();
  void findFrontierCells();
};
