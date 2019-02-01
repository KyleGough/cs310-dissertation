#include <vector>
#include "SenseCell.h"
#include "QuadTree.h"
using namespace std;
#pragma once

class Drone {
public:
  //Data Members.
  string name;
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
  void init(float x, float y, string _name);
  void setPosition(float x,  float y);
  void sense();
private:
  //Member functions.
  void updateInternalMap();
  void findFrontierCells();
  Cell getBestFrontier();
  void recordConfiguration();
};
