#include <vector>
#include <map>
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
  void recordConfiguration();
  void navigateToTarget();
private:
  //Member functions.
  void updateInternalMap();
  void findFrontierCells();
  pair<Cell,int> getBestFrontier();
  float getCellDistance(Cell start, Cell end);
  Cell getClosestCell();
  vector<Cell> astar(); //###
  int cellToInt(Cell src);
  Cell intToCell(int src);
  vector<Cell> getPath(map<int,int> previous, int current);
};
