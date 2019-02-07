#include <vector>
#include <map>
#include "SenseCell.h"
using namespace std;
#pragma once

class Drone {
public:
  //Data Members.
  string name;
  float posX;
  float posY;
  float bearing;
  vector<vector<int>> internalMap;
  static float searchRadius;
  //Member Functions.
  static void setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave);
  void init(float x, float y, string _name);
  void setPosition(float x,  float y);
  pair<vector<SenseCell>,vector<SenseCell>> sense();
  void recordConfiguration();
  vector<Cell> getPathToTarget(pair<Cell,int> target);
  void process();
private:
  //Member functions.
  void updateInternalMap(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer);
  void findFrontierCells(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer);
  pair<Cell,int> getBestFrontier();
  pair<Cell,int> getNearestFrontier();
  void getLatestFrontier();
  float getCellManhattanDist(Cell start, Cell end);
  float getCellEuclideanDist(Cell start, Cell end);
  Cell getClosestCell(float x, float y);
  vector<Cell> searchAStar(Cell start, Cell dest);
  int cellToInt(Cell src);
  Cell intToCell(int src);
  vector<Cell> getAStarPath(map<int,int> previous, int current);
};
