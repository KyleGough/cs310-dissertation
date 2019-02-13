#include <vector>
#include <map>
#include "DroneConfig.h"
#include "SenseCell.h"
using namespace std;
#pragma once

class Drone {
public:
  //Data Members.
  static float searchRadius;
  static float communicationRadius;
  static int communicationTimeBuffer;
  string name;
  float posX;
  float posY;
  float bearing;
  bool complete;
  vector<vector<int>> internalMap;
  vector<DroneConfig> pathList;
  //Member Functions.
  static void setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave);
  void init(float x, float y, string _name, int _frontierChoiceMethod, int droneCount);
  void setPosition(float x,  float y);
  pair<vector<SenseCell>,vector<SenseCell>> sense();
  void recordConfiguration();
  vector<Cell> getPathToTarget(pair<Cell,int> target);
  void process();
  bool allowCommunication(int x);
  void combineMaps(vector<vector<int>> targetMap);
private:
  //Data Members.
  map<int, int> frontierCells;
  int currentTimestep;
  pair<Cell,int> currentTarget;
  vector<Cell> targetPath;
  float totalTravelled;
  int freeCount;
  int occupiedCount;
  int commFreeCount;
  int commOccupiedCount;
  int frontierChoiceMethod;
  vector<int> lastCommunication;
  //Member functions.
  void updateInternalMap(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer);
  void findFrontierCells(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer);
  pair<Cell,int> getBestFrontier();
  pair<Cell,int> getNearestFrontier();
  pair<Cell,int> getLatestFrontier();
  float getDistToDrone(Cell dest);
  float getDistToDrone(int x, int y);
  float getCellManhattanDist(Cell start, Cell end);
  float getCellEuclideanDist(Cell start, Cell end);
  Cell getClosestCell(float x, float y);
  vector<Cell> searchAStar(Cell start, Cell dest);
  int cellToInt(Cell src);
  Cell intToCell(int src);
  vector<Cell> getAStarPath(map<int,int> previous, int current);
  void outputStatistics();
};
