#include <vector>
#include <map>
#include "DroneConfig.h"
#include "SenseCell.h"
using namespace std;
#pragma once

class Drone {
public:
  //Data Members.
  static int droneCount;
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
  void init(int id, float x, float y, string _name, int _frontierChoiceMethod);
  void setPosition(float x,  float y);
  void process();
  bool allowCommunication(int x);
  void combineMaps(vector<vector<int>> referenceMap, int droneID);
  vector<string> getStatistics();
  void addNearDrone(float x, float y);
private:
  //Data Members.
  static int caveWidth;
  static int caveHeight;
  static vector<vector<int>> cave;
  int id;
  int currentTimestep;
  map<int, int> frontierCells;
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
  pair<vector<SenseCell>,vector<SenseCell>> sense();
  vector<Cell> getPathToTarget(pair<Cell,int> target);
  void recordConfiguration();
  void updateInternalMap(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer);
  void findFrontierCells(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer);
  vector<pair<float,float>> getNearDroneWeightMap();
  pair<Cell,int> getBestFrontier(vector<pair<float,float>> nearDroneWeightMap);
  pair<Cell,int> getDebug(vector<pair<float,float>> nearDroneWeightMap); //###
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
