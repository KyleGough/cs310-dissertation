#include <iostream>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>
#include <algorithm>
#include <vector>
#include <algorithm>
#include <map>
#include <set>
#include <limits>
#include <queue>
#include "Cell.h"
#include "SenseCell.h"
#include "DroneConfig.h"
#include "MapCell.h"
#include "Drone.h"
using namespace std;


static constexpr float maxVelocity = 0.25f;
static constexpr float acceleration = 0.1f;
float Drone::searchRange = 10.0f; //### //Range of localised search.
static int caveWidth;
static int caveHeight;
static vector<vector<int>> cave;

string name; //Name of the drone. //###
float posX; //Current x position in the cave.
float posY; //Current y position in the cave.
float orientation; //Orientation: 0 -> Facing North.
//vector<SenseCell> freeCellBuffer; //List of free cells sensed from the last sense operation.
//vector<SenseCell> occupiedCellBuffer; //List of occupied cells sensed from the last sense operation.
vector<vector<int>> internalMap; //###
map<int, int> frontierCells; //Free cells that are adjacent to unknowns.
vector<DroneConfig> pathList; //List of drone configurations for each timestep. //###
int currentTimestep; //###
pair<Cell,int> currentTarget; //###
vector<Cell> targetPath; //###


//Less than comparison function for two SenseCell objects.
bool operator <(const SenseCell& a, const SenseCell& b) {
  return a.range < b.range;
}

//Sets static cave properties.
void Drone::setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave) {
  caveWidth = _caveWidth;
  caveHeight = _caveHeight;
  cave = _cave;
}

//Initalises the drone's starting position, name and internal map.
void Drone::init(float x, float y, string _name) {
  posX = x;
  posY = y;
  orientation = 0.0f;
  name = _name;
  currentTimestep = 0;
  currentTarget = make_pair(Cell(-1,-1), -1); //Unreachable target.
  frontierCells.clear();

  //Sets the internal map to all unknowns.
  internalMap.clear();
  for (size_t i = 0; i < caveWidth; i++) {
    vector<int> column;
    for (size_t j = 0; j < caveHeight; j++) {
      column.push_back(Unknown);
    }
    internalMap.push_back(column);
  }

  testinit(); //###

  recordConfiguration();
}

//Sets the drone's current position in the cave.
void Drone::setPosition(float x, float y) {
  posX = x;
  posY = y;
  //###
  //cout << " + [" << name << "]" << " - Pos: (" << posX << "," << posY << ")" << endl;
}

//Models the sensing of the immediate local environment.
pair<vector<SenseCell>,vector<SenseCell>> Drone::sense() {

  vector<SenseCell> candidates; //List of candidate cells.
  vector<SenseCell> freeCells; //List of found free cells.
  vector<SenseCell> occupiedCells; //List of found occupied cells.
  vector<SenseCell> checkCells; //List of cells to check.

  //For each cell in the bounding box of the search range.
  //Discards Out-of-bounds cells (e.g. i = -1).
  for (size_t i = max(0, (int)floor(posX - searchRange)); i <= min(caveWidth - 1, (int)ceil(posX + searchRange)); i++) {
    for (size_t j = max(0, (int)floor(posY - searchRange)); j <= min(caveHeight - 1, (int)ceil(posY + searchRange)); j++) {
      //Allows only cells in the range.
      float range = pow(pow(posX - (float)i, 2.0) + pow(posY - (float)j, 2.0), 0.5);
      if (range > searchRange) { continue; }
      //Push the candidate cell onto the vector.
      candidates.push_back(SenseCell(i,j,range));
    }
  }

  //Sorts the list of cells by distance to the drone in increasing order.
  sort(candidates.begin(), candidates.end());

  //Check to make sure you can't sense objects hidden behind something else.
  //ray from drone center to cell center.
  for (vector<SenseCell>::iterator dest = candidates.begin(); dest != candidates.end(); ++dest) {
    //If the cell range is 1 or less then immediately add it to the list.
    if (dest->range <= 1) {
      if (cave[dest->x][dest->y] == Free || cave[dest->x][dest->y] == Frontier) {
        freeCells.push_back(*dest);
      }
      else {
        occupiedCells.push_back(*dest);
        checkCells.push_back(*dest);
      }
      continue;
    }

    bool collisionDetected = false;

    //Obstacle in line of sight between drone position and destination cell check.
    for (vector<SenseCell>::iterator occupyCheck = checkCells.begin(); occupyCheck != checkCells.end(); ++occupyCheck) {
      //Ignore if the cell to check is free.
      if (cave[occupyCheck->x][occupyCheck->y] == Free) { continue; }

      float tx0 = (occupyCheck->x - 0.5f - posX) / (dest->x - posX);
      float tx1 = (occupyCheck->x + 0.5f - posX) / (dest->x - posX);
      float ty0 = (occupyCheck->y - 0.5f - posY) / (dest->y - posY);
      float ty1 = (occupyCheck->y + 0.5f - posY) / (dest->y - posY);

      if (tx0 >= 0 && tx0 <= 1) {
        float yCheck = posY + tx0 * (dest->y - posY);
        if (yCheck >= occupyCheck->y - 0.5f && yCheck <= occupyCheck->y + 0.5f) {
          collisionDetected = true;
          break;
        }
      }
      if (tx1 >= 0 && tx1 <= 1) {
        float yCheck = posY + tx1 * (dest->y - posY);
        if (yCheck >= occupyCheck->y - 0.5f && yCheck <= occupyCheck->y + 0.5f) {
          collisionDetected = true;
          break;
        }
      }
      if (ty0 >= 0 && ty0 <= 1) {
        float xCheck = posX + ty0 * (dest->x - posX);
        if (xCheck >= occupyCheck->x - 0.5f && xCheck <= occupyCheck->x + 0.5f) {
          collisionDetected = true;
          break;
        }
      }
      if (ty1 >= 0 && ty0 <= 1) {
        float xCheck = posX + ty1 * (dest->x - posX);
        if (xCheck >= occupyCheck->x - 0.5f && xCheck <= occupyCheck->x + 0.5f) {
          collisionDetected = true;
          break;
        }
      }
    }

    //If no collision detected then the destination cell is in line of sight from the drone's position.
    if (!collisionDetected) {
      if (cave[dest->x][dest->y] == Free || cave[dest->x][dest->y] == Frontier) {
        freeCells.push_back(*dest);
      }
      else {
        occupiedCells.push_back(*dest);
        checkCells.push_back(*dest);
      }
    }
    else {
      checkCells.push_back(*dest);
    }
  }

  return make_pair(freeCells, occupiedCells);
}

//Updates the internal map of the drone to include recently sensed free and occupied cells.
void Drone::updateInternalMap(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer) {

  //Adds all free cells to the internal map.
  for (vector<SenseCell>::iterator freeCell = freeCellBuffer.begin(); freeCell != freeCellBuffer.end(); ++freeCell) {
    int x = freeCell->x;
    int y = freeCell->y;
    if (internalMap[x][y] == Unknown) {
      internalMap[x][y] = Free;
    }
  }

  //Adds all occupied cells to the internal map.
  for (vector<SenseCell>::iterator occupyCell = occupiedCellBuffer.begin(); occupyCell != occupiedCellBuffer.end(); ++occupyCell) {
    int x = occupyCell->x;
    int y = occupyCell->y;
    if (internalMap[x][y] == Unknown) {
      internalMap[x][y] = Occupied;
    }
  }
}

//Recalculates the set of frontier cells in the internal map.
void Drone::findFrontierCells(vector<SenseCell> freeCellBuffer, vector<SenseCell> occupiedCellBuffer) {

  //List of cells to check if they are frontier cels.
  vector<Cell> frontierCheck;

  //Iterates through each newly sensed free cell.
  //If the cell itself or a neighbour is a frontier cell, add it to the check list and set it to free.
  for (vector<SenseCell>::iterator freeCell = freeCellBuffer.begin(); freeCell != freeCellBuffer.end(); ++freeCell) {
    int x = freeCell->x;
    int y = freeCell->y;
    int i = y * caveWidth + x; //Dictionary key for the cell mapped into 1D.
    if (internalMap[x][y] == Frontier) {
      internalMap[x][y] = Free;
      frontierCells.erase(i);
    }
    if (x - 1 >= 0 && internalMap[x-1][y] == Frontier) {
      internalMap[x-1][y] = Free;
      frontierCells.erase(i-1);
      frontierCheck.push_back(Cell(x-1,y));
    }
    if (x + 1 < caveWidth && internalMap[x+1][y] == Frontier) {
      internalMap[x+1][y] = Free;
      frontierCells.erase(i+1);
      frontierCheck.push_back(Cell(x+1,y));
    }
    if (y - 1 >= 0 && internalMap[x][y-1] == Frontier) {
      internalMap[x][y-1] = Free;
      frontierCells.erase(i-caveWidth);
      frontierCheck.push_back(Cell(x,y-1));
    }
    if (y + 1 < caveHeight && internalMap[x][y+1] == Frontier) {
      internalMap[x][y+1] = Free;
      frontierCells.erase(i+caveWidth);
      frontierCheck.push_back(Cell(x,y+1));
    }
    frontierCheck.push_back(Cell(x,y)); //###???
  }

  //Iterates through each newly sensed occupied cell.
  //If a neighbour is a frontier cell, add it to the check list and set it to free.
  for (vector<SenseCell>::iterator occupyCell = occupiedCellBuffer.begin(); occupyCell != occupiedCellBuffer.end(); ++occupyCell) {
    int x = occupyCell->x;
    int y = occupyCell->y;
    int i = y * caveWidth + x; //Dictionary key for the cell mapped into 1D.
    if (x - 1 >= 0 && internalMap[x-1][y] == Frontier) {
      internalMap[x-1][y] = Free;
      frontierCells.erase(i-1);
      frontierCheck.push_back(Cell(x-1,y));
    }
    if (x + 1 < caveWidth && internalMap[x+1][y] == Frontier) {
      internalMap[x+1][y] = Free;
      frontierCells.erase(i+1);
      frontierCheck.push_back(Cell(x+1,y));
    }
    if (y - 1 >= 0 && internalMap[x][y-1] == Frontier) {
      internalMap[x][y-1] = Free;
      frontierCells.erase(i-caveWidth);
      frontierCheck.push_back(Cell(x,y-1));
    }
    if (y + 1 < caveHeight && internalMap[x][y+1] == Frontier) {
      internalMap[x][y+1] = Free;
      frontierCells.erase(i+caveWidth);
      frontierCheck.push_back(Cell(x,y+1));
    }
  }

  //For each cell to check if it neighbours an unknown cell set it as a Frontier cell and add it to the list of frontiers.
  for (vector<Cell>::iterator frontierCell = frontierCheck.begin(); frontierCell != frontierCheck.end(); ++frontierCell) {
    int x = frontierCell->x;
    int y = frontierCell->y;
    int i = y * caveWidth + x; //Dictionary key for the cell mapped into 1D.
    if (x - 1 >= 0 && internalMap[x-1][y] == Unknown) {
      internalMap[x][y] = Frontier;
      frontierCells[i] = currentTimestep;
      continue;
    }
    if (x + 1 < caveWidth && internalMap[x+1][y] == Unknown) {
      internalMap[x][y] = Frontier;
      frontierCells[i] = currentTimestep;
      continue;
    }
    if (y - 1 >= 0 && internalMap[x][y-1] == Unknown) {
      internalMap[x][y] = Frontier;
      frontierCells[i] = currentTimestep;
      continue;
    }
    if (y + 1 < caveHeight && internalMap[x][y+1] == Unknown) {
      internalMap[x][y] = Frontier;
      frontierCells[i] = currentTimestep;
      continue;
    }
  }
}

//Finds the best frontier cell to navigate to.
pair<Cell,int> Drone::getBestFrontier() {

  float bestDist = numeric_limits<float>::max();
  Cell bestFrontier = Cell(0,0);
  int timestep = -1;

  //Iterates over every frontier cell to find the frontier nearest to the current drone position.
  for(auto& frontier : frontierCells) {
    int y = (int)frontier.first / caveWidth;
    int x = frontier.first % caveWidth;
    float dist = pow(pow(x - posX, 2.0f) + pow(y - posY, 2.0f), 0.5f);
    if (dist < bestDist) {
      bestDist = dist;
      bestFrontier = Cell(x,y);
      timestep = frontier.second;
    }
  }

  //###
  cout << "Best Frontier: [" << bestDist << "] - (" << bestFrontier.x << "," << bestFrontier.y << ")" << endl;
  cout << "Timestep: (" << timestep << ")" << endl;
  cout << "Frontier Count: " << frontierCells.size() << endl;

  return make_pair(bestFrontier, timestep);
}

//Adds the drone's current configuration to the path.
void Drone::recordConfiguration() {
  pathList.push_back(DroneConfig(currentTimestep, posX, posY, orientation));
  currentTimestep++;
}

//Gets the closest cell to the drone's current position.
Cell Drone::getClosestCell(float x, float y) {

  float minDist = 100; //Abitrary large number.
  Cell closestCell;

  for (int i = (int)floor(x); i <= (int)ceil(x); i++) {
    for (int j = (int)floor(y); j <= (int)ceil(y); j++) {
      float dist = pow(pow((float)i - x, 2.0f) + pow((float)j - y, 2.0f), 0.5f);
      if (dist < minDist) {
        minDist = dist;
        closestCell = Cell(i,j);
      }
    }
  }

  return closestCell;
}

//Uses A* and previously stored mapping of frontiers to find the path from the current position to the best frontier.
vector<Cell> Drone::getPathToTarget(pair<Cell,int> target) {
  int targetTimestep = target.second;
  Cell startPos = getClosestCell(posX, posY);

  //If the target frontier cell was sensed in the current timestep.
  if (targetTimestep == currentTimestep) {
    cout << "TARGET ON CURRENT TIMESTEP" << endl; //###
    vector<Cell> path = searchAStar(startPos, target.first); //Gets the path using A*.
    reverse(path.begin(), path.end()); //Reverses the path.
    return path;
  }
  //If the target frontier was sensed in a previous timestep.
  else {
    //Backtrack.
    cout << "BACKTRACK" << endl; //###
    Cell midPos;

    //Gets the position from where the target frontier was last observed from.
    for (vector<DroneConfig>::reverse_iterator config = pathList.rbegin(); config != pathList.rend(); ++config) {
      if (config->timestep == targetTimestep) {
        midPos = getClosestCell(config->x, config->y);
        break;
      }
    }

    vector<Cell> pathA = searchAStar(startPos, midPos);
    vector<Cell> pathB = searchAStar(midPos, target.first);
    reverse(pathA.begin(), pathA.end()); //Reverses path A.
    reverse(pathB.begin(), pathB.end()); //Reverses path B.
    pathB.erase(pathB.begin()); //Removes the first element which is present in both vectors.
    pathA.insert(pathA.end(), pathB.begin(), pathB.end()); //Concatenates the paths.
    return pathA;
  }
}

//Maps a cell to an integer value.
int Drone::cellToInt(Cell src) {
  return src.y * caveWidth + src.x;
}

//Maps an integer value to a cell in the cave.
Cell Drone::intToCell(int src) {
  int y = (int)src / caveWidth;
  int x = src % caveWidth;
  return Cell(x,y);
}

//Gets the Manhattan distance between two cells in the cave.
float Drone::getCellManhattanDist(Cell start, Cell end) {
  return abs(start.x - end.x) + abs(start.y - end.y);
}

//Gets the Euclidean distance between two cells in the cave.
float Drone::getCellEuclideanDist(Cell start, Cell end) {
  return pow(pow(start.x - end.x, 2.0f) + pow(start.y - end.y, 2.0f), 0.5f);
}

//Constructs the final path obtained from the A* algorithm.
vector<Cell> Drone::getAStarPath(map<int,int> previous, int current) {
  vector<Cell> totalPath;
  int cur = current;
  totalPath.push_back(intToCell(cur));
  //temp?
  while (previous.count(cur) > 0) {
    cur = previous[cur];
    totalPath.push_back(intToCell(cur));
  }
  return totalPath;
}

//Uses the A* algorithm to find a path between two cells.
vector<Cell> Drone::searchAStar(Cell start, Cell dest) {

  cout << "[A STAR] - Start: (" << start.x << "," << start.y << ") - Goal: (" << dest.x << "," << dest.y << ")" << endl;

  //If start cell is the same as the destination.
  if (start == dest) {
    vector<Cell> single;
    single.push_back(start);
    return single;
  }

  set<int> closedSet; //Set of evaluated cells.
  set<int> openSet; //Set of unevaluated cells.
  openSet.insert(cellToInt(start));

  map<int,int> previous;

  map<int,float> gScore;
  gScore[cellToInt(start)] = 0;

  map<int,float> fScore;
  fScore[cellToInt(start)] = getCellManhattanDist(start, dest);

  while (!openSet.empty()) {
    Cell current;
    float minScore = numeric_limits<float>::max();

    //Gets the cell with the smallest fScore.
    //###cout << "[FSCORE] - ";
    for (auto const& x : fScore) {
      //###cout << "(" << intToCell(x.first).x << "," << intToCell(x.first).y << "," << x.second << ")";
      if (openSet.count(x.first) > 0 && x.second < minScore) {
        minScore = x.second;
        current = intToCell(x.first);
      }
    }

    //###cout << endl << "[Current] - (" << current.x << "," << current.y << ")" << endl;

    int currentI = cellToInt(current);

    if (current == dest) {
      return getAStarPath(previous, currentI);
    }

    set<int>::iterator currentIt = openSet.find(currentI);
    openSet.erase(currentIt);
    closedSet.insert(currentI);

    vector<Cell> neighbours; //List of adjacent free/frontier cells to the current cell.
    int x = current.x;
    int y = current.y;

    bool left = x - 1 >= 0 && (internalMap[x-1][y] == Free || internalMap[x-1][y] == Frontier);
    bool right = x + 1 < caveWidth && (internalMap[x+1][y] == Free || internalMap[x+1][y] == Frontier);
    bool bottom = y - 1 >= 0 && (internalMap[x][y-1] == Free || internalMap[x][y-1] == Frontier);
    bool top = y + 1 < caveHeight && (internalMap[x][y+1] == Free || internalMap[x][y+1] == Frontier);
    bool bottomleft = bottom && left && (internalMap[x-1][y-1] == Free || internalMap[x-1][y-1] == Frontier);
    bool bottomright = bottom && right && (internalMap[x+1][y-1] == Free || internalMap[x+1][y-1] == Frontier);
    bool topleft = top && left && (internalMap[x-1][y+1] == Free || internalMap[x-1][y+1] == Frontier);
    bool topright = top && right && (internalMap[x+1][y+1] == Free || internalMap[x+1][y+1] == Frontier);

    //Left Neighbour.
    if (left) { neighbours.push_back(Cell(x-1,y)); }
    //Right Neighbour.
    if (right) { neighbours.push_back(Cell(x+1,y)); }
    //Bottom Neighbour.
    if (bottom) { neighbours.push_back(Cell(x,y-1)); }
    //Top Neighbour.
    if (top) { neighbours.push_back(Cell(x,y+1)); }
    //Bottom-Left Neighbour.
    if (bottomleft) { neighbours.push_back(Cell(x-1,y-1)); }
    //Bottom-Right Neighbour.
    if (bottomright) { neighbours.push_back(Cell(x+1,y-1)); }
    //Top-Left Neighbour.
    if (topleft) { neighbours.push_back(Cell(x-1,y+1)); }
    //Top-Right Neighbour.
    if (topright) { neighbours.push_back(Cell(x+1,y+1)); }
    //###where dist = 1 change for diagonals where sqr(2)


    //###cout << "[Neighbours] - ";
    for (vector<Cell>::iterator n = neighbours.begin(); n != neighbours.end(); ++n) {
      Cell neighbour = *n;
      int neighbourI = cellToInt(neighbour);
      //###cout << "(" << neighbour.x << "," << neighbour.y << ") ";

      //Skip neighbour cell if it has previously been evaluated.
      if (closedSet.count(neighbourI) > 0) {
        continue;
      }

      int midDist = gScore[currentI] + getCellEuclideanDist(current, neighbour);

      //New node discovered.
      if (openSet.count(neighbourI) == 0) {
        openSet.insert(neighbourI);
      }
      else if (midDist >= gScore[neighbourI]) {
        continue;
      }

      previous[neighbourI] = currentI;
      gScore[neighbourI] = midDist;
      fScore[neighbourI] = gScore[neighbourI] + getCellManhattanDist(neighbour, dest);

    }
    //###cout << endl;
  }
}



void Drone::testinit() {
  pair<vector<SenseCell>,vector<SenseCell>> buffers = sense();
  updateInternalMap(buffers.first, buffers.second);
  findFrontierCells(buffers.first, buffers.second);
  currentTarget = getBestFrontier();
  targetPath = getPathToTarget(currentTarget);
}


void Drone::test() {

  cout << "Target - (" << currentTarget.first.x << "," << currentTarget.first.y << ") - " << targetPath.size() << endl;

  if (internalMap[currentTarget.first.x][currentTarget.first.y] != Frontier) {
    cout << "New Target" << endl;
    currentTarget = getBestFrontier();
    targetPath = getPathToTarget(currentTarget);
  }
  else {
    cout << "Current Target" << endl;
    cout << "[Pos] - " << targetPath.front().x << "," << targetPath.front().y << endl;
    setPosition(targetPath.front().x, targetPath.front().y);
    targetPath.erase(targetPath.begin()); //Removes the first cell in the target path.
    cout << "[Nxt] - " << targetPath.front().x << "," << targetPath.front().y << endl;
  }

  pair<vector<SenseCell>,vector<SenseCell>> buffers = sense();
  updateInternalMap(buffers.first, buffers.second);
  findFrontierCells(buffers.first, buffers.second);

  recordConfiguration();
}

//If drone-to-drone collision avoidance becomes too tough.
//Allow them to pass through each other, saying they take different altitudes.
