#include <iostream>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>
#include <algorithm>
#include <vector>
#include <algorithm>
#include <map>
#include <limits>
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
vector<SenseCell> freeCellBuffer; //List of free cells sensed from the last sense operation.
vector<SenseCell> occupiedCellBuffer; //List of occupied cells sensed from the last sense operation.
vector<vector<int>> internalMap; //###
map<int, int> frontierCells; //Free cells that are adjacent to unknowns.
vector<DroneConfig> path; //List of drone configurations for each timestep. //###
int currentTimestep; //###
Cell target; //###


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
  freeCellBuffer.clear();
  occupiedCellBuffer.clear();
  frontierCells.clear(); //###

  //Sets the internal map to all unknowns.
  internalMap.clear();
  for (size_t i = 0; i < caveWidth; i++) {
    vector<int> column;
    for (size_t j = 0; j < caveHeight; j++) {
      column.push_back(Unknown);
    }
    internalMap.push_back(column);
  }
}

//Sets the drone's current position in the cave.
void Drone::setPosition(float x, float y) {
  posX = x;
  posY = y;
  //###
  cout << " + [" << name << "]" << " - Pos: (" << posX << "," << posY << ")" << endl;
}

//Models the sensing of the immediate local environment.
void Drone::sense() {

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

  //Sets the cell buffers to the sensed cells.
  freeCellBuffer = freeCells;
  occupiedCellBuffer = occupiedCells;

  //###remove one movement and a* implemented.
  updateInternalMap();
  findFrontierCells();
  Cell a = getBestFrontier(); //###
}

//Updates the internal map of the drone to include recently sensed free and occupied cells.
void Drone::updateInternalMap() {

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
void Drone::findFrontierCells() {

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
Cell Drone::getBestFrontier() {

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

  return bestFrontier;
}

//Adds the drone's current configuration to the path.
void Drone::recordConfiguration() {
  path.push_back(DroneConfig(currentTimestep, posX, posY, orientation));
  //###maybe timestep++ here.
}

/*void navigateToTarget() {
  //if same timestep as target, no backtracking, use A* to get the path then navigate to it.

  //if previous timestep then
  //backtrack
  //once reached target timestep then use a*.
}*/


/*void getTarget() {
  target = getBestFrontier();
}*/





//###
/*void Drone::pathingxddd() {

  sense();
  updateInternalMap();
  findFrontierCells();
  Cell bestFrontier = getBestFrontier();


}*/


//If drone-to-drone collision avoidance becomes too tough.
//Allow them to pass through each other, saying they take different altitudes.

//(1) Search local area. DONE
//(2) Place sensed data into internal map. DONE
//(3) Compute frontier cells. DONE
//(4) Find optimal frontier cell.
//(5) Plan path to frontier cell.
