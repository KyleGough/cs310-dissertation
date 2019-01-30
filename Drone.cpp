#include <iostream>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>
#include <algorithm>
#include <vector>
#include "Cell.h"
#include "SenseCell.h"
#include "MapCell.h"
#include "Drone.h"
using namespace std;

static constexpr float maxVelocity = 0.25f;
static constexpr float acceleration = 0.1f;
float Drone::searchRange = 6.0f; //### //Range of localised search.
static int caveWidth;
static int caveHeight;
static vector<vector<int>> cave;

float posX; //Current x position in the cave.
float posY; //Current y position in the cave.
float orientation; //Orientation: 0 -> Facing North.
vector<SenseCell> freeCellBuffer; //List of free cells sensed from the last sense operation.
vector<SenseCell> occupiedCellBuffer; //List of occupied cells sensed from the last sense operation.

Quad quadCave(); //###Known contents of the cave. ###Start all unknown.
vector<vector<int>> internalMap;
vector<SenseCell> frontierCells; //Free cells that are adjacent to unknowns.
//###List path; //Position and time pairs.


//Less than comparison function for two SenseCell objects.
bool operator <(const SenseCell& a, const SenseCell& b) {
  return a.range < b.range;
}


//Sets global cave properties.
void Drone::setParams(int _caveWidth, int _caveHeight, vector<vector<int>> _cave) {
  caveWidth = _caveWidth;
  caveHeight = _caveHeight;
  cave = _cave;
}

void Drone::init(int x, int y) {
  posX = x;
  posY = y;
  quadCave.topLeft = Point(0,0);
  quadCave.botRight = Point(caveWidth, caveHeight);

  internalMap.clear();
  for (int i = 0; i < caveWidth; i++) {
    vector<int> column;
    for (int j = 0; j < caveHeight; j++) {
      column.push_back(Unknown);
    }
    internalMap.push_back(column);
  }

}

//Sets the drone's current position in the cave.
void Drone::setPosition(int x, int y) {
  posX = x;
  posY = y;
}

//Models the sensing of the immediate local environment.
void Drone::sense() {

  vector<SenseCell> candidates; //List of candidate cells.
  vector<SenseCell> freeCells; //List of found free cells.
  vector<SenseCell> occupiedCells; //List of found occupied cells.
  vector<SenseCell> checkCells; //List of cells to check.

  //For each cell in the bounding box of the search range.
  for (int i = floor(posX - searchRange); i <= ceil(posX + searchRange); i++) {
    for (int j = floor(posY - searchRange); j <= ceil(posY + searchRange); j++) {
      //Discard Out-of-bounds cells.
      if (i < 0 || j < 0 || i >= caveWidth || j >= caveHeight) { continue; }
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
      if (cave[dest->x][dest->y] == Free) {
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
          /*###cout << "COLLISION at (" << dest->x << "," << dest->y << ") with (" << occupyCheck->x << "," << occupyCheck->y << ")";
          cout << "[" << tx0 << "," << yCheck << "]" << endl;*/
          break;
        }
      }
      if (tx1 >= 0 && tx1 <= 1) {
        float yCheck = posY + tx1 * (dest->y - posY);
        if (yCheck >= occupyCheck->y - 0.5f && yCheck <= occupyCheck->y + 0.5f) {
          collisionDetected = true;
          /*###cout << "COLLISION at (" << dest->x << "," << dest->y << ") with (" << occupyCheck->x << "," << occupyCheck->y << ")";
          cout << "[" << tx1 << "," << yCheck << "]" << endl;*/
          break;
        }
      }
      if (ty0 >= 0 && ty0 <= 1) {
        float xCheck = posX + ty0 * (dest->x - posX);
        if (xCheck >= occupyCheck->x - 0.5f && xCheck <= occupyCheck->x + 0.5f) {
          collisionDetected = true;
          /*###cout << "COLLISION at (" << dest->x << "," << dest->y << ") with (" << occupyCheck->x << "," << occupyCheck->y << ")";
          cout << "[" << ty0 << "," << xCheck << "]" << endl;*/
          break;
        }
      }
      if (ty1 >= 0 && ty0 <= 1) {
        float xCheck = posX + ty1 * (dest->x - posX);
        if (xCheck >= occupyCheck->x - 0.5f && xCheck <= occupyCheck->x + 0.5f) {
          collisionDetected = true;
          /*###cout << "COLLISION at (" << dest->x << "," << dest->y << ") with (" << occupyCheck->x << "," << occupyCheck->y << ")";
          cout << "[" << ty1 << "," << xCheck << "]" << endl;*/
          break;
        }
      }
    }

    //If no collision detected then the destination cell is in line of sight from the drone's position.
    if (!collisionDetected) {
      if (cave[dest->x][dest->y] == Free) {
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

  //Removes contents from the buffers.
  //freeCellBuffer.clear();
  //occupiedCellBuffer.clear();
  //Sets the cell buffers to the sensed cells.
  freeCellBuffer = freeCells;
  occupiedCellBuffer = occupiedCells;

  //###
  cout << "FREE CELLS (" << freeCells.size() << ")" << endl;
  for (vector<SenseCell>::iterator it = freeCells.begin(); it != freeCells.end(); ++it) {
    cout << "(" << it->x << "," << it->y << ") ";
  }
  cout << endl;

  cout << "OCCUPIED CELLS (" << occupiedCells.size() << ")" << endl;
  for (vector<SenseCell>::iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it) {
    cout << "(" << it->x << "," << it->y << ") ";
  }
  cout << endl;

  updateInternalMap();
  findFrontierCells();
}

//Updates the internal map of the drone to include recently sensed free and occupied cells.
void Drone::updateInternalMap() {

  //Adds all free cells to the internal map.
  cout << "FREE" << endl; //###
  for (vector<SenseCell>::iterator freeCell = freeCellBuffer.begin(); freeCell != freeCellBuffer.end(); ++freeCell) {
    int x = freeCell->x;
    int y = freeCell->y;
    if (internalMap[x][y] == Unknown) {
      internalMap[x][y] = Free;
    }

    /*//if (quadCave.search(Point(freeCell->x, freeCell->y)) == NULL) {
      cout << freeCell->x << "," << freeCell->y << "Map: " << cave[freeCell->x][freeCell->y] << endl;
      int _x = freeCell->x;
      int _y = freeCell->y;
      Point a = Point(_x,_y);
      QuadNode p = QuadNode(a, 90);
      quadCave.insert(&p);
    //}*/
  }

  cout << "OCCUPY" << endl; //###
  //Adds all occupied cells to the internal map.
  for (vector<SenseCell>::iterator occupyCell = occupiedCellBuffer.begin(); occupyCell != occupiedCellBuffer.end(); ++occupyCell) {
    int x = occupyCell->x;
    int y = occupyCell->y;
    if (internalMap[x][y] == Unknown) {
      internalMap[x][y] = Occupied;
    }

    /*//if (quadCave.search(Point(occupyCell->x, occupyCell->y)) == NULL) {
      cout << occupyCell->x << "," << occupyCell->y << "Map: " << cave[occupyCell->x][occupyCell->y] << endl;
      int _x = occupyCell->x;
      int _y = occupyCell->y;
      Point a = Point(_x,_y);
      QuadNode v = QuadNode(a,69);
      quadCave.insert(&v);
  //  }*/
  }

}

void Drone::findFrontierCells() {

  //###
  vector<Cell> frontierCheck;

  for (vector<SenseCell>::iterator freeCell = freeCellBuffer.begin(); freeCell != freeCellBuffer.end(); ++freeCell) {
    int x = freeCell->x;
    int y = freeCell->y;
    if (x - 1 >= 0 && internalMap[x-1][y] == Frontier) {
      internalMap[x-1][y] = Free;
      frontierCheck.push_back(Cell(x-1,y));
    }
    if (x + 1 < caveWidth && internalMap[x+1][y] == Frontier) {
      internalMap[x+1][y] = Free;
      frontierCheck.push_back(Cell(x+1,y));
    }
    if (y - 1 >= 0 && internalMap[x][y-1] == Frontier) {
      internalMap[x][y-1] = Free;
      frontierCheck.push_back(Cell(x,y-1));
    }
    if (y + 1 < caveHeight && internalMap[x][y+1] == Frontier) {
      internalMap[x][y+1] = Free;
      frontierCheck.push_back(Cell(x,y+1));
    }
    frontierCheck.push_back(Cell(x,y));
  }

  for (vector<SenseCell>::iterator occupyCell = occupiedCellBuffer.begin(); occupyCell != occupiedCellBuffer.end(); ++occupyCell) {
    int x = occupyCell->x;
    int y = occupyCell->y;
    if (x - 1 >= 0 && internalMap[x-1][y] == Frontier) {
      internalMap[x-1][y] = Free;
      frontierCheck.push_back(Cell(x-1,y));
    }
    if (x + 1 < caveWidth && internalMap[x+1][y] == Frontier) {
      internalMap[x+1][y] = Free;
      frontierCheck.push_back(Cell(x+1,y));
    }
    if (y - 1 >= 0 && internalMap[x][y-1] == Frontier) {
      internalMap[x][y-1] = Free;
      frontierCheck.push_back(Cell(x,y-1));
    }
    if (y + 1 < caveHeight && internalMap[x][y+1] == Frontier) {
      internalMap[x][y+1] = Free;
      frontierCheck.push_back(Cell(x,y+1));
    }
  }

  for (vector<Cell>::iterator frontierCell = frontierCheck.begin(); frontierCell != frontierCheck.end(); ++frontierCell) {
    int x = frontierCell->x;
    int y = frontierCell->y;
    if (x - 1 >= 0 && internalMap[x-1][y] == Unknown) {
      internalMap[x][y] = Frontier;
    }
    if (x + 1 < caveWidth && internalMap[x+1][y] == Frontier) {
      internalMap[x][y] = Frontier;
    }
    if (y - 1 >= 0 && internalMap[x][y-1] == Frontier) {
      internalMap[x][y] = Frontier;
    }
    if (y + 1 < caveHeight && internalMap[x][y+1] == Frontier) {
      internalMap[x][y] = Frontier;
    }

  }



  cout << "Find Frontier Cells." << endl;
  for (int i = 0; i < caveWidth; i++) {
    for (int j = 0; j < caveHeight; j++) {
      //for each frontier neighbour of cell in both buffers.
      //plus cell itself.

    }
  }

}



//If drone-to-drone collision avoidance becomes too tough.
//Allow them to pass through each other, saying they take different altitudes.


//(1)
//Search local area.

//(2)
//Place sensed data into internal map.

//(3)
//Compute frontier cells.

//(4)
//Find optimal frontier cell.

//(5)
//Plan path to frontier cell.
