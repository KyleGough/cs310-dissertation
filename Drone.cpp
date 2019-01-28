#include <iostream>
#include <stdio.h>
#include <math.h>
#include "Drone.h"
using namespace std;

static constexpr float maxVelocity = 0.25f;
static constexpr float acceleration = 0.1f;
static constexpr float searchRange = 5.0f; //Range of localised search.
static int caveWidth;
static int caveHeight;

float x; //Current x position in the cave.
float y; //Current y position in the cave.
float orientation; //Orientation: 0 -> Facing North.

//###int internalMap[caveWidth][caveHeight]; //Known contents of the cave. Start all unknown.
//###List frontierCells; //Free cells that are adjacent to unknowns.
//###List path; //Position and time pairs.


//Sets global cave properties.
void Drone::setParams(int _caveWidth, int _caveHeight) {
  caveWidth = _caveWidth;
  caveHeight = _caveHeight;
}

//Sets the drone's current position in the cave.
void Drone::setPosition(int _x, int _y) {
  x = _x;
  y = _y;
}

//Senses the local environment.
void Drone::sense() {

  for (int i = floor(x - searchRange); i <= ceil(x + searchRange); i++) {
    for (int j = floor(y - searchRange); j <= ceil(y + searchRange); j++) {
      //Out-of-bounds cells.
      if (i < 0 || j < 0 || i >= caveWidth || j >= caveHeight) {
        continue;
      }

      cout << i << "-" << j << endl;
    }
  }

  //Check to make sure you can't sense objects hidden behind something else.

  //ray from drone center to cell center.



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
