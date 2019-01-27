#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;

class Drone {

  public:

  float x; //Current x position in the cave.
  float y; //Current y position in the cave.
  float orientation; //Orientation: 0 -> Facing North.

  //###int internalMap[caveWidth][caveHeight]; //Known contents of the cave. Start all unknown.
  //###List frontierCells; //Free cells that are adjacent to unknowns.
  //###List path; //Position and time pairs.


  //Constructor.
  Drone(int _x, int _y) {
    x = _x;
    y = _y;
  }

  void sense() {
    //Scan a block of size searchrangexsearchrange
    //determine if a cell is within a circle radius.

    for (int i = floor(x - searchRange); i < ceil(x + searchRange); i++) {
      for (int j = floor(y - searchRange); j < ceil(y + searchRange); j++) {
        cout << i << "-" << j << endl;
      }
    }


  }



  private:

  static constexpr float maxVelocity = 0.25f;
  static constexpr float acceleration = 0.1f;
  static constexpr float searchRange = 5.0f; //Range of localised search.

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

};
