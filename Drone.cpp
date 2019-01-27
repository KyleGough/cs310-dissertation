
float x; //Current x position in the cave.
float y; //Current y position in the cave.
float orientation; //Orientation: 0 -> Facing North.

int internalMap[caveWidth][caveHeight]; //Known contents of the cave. Start all unknown.
List frontierCells. //Free cells that are adjacent to unknowns.
List path; //Position and time pairs.
