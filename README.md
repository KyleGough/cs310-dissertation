<h1>Dissertation - Using Swarm AI to map a cave network</h1>

## Abstract
This project aims to find an efficient solution to cave exploration and mapping by simulating
multiple autonomous flying drones, all initially unaware of their environment in a visualised
and custom generated cave environment. The project tackles various problems in the field of
robotics such as: sensing, frontier searching, teamwork, route planning and target selection.
Drones will build an occupancy grid of its environment then proceed to map the cave by
repeatedly identifying and searching an optimal frontier cell which are boundaries between
unoccupied areas and unknown areas. Incorporating multiple drones into the simulation
allows for communication of information to propel search efficiency and thus reduce search
duplication achieved by mimicking swarm behaviour. Such behaviour includes exploring
frontiers away from nearby drones, moving away from nearby drones, and branching out at
junctions. Realistic caves for the simulation are generated using a random but reproducible
process combining Simplex Noise, Cellular Automata and Flood fill. The project aims to
identify how various factors of the drone’s behaviour such as how many drones exist or their
search radius may affect the performance of the search.

## Running
Build from source
```
./build
```

Run the program
```
./main
```
