#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

struct DroneConfig {
  int timestep;
  float x;
  float y;
  float orientation;
  DroneConfig(int _timestep, float _x, float _y, float _orientation) {
    timestep = _timestep;
    x = _x;
    y = _y;
    orientation = _orientation;
  }
};

#endif
