#ifndef SENSECELL_H
#define SENSECELL_H

struct SenseCell {
  int x;
  int y;
  float range;
  SenseCell(float _x, float _y, float _range) : x(_x), y(_y), range(_range) {}
};

#endif
