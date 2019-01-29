#ifndef SENSECELL_H
#define SENSECELL_H

struct SenseCell {
  float x;
  float y;
  float range;

  SenseCell(float _x, float _y, float _range) : x(_x), y(_y), range(_range) {}
};

#endif
