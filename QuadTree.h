#ifndef QUADTREE_H
#define QUADTREE_H
//https://www.geeksforgeeks.org/quad-tree/
//Need to re-implement, but for now use for testing.
#include <iostream>
#include <cmath>
using namespace std;


// The objects that we want stored in the quadtree
struct QuadNode {
    int x;
    int y;
    int data;

    QuadNode(int _x, int _y, int _data) {
      x = _x;
      y = _y;
      data = _data;
    }
};

// The main quadtree class
class Quad {

    // Contains details of QuadNode
    QuadNode *n;

    // Children of this tree
    Quad *topLeftTree;
    Quad *topRightTree;
    Quad *botLeftTree;
    Quad *botRightTree;

public:
    // Hold details of the boundary of this node
    int top;
    int left;
    int bot;
    int right;

    Quad() {
        top = 0;
        left = 0;
        bot = 0;
        right = 0;
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
    }

    Quad(int _left, int _top, int _bot, int _right) {
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
        left = _left;
        right = _right;
        top = _top;
        bot = _bot;
    }

    // Insert a node into the quadtree
    void insert(int x, int y);

    // Find a node in a quadtree
    QuadNode* search(int x, int y);

    // Check if current quadtree contains the point
    bool inBoundary(int x, int y);

};

#endif
