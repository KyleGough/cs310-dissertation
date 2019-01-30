#ifndef QUADTREE_H
#define QUADTREE_H
//https://www.geeksforgeeks.org/quad-tree/
//Need to re-implement, but for now use for testing.
#include <iostream>
#include <cmath>
using namespace std;

// Used to hold details of a point
struct Point
{
    float x;
    float y;
    Point(int _x, int _y) {
        x = _x;
        y = _y;
    }
    Point() {
        x = 0;
        y = 0;
    }
};

// The objects that we want stored in the quadtree
struct QuadNode {
    Point pos;
    int data;
    QuadNode(Point _pos, int _data) {
        pos = _pos;
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
    Point topLeft;
    Point botRight;

    Quad() {
        topLeft = Point(0, 0);
        botRight = Point(0, 0);
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
    }

    Quad(Point topL, Point botR) {
        n = NULL;
        topLeftTree  = NULL;
        topRightTree = NULL;
        botLeftTree  = NULL;
        botRightTree = NULL;
        topLeft = topL;
        botRight = botR;
    }

    // Insert a node into the quadtree
    void insert(QuadNode *node);

    // Find a node in a quadtree
    QuadNode* search(Point p);

    // Check if current quadtree contains the point
    bool inBoundary(Point p);

};

#endif
