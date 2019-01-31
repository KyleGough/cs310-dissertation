#include <iostream>
#include <cmath>
#include "QuadTree.h"
using namespace std;


void Quad::insert(QuadNode *node) {

    if (node == nullptr)
        return;

    // Current quad cannot contain it
    if (!inBoundary(node->x, node->y))
        return;

    // We are at a quad of unit area
    // We cannot subdivide this quad further
    if (abs(left - right) <= 1 &&
        abs(top - bot) <= 1)
    {
        if (n == nullptr)
            n = node;
        return;
    }

    if ((left + right) / 2 >= node->x)
    {
        // Indicates topLeftTree
        if ((top + bot) / 2 >= node->y)
        {
            if (topLeftTree == nullptr)
                topLeftTree = new Quad(left, top, (left + right) / 2, (top + bot) / 2);
            topLeftTree->insert(node->x, node->y);
        }

        // Indicates botLeftTree
        else
        {
            if (botLeftTree == nullptr)
                botLeftTree = new Quad(left, (top + bot) / 2, (left + right) / 2, bot);
            botLeftTree->insert(node->x, node->y);
        }
    }
    else
    {
        // Indicates topRightTree
        if ((top + bot) / 2 >= node->y)
        {
            if (topRightTree == nullptr)
                topRightTree = new Quad((left + right) / 2, top, right, (top + bot) / 2);
            topRightTree->insert(node->x, node->y);
        }

        // Indicates botRightTree
        else
        {
            if (botRightTree == nullptr)
                botRightTree = new Quad((left + right) / 2, (top + bot) / 2, right, bot);
            botRightTree->insert(node->x, node->y);
        }
    }
}

QuadNode* Quad::search(int x, int y) {
    // Current quad cannot contain it
    if (!inBoundary(x,y))
        return nullptr;

    // We are at a quad of unit length
    // We cannot subdivide this quad further
    if (n != nullptr)
        return n;

    if ((left + right) / 2 >= x)
    {
        // Indicates topLeftTree
        if ((top + bot) / 2 >= y)
        {
            if (topLeftTree == nullptr)
                return nullptr;
            return topLeftTree->search(p);
        }

        // Indicates botLeftTree
        else
        {
            if (botLeftTree == nullptr)
                return nullptr;
            return botLeftTree->search(p);
        }
    }
    else
    {
        // Indicates topRightTree
        if ((top + bot) / 2 >= y)
        {
            if (topRightTree == nullptr)
                return nullptr;
            return topRightTree->search(p);
        }

        // Indicates botRightTree
        else
        {
            if (botRightTree == nullptr)
                return nullptr;
            return botRightTree->search(p);
        }
    }
};

bool Quad::inBoundary(int x, int y) {
  return (x >= left &&
      x <= right &&
      y >= top &&
      y <= bot);
}
