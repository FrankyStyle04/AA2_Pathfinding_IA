#pragma once
#include <vector>
#include <queue>
#include <iostream>
#include "Vector2D.h"
#include "Grid.h"

class SearchVisualizer {

private:
    Grid* grid;
    std::queue<Vector2D> frontier;
    std::vector<Vector2D> dynamic_frontier;
    int nodes_added; 

public:
    SearchVisualizer(Grid* grid);

    void reset();

    void addToFrontier(Vector2D node);

    Vector2D popFrontier();

    bool isFrontierEmpty() const;

    const std::vector<Vector2D>& getDynamicFrontier() const;

    int getNodesAddedCount() const;

};
