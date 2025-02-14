#pragma once
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <time.h>
#include "Agent.h"

class Grid
{
public:
	Grid(char* wallsFilename, char* nodeWeightsFilename);
	~Grid();

private:
	int num_cell_x;
	int num_cell_y;

	std::vector< std::vector<int> > terrain;
	std::vector< std::vector<Node*> > nodeGrid;

public:
	
	int getNumCellX();
	int getNumCellY();
	Node* getNode(int x, int y) {
		
		return nodeGrid[y][x];
	}

	std::vector<Vector2D> getNeighbors(Vector2D cell);
	void updateNodeWeights(Vector2D enemyPos, int maxDistance, int maxWeight);
	float getCost(Vector2D from, Vector2D to);

	Vector2D cell2pix(Vector2D cell);
	Vector2D pix2cell(Vector2D pix);
	bool isValidCell(Vector2D cell);
};
