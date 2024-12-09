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
	Vector2D cell2pix(Vector2D cell);
	Vector2D pix2cell(Vector2D pix);
	bool isValidCell(Vector2D cell);
	int getNumCellX();
	int getNumCellY();
	Node* GetNode(int x, int y) {
		
		return nodeGrid[y][x];
	}
};
