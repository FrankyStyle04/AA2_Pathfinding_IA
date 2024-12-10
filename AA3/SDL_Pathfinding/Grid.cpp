#include "Grid.h"

using namespace std;

Grid::Grid(char* wallsFilename, char* nodeWeightsFilename)
{
	num_cell_x = SRC_WIDTH / CELL_SIZE;
	num_cell_y = SRC_HEIGHT / CELL_SIZE;

	// Initialize the terrain matrix from file (for each cell a zero value indicates it's a wall, positive values indicate terrain cell cost)
	std::ifstream infile(wallsFilename);
	std::string line;
	while (std::getline(infile, line))
	{
		vector<int> terrain_row;
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ','))
			terrain_row.push_back(atoi(cell.c_str()));
		SDL_assert(terrain_row.size() == num_cell_x);
		terrain.push_back(terrain_row);
	}
	SDL_assert(terrain.size() == num_cell_y);
	infile.close();


	std::ifstream infile2(nodeWeightsFilename);
	std::string line2;
	int x = 0;
	int y = 0;
	while (std::getline(infile2, line2))
	{
		vector<Node*> nodeGrid_row;
		std::stringstream lineStream(line2);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {

			Node* node = new Node(x, y, atoi(cell.c_str()));
			nodeGrid_row.push_back(node);
			x += 1;
		}
		x = 0;
		y += 1;

		SDL_assert(nodeGrid_row.size() == num_cell_x);
		nodeGrid.push_back(nodeGrid_row);
	}
	SDL_assert(nodeGrid.size() == num_cell_y);
	infile2.close();
}

Grid::~Grid()
{
}

int Grid::getNumCellX()
{
	return num_cell_x;
}

int Grid::getNumCellY()
{
	return num_cell_y;
}

std::vector<Vector2D> Grid::getNeighbors(Vector2D cell)
{
	std::vector<Vector2D> neighbors;

	std::vector<Vector2D> directions = {
		Vector2D(1, 0), Vector2D(-1, 0), Vector2D(0, 1), Vector2D(0, -1)
	};

	for (Vector2D dir : directions) {
		Vector2D neighbor = cell + dir;
		if (isValidCell(neighbor)) {
			neighbors.push_back(neighbor);
		}
	}

	return neighbors;
}

float Grid::getCost(Vector2D from, Vector2D to)
{
	if (!isValidCell(from) || !isValidCell(to)) {
		return std::numeric_limits<float>::infinity();
	}

	return static_cast<float>(nodeGrid[(int)to.y][(int)to.x]->getWeight());
}

Vector2D Grid::cell2pix(Vector2D cell)
{
	int offset = CELL_SIZE / 2;
	return Vector2D(cell.x * CELL_SIZE + offset, cell.y * CELL_SIZE + offset);
}

Vector2D Grid::pix2cell(Vector2D pix)
{
	return Vector2D((float)((int)pix.x / CELL_SIZE), (float)((int)pix.y / CELL_SIZE));
}

bool Grid::isValidCell(Vector2D cell)
{
	if ((cell.x < 0) || (cell.y < 0) || (cell.y >= terrain.size()) || (cell.x >= terrain[0].size()))
		return false;
	return !(terrain[(unsigned int)cell.y][(unsigned int)cell.x] == 0);
}