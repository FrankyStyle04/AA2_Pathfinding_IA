#pragma once
#include <vector>

class Node
{

public:
	Node(int x, int y, int weight) :x(x), y(y), weight(weight) {}
	int getWeight() {
		return weight;
	}
private:
	int x;
	int y;
	int weight;
	std::vector<std::pair<Node*, int>> neighbors;

	void setWeight(int weight1) {
		weight = weight1;
	}
};
