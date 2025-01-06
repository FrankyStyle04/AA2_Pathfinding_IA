#pragma once
#include <vector>

class Node
{


private:
	int x;
	int y;
	int weight;
	std::vector<std::pair<Node*, int>> neighbors;
public:
	Node(int x, int y, int weight) :x(x), y(y), weight(weight) {}
	int getWeight() {
		return weight;
	}
	void setWeight(int weight1) {
		weight = weight1;
	}

	void updateWeight(int baseWeight, int distance, int maxDistance);

	
};
