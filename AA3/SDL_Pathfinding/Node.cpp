#include "Node.h"

void Node::updateWeight(int baseWeight, int distance, int maxDistance) {
    int newWeight = baseWeight - ((baseWeight - 1) * distance) / maxDistance;
    if (newWeight < 1) newWeight = 1; // Asegúrate de que el peso mínimo sea 1
    setWeight(newWeight);
}
