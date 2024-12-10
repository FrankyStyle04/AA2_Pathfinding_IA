#include "SearchVisualizer.h"

SearchVisualizer::SearchVisualizer(Grid* grid) : grid(grid), nodes_added(0) {}

void SearchVisualizer::reset() {
    while (!frontier.empty()) {
        frontier.pop();
    }
    nodes_added = 0;
    dynamic_frontier.clear();
}

void SearchVisualizer::addToFrontier(Vector2D node) {
    if (!grid->isValidCell(node)) return;  // Verificar validez de la celda

    frontier.push(node);
    dynamic_frontier.push_back(grid->cell2pix(node));
    nodes_added++;
    std::cout << "Nodes afegits a la frontera: " << nodes_added << std::endl;
}

Vector2D SearchVisualizer::popFrontier() {
    if (frontier.empty()) return Vector2D(-1, -1);
    Vector2D node = frontier.front();
    frontier.pop();
    return node;
}

bool SearchVisualizer::isFrontierEmpty() const {
    return frontier.empty();
}

const std::vector<Vector2D>& SearchVisualizer::getDynamicFrontier() const {
    return dynamic_frontier;
}

int SearchVisualizer::getNodesAddedCount() const {
    return nodes_added;
}
