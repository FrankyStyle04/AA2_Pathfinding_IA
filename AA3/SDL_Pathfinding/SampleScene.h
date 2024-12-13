#pragma once
#include "ScenePathFindingMouse.h"
#include <climits> // Para INT_MAX
#include <numeric> // Para std::accumulate
#include <fstream>

struct AlgorithmMetrics {
    std::vector<int> exploredNodes;
    int maxExplored = 0;
    int minExplored = INT_MAX;
    float averageExplored = 0;

    void calculateStatistics() {
        if (!exploredNodes.empty()) {
            maxExplored = *std::max_element(exploredNodes.begin(), exploredNodes.end());
            minExplored = *std::min_element(exploredNodes.begin(), exploredNodes.end());
            averageExplored = std::accumulate(exploredNodes.begin(), exploredNodes.end(), 0.0f) / exploredNodes.size();
        }
    }
};

class SampleScene : public ScenePathFindingMouse
{
public:
    void runSampling(int sampleCount) {
        AlgorithmMetrics bfsMetrics, dijkstraMetrics, aStarMetrics, gbfsMetrics;

        for (int i = 0; i < sampleCount; ++i) {
            Vector2D startCell = getRandomValidCell();
            Vector2D goalCell = getRandomValidCell();

            std::cout << "Muestreo " << i + 1 << " de " << sampleCount << std::endl;
            std::cout << "Inicio: (" << startCell.x << ", " << startCell.y << ")" << std::endl;
            std::cout << "Meta: (" << goalCell.x << ", " << goalCell.y << ")" << std::endl;

            // BFS
            search_visualizer->reset();
            BFSAlgorithm(startCell, goalCell);
            while (!StepBestFirstSearch());
            bfsMetrics.exploredNodes.push_back(search_visualizer->getNodesAddedCount());

            // Dijkstra
            search_visualizer->reset();
            DijkstraAlgorithm(startCell, goalCell);
            while (!StepDijkstra());
            dijkstraMetrics.exploredNodes.push_back(search_visualizer->getNodesAddedCount());

            // A*
            search_visualizer->reset();
            AStarAlgorithm(startCell, goalCell);
            while (!StepA());
            aStarMetrics.exploredNodes.push_back(search_visualizer->getNodesAddedCount());

            // GBFS
            search_visualizer->reset();
            GBFSAlgorithm(startCell, goalCell);
            while (!StepGBFS());
            gbfsMetrics.exploredNodes.push_back(search_visualizer->getNodesAddedCount());
        }

        // Calcular estadísticas
        bfsMetrics.calculateStatistics();
        dijkstraMetrics.calculateStatistics();
        aStarMetrics.calculateStatistics();
        gbfsMetrics.calculateStatistics();

        // Guardar resultados en un archivo
        saveResultsToFile("results.txt", bfsMetrics, dijkstraMetrics, aStarMetrics, gbfsMetrics);

        // Reportar resultados en consola
        reportResults("BFS", bfsMetrics);
        reportResults("Dijkstra", dijkstraMetrics);
        reportResults("A*", aStarMetrics);
        reportResults("GBFS", gbfsMetrics);
    }

private:
    Vector2D getRandomValidCell() {
        Vector2D randomCell;
        do {
            randomCell = Vector2D((float)(rand() % grid->getNumCellX()), (float)(rand() % grid->getNumCellY()));
        } while (!grid->isValidCell(randomCell));
        return randomCell;
    }

    void reportResults(const std::string& algorithmName, const AlgorithmMetrics& metrics) {
        std::cout << "\nResultados para " << algorithmName << ":\n";
        std::cout << "Máximo de nodos explorados: " << metrics.maxExplored << std::endl;
        std::cout << "Mínimo de nodos explorados: " << metrics.minExplored << std::endl;
        std::cout << "Promedio de nodos explorados: " << metrics.averageExplored << std::endl;
    }

    void saveResultsToFile(const std::string& filename, const AlgorithmMetrics& bfsMetrics, const AlgorithmMetrics& dijkstraMetrics, const AlgorithmMetrics& aStarMetrics, const AlgorithmMetrics& gbfsMetrics) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error al abrir el archivo " << filename << std::endl;
            return;
        }

        file << "Resultados para BFS:\n";
        file << "Máximo de nodos explorados: " << bfsMetrics.maxExplored << "\n";
        file << "Mínimo de nodos explorados: " << bfsMetrics.minExplored << "\n";
        file << "Promedio de nodos explorados: " << bfsMetrics.averageExplored << "\n\n";

        file << "Resultados para Dijkstra:\n";
        file << "Máximo de nodos explorados: " << dijkstraMetrics.maxExplored << "\n";
        file << "Mínimo de nodos explorados: " << dijkstraMetrics.minExplored << "\n";
        file << "Promedio de nodos explorados: " << dijkstraMetrics.averageExplored << "\n\n";

        file << "Resultados para A*:\n";
        file << "Máximo de nodos explorados: " << aStarMetrics.maxExplored << "\n";
        file << "Mínimo de nodos explorados: " << aStarMetrics.minExplored << "\n";
        file << "Promedio de nodos explorados: " << aStarMetrics.averageExplored << "\n\n";

        file << "Resultados para GBFS:\n";
        file << "Máximo de nodos explorados: " << gbfsMetrics.maxExplored << "\n";
        file << "Mínimo de nodos explorados: " << gbfsMetrics.minExplored << "\n";
        file << "Promedio de nodos explorados: " << gbfsMetrics.averageExplored << "\n\n";

        file.close();
        std::cout << "Resultados guardados en " << filename << std::endl;
    }
};
