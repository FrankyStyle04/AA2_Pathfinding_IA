#pragma once
#include <vector>
#include <queue>
#include <iostream>
#include "Scene.h"
#include "Agent.h"
#include "PathFollowing.h"
#include "Grid.h"
#include "SearchVisualizer.h"

class ScenePathFindingMouse : public Scene {
public:
    ScenePathFindingMouse();
    ~ScenePathFindingMouse();

    void update(float dtime, SDL_Event* event); 
    void draw();
    const char* getTitle();

private:
    SearchVisualizer* search_visualizer;

    // Estado del algoritmo BFS
    bool isStarted = false;
    Vector2D target;

    // Control de interacción
    bool isClicking = false; 
    Vector2D clickedTarget = Vector2D(-1, -1); 

    // Agente y entorno
    std::vector<Agent*> agents;
    Vector2D coinPosition;
    Grid* grid;

    // Configuración gráfica
    bool draw_grid = false;
    SDL_Texture* background_texture = nullptr;
    SDL_Texture* coin_texture = nullptr;

    Uint32 bfsStepTime = 0;   
    int bfsDelay = 20;                         
    std::vector<std::vector<bool>> visitedNodes;
    std::vector<std::vector<Vector2D>> cameFrom; 

    // Métodos auxiliares
    void StartBestFirstSearch(Vector2D start, Vector2D goal); // Inicia BFS
    bool StepBestFirstSearch();                              // Avanza un paso en BFS
    void drawMaze();
    void drawCoin();
    bool loadTextures(char* filename_bg, char* filename_coin);
};
