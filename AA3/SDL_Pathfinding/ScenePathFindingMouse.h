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
    bool bfs_in_progress = false;
    Vector2D bfs_goal;

    // Control de interacción
    bool click_locked = false; 
    Vector2D click_target = Vector2D(-1, -1); 

    // Agente y entorno
    std::vector<Agent*> agents;
    Vector2D coinPosition;
    Grid* maze;

    // Configuración gráfica
    bool draw_grid = false;
    SDL_Texture* background_texture = nullptr;
    SDL_Texture* coin_texture = nullptr;

    Uint32 last_bfs_step_time = 0;   
    int bfs_delay = 20;                         
    std::vector<std::vector<bool>> bfs_visited;
    std::vector<std::vector<Vector2D>> bfs_came_from; 

    // Métodos auxiliares
    void startBFS(Vector2D start, Vector2D goal); // Inicia BFS
    bool stepBFS();                              // Avanza un paso en BFS
    void drawMaze();
    void drawCoin();
    bool loadTextures(char* filename_bg, char* filename_coin);
};
