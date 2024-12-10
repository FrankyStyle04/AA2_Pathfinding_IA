#pragma once
#include <vector>
#include <queue>
#include <iostream>
#include <map>

#include "Scene.h"
#include "Agent.h"
#include "PathFollowing.h"
#include "Grid.h"
#include "SearchVisualizer.h"

enum PathfindingAlgorithm {
    BFS,
    DIJKSTRA,
    A,
    GBFS
};

class ScenePathFindingMouse : public Scene {
public:
    ScenePathFindingMouse();
    ~ScenePathFindingMouse();

    void update(float dtime, SDL_Event* event); 
    void draw();
    const char* getTitle();

private:
    SearchVisualizer* search_visualizer;
    PathfindingAlgorithm currentAlgorithm = BFS;

    // Inputs
    bool isClicking = false; 
    Vector2D clickedTarget = Vector2D(-1, -1); 

    // Agent
    std::vector<Agent*> agents;
    Vector2D coinPosition;
    Grid* grid;

    // Grafic configuration
    bool draw_grid = false;
    SDL_Texture* background_texture = nullptr;
    SDL_Texture* coin_texture = nullptr;

    //BFS variables
    Uint32 bfsStepTime = 0;   
    int bfsDelay = 20;                         
    std::vector<std::vector<bool>> visitedNodes;
    std::vector<std::vector<Vector2D>> cameFrom;
    bool isStarted = false;
    Vector2D target;

    //Dijkstra variables
    std::priority_queue<std::pair<float, Vector2D>, std::vector<std::pair<float, Vector2D>>, std::greater<>> dijkstraFrontier;
    std::map<Vector2D, Vector2D> dijkstraCameFrom;
    std::map<Vector2D, float> dijkstraCostSoFar;
    bool isDijkstraRunning = false;
    Vector2D dijkstraGoal;

    //BFS REGION
    void BFSAlgorithm(Vector2D start, Vector2D goal);
    bool StepBestFirstSearch();

    //DIJKSTRA REGION
    void DijkstraAlgorithm(Vector2D start, Vector2D goal);
    bool StepDijkstra();
    
    
    void drawMaze();
    void drawCoin();
    bool loadTextures(char* filename_bg, char* filename_coin);
};
