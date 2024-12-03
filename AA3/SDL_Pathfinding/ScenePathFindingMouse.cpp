#include "ScenePathFindingMouse.h"

ScenePathFindingMouse::ScenePathFindingMouse() {
    draw_grid = false;
    maze = new Grid("../res/maze.csv");

    if (!loadTextures("../res/maze.png", "../res/coin.png")) {
        std::cerr << "Error: No se pudieron cargar las texturas." << std::endl;
        exit(EXIT_FAILURE);
    }

    srand((unsigned int)time(NULL));

    // Crear y configurar el agente
    Agent* agent = new Agent;
    agent->loadSpriteTexture("../res/soldier.png", 4);
    agent->setBehavior(new PathFollowing);
    agent->setTarget(Vector2D(-20, -20));
    agents.push_back(agent);

    // Posicionar el agente en una celda aleatoria válida
    Vector2D rand_cell(-1, -1);
    while (!maze->isValidCell(rand_cell)) {
        rand_cell = Vector2D((float)(rand() % maze->getNumCellX()), (float)(rand() % maze->getNumCellY()));
    }
    agents[0]->setPosition(maze->cell2pix(rand_cell));

    // Colocar la moneda en una celda aleatoria válida
    coinPosition = Vector2D(-1, -1);
    while (!maze->isValidCell(coinPosition) || (Vector2D::Distance(coinPosition, rand_cell) < 3)) {
        coinPosition = Vector2D((float)(rand() % maze->getNumCellX()), (float)(rand() % maze->getNumCellY()));
    }

    // Crear el visualizador de búsqueda
    search_visualizer = new SearchVisualizer(maze);
}

ScenePathFindingMouse::~ScenePathFindingMouse() {
    if (background_texture) SDL_DestroyTexture(background_texture);
    if (coin_texture) SDL_DestroyTexture(coin_texture);
    for (Agent* agent : agents) delete agent;
    delete search_visualizer;
}

void ScenePathFindingMouse::update(float dtime, SDL_Event* event) {
    if (event->type == SDL_MOUSEBUTTONDOWN && !bfs_in_progress && !click_locked) {
        Vector2D clickedCell = maze->pix2cell(Vector2D((float)(event->button.x), (float)(event->button.y)));
        if (maze->isValidCell(clickedCell)) {
            Vector2D startCell = maze->pix2cell(agents[0]->getPosition());
            startBFS(startCell, clickedCell);
            click_target = maze->cell2pix(clickedCell);
            click_locked = true;
        }
    }

    if (bfs_in_progress && SDL_GetTicks() - last_bfs_step_time > bfs_delay) {
        last_bfs_step_time = SDL_GetTicks();
        if (stepBFS()) {
            std::vector<Vector2D> path;
            for (Vector2D step = bfs_goal; step != Vector2D(-1, -1); step = bfs_came_from[(int)step.y][(int)step.x]) {
                path.push_back(maze->cell2pix(step));
            }
            std::reverse(path.begin(), path.end());

            agents[0]->clearPath();
            for (Vector2D point : path) {
                agents[0]->addPathPoint(point);
            }

            bfs_in_progress = false;
        }
    } else if (!bfs_in_progress) {
        agents[0]->update(dtime, event);
        if (maze->pix2cell(agents[0]->getPosition()) == maze->pix2cell(click_target)) {
            click_locked = false;
        }
    }
}

void ScenePathFindingMouse::draw() {
    drawMaze();
    drawCoin();
    for (Vector2D cell : search_visualizer->getDynamicFrontier()) {
        draw_circle(TheApp::Instance()->getRenderer(), (int)cell.x, (int)cell.y, 10, 255, 0, 0, 255);
    }
    agents[0]->draw();
}

const char* ScenePathFindingMouse::getTitle() {
    return "SDL Path Finding :: BFS";
}

void ScenePathFindingMouse::startBFS(Vector2D start, Vector2D goal) {
    if (!maze->isValidCell(start) || !maze->isValidCell(goal)) return;
    bfs_in_progress = true;
    bfs_goal = goal;

    search_visualizer->reset();
    bfs_visited = std::vector<std::vector<bool>>(maze->getNumCellY(), std::vector<bool>(maze->getNumCellX(), false));
    bfs_came_from = std::vector<std::vector<Vector2D>>(maze->getNumCellY(), std::vector<Vector2D>(maze->getNumCellX(), Vector2D(-1, -1)));
    bfs_visited[(int)start.y][(int)start.x] = true;
    search_visualizer->addToFrontier(start);
}

bool ScenePathFindingMouse::stepBFS() {
    if (search_visualizer->isFrontierEmpty()) return false;

    Vector2D current = search_visualizer->popFrontier();
    if (current == bfs_goal) return true;

    std::vector<Vector2D> neighbors = {
        Vector2D(current.x + 1, current.y), Vector2D(current.x - 1, current.y),
        Vector2D(current.x, current.y + 1), Vector2D(current.x, current.y - 1)
    };

    for (Vector2D next : neighbors) {
        if (next.x >= 0 && next.y >= 0 && next.x < maze->getNumCellX() && next.y < maze->getNumCellY()) {
            if (maze->isValidCell(next) && !bfs_visited[(int)next.y][(int)next.x]) {
                search_visualizer->addToFrontier(next);
                bfs_visited[(int)next.y][(int)next.x] = true;
                bfs_came_from[(int)next.y][(int)next.x] = current;
            }
        }
    }
    return false;
}

void ScenePathFindingMouse::drawMaze() {
    SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
    SDL_Rect rect;
    Vector2D coords;
    for (int j = 0; j < maze->getNumCellY(); j++) {
        for (int i = 0; i < maze->getNumCellX(); i++) {
            if (!maze->isValidCell(Vector2D((float)i, (float)j))) {
                coords = maze->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
                rect = { (int)coords.x, (int)coords.y, CELL_SIZE, CELL_SIZE };
                SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect);
            }
        }
    }
}

void ScenePathFindingMouse::drawCoin() {
    Vector2D coin_coords = maze->cell2pix(coinPosition);
    int offset = CELL_SIZE / 2;
    SDL_Rect dstrect = { (int)coin_coords.x - offset, (int)coin_coords.y - offset, CELL_SIZE, CELL_SIZE };
    SDL_RenderCopy(TheApp::Instance()->getRenderer(), coin_texture, NULL, &dstrect);
}

bool ScenePathFindingMouse::loadTextures(char* filename_bg, char* filename_coin) {
    SDL_Surface* image = IMG_Load(filename_bg);
    if (!image) return false;
    background_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);
    SDL_FreeSurface(image);

    image = IMG_Load(filename_coin);
    if (!image) return false;
    coin_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);
    SDL_FreeSurface(image);

    return true;
}
