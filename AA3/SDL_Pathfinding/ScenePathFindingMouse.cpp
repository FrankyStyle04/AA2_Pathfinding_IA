#include "ScenePathFindingMouse.h"

#pragma region START

ScenePathFindingMouse::ScenePathFindingMouse() {
    draw_grid = false;
    grid = new Grid("../res/maze.csv");

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
    while (!grid->isValidCell(rand_cell)) {
        rand_cell = Vector2D((float)(rand() % grid->getNumCellX()), (float)(rand() % grid->getNumCellY()));
    }
    agents[0]->setPosition(grid->cell2pix(rand_cell));

    // Colocar la moneda en una celda aleatoria válida
    coinPosition = Vector2D(-1, -1);
    while (!grid->isValidCell(coinPosition) || (Vector2D::Distance(coinPosition, rand_cell) < 3)) {
        coinPosition = Vector2D((float)(rand() % grid->getNumCellX()), (float)(rand() % grid->getNumCellY()));
    }

    // Crear el visualizador de búsqueda
    search_visualizer = new SearchVisualizer(grid);
}

ScenePathFindingMouse::~ScenePathFindingMouse() {
    if (background_texture) SDL_DestroyTexture(background_texture);
    if (coin_texture) SDL_DestroyTexture(coin_texture);
    for (Agent* agent : agents) delete agent;
    delete search_visualizer;
}

const char* ScenePathFindingMouse::getTitle() {
    return "SDL Path Finding :: BFS";
}

#pragma endregion

#pragma region BFS LOGIC

void ScenePathFindingMouse::update(float dtime, SDL_Event* event)
{
    if (event->type == SDL_MOUSEBUTTONDOWN && !isStarted && !isClicking)
    {
        Vector2D clickedCell = grid->pix2cell(Vector2D((float)(event->button.x), (float)(event->button.y)));
        
        if (grid->isValidCell(clickedCell))
        {
            Vector2D startCell = grid->pix2cell(agents[0]->getPosition());
            StartBestFirstSearch(startCell, clickedCell);
            clickedTarget = grid->cell2pix(clickedCell);
            isClicking = true;
        }
    }

    if (isStarted && SDL_GetTicks() - bfsStepTime > bfsDelay)
    {
        bfsStepTime = SDL_GetTicks();

        //Si encuentra el objetivo
        if (StepBestFirstSearch())
        {
            //Hacemos el camino del agente con el cameFrom
            std::vector<Vector2D> path;
            for (Vector2D step = target; step != Vector2D(-1, -1); step = cameFrom[(int)step.y][(int)step.x])
            {
                path.push_back(grid->cell2pix(step));
            }
            
            std::reverse(path.begin(), path.end());
            agents[0]->clearPath();
            
            for (Vector2D point : path)
            {
                agents[0]->addPathPoint(point);
            }
            isStarted = false;
        }
    }
    else if (!isStarted)
    {
        agents[0]->update(dtime, event);
        if (grid->pix2cell(agents[0]->getPosition()) == grid->pix2cell(clickedTarget))
        {
            search_visualizer->reset();
            isClicking = false;
        }
    }
}

void ScenePathFindingMouse::StartBestFirstSearch(Vector2D start, Vector2D goal) {
    //Chequeamos que las celdas estén contenidas en la grid
    if (!grid->isValidCell(start) || !grid->isValidCell(goal)) return;
    isStarted = true;
    target = goal;

    //Restart the visual last search
    search_visualizer->reset();

    //Marcar las celdas como visitadas o no visitadas
    visitedNodes = std::vector<std::vector<bool>>(grid->getNumCellY(), std::vector<bool>(grid->getNumCellX(), false));

    //Guardamos las posiciones de las que vienes para poder reconstruir el camino
    cameFrom = std::vector<std::vector<Vector2D>>(grid->getNumCellY(), std::vector<Vector2D>(grid->getNumCellX(), Vector2D(-1, -1)));

    visitedNodes[(int)start.y][(int)start.x] = true;
    
    search_visualizer->addToFrontier(start);
}

bool ScenePathFindingMouse::StepBestFirstSearch() {
    //Si no hay frontera devolvemos false
    if (search_visualizer->isFrontierEmpty()) return false;
    
    Vector2D current = search_visualizer->popFrontier();

    //Si el buscado es el target devolvemos true
    if (current == target) return true;

    //Miramos los vecinos
    std::vector<Vector2D> neighbors = {
        Vector2D(current.x + 1, current.y), Vector2D(current.x - 1, current.y),
        Vector2D(current.x, current.y + 1), Vector2D(current.x, current.y - 1)
    };

    for (Vector2D next : neighbors)
    {
        if (next.x >= 0 && next.y >= 0 && next.x < grid->getNumCellX() && next.y < grid->getNumCellY())
        {
            if (grid->isValidCell(next) && !visitedNodes[(int)next.y][(int)next.x])
            {
                search_visualizer->addToFrontier(next);
                visitedNodes[(int)next.y][(int)next.x] = true;
                cameFrom[(int)next.y][(int)next.x] = current;
            }
        }
    }
    return false;
}

#pragma endregion

#pragma region DRAWABLE

void ScenePathFindingMouse::draw() {
    drawMaze();
    drawCoin();
    for (Vector2D cell : search_visualizer->getDynamicFrontier()) {
        draw_circle(TheApp::Instance()->getRenderer(), (int)cell.x, (int)cell.y, 10, 255, 0, 0, 255);
    }
    agents[0]->draw();
}

void ScenePathFindingMouse::drawMaze() {
    SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
    SDL_Rect rect;
    Vector2D coords;
    for (int j = 0; j < grid->getNumCellY(); j++) {
        for (int i = 0; i < grid->getNumCellX(); i++) {
            if (!grid->isValidCell(Vector2D((float)i, (float)j))) {
                coords = grid->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
                rect = { (int)coords.x, (int)coords.y, CELL_SIZE, CELL_SIZE };
                SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect);
            }
        }
    }
}

void ScenePathFindingMouse::drawCoin() {
    Vector2D coin_coords = grid->cell2pix(coinPosition);
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

#pragma endregion

