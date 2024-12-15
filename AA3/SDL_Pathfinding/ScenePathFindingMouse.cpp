#include "ScenePathFindingMouse.h"
#include <map>

#pragma region START

ScenePathFindingMouse::ScenePathFindingMouse() {
	draw_grid = false;
	grid = new Grid("../res/maze.csv", "../res/NodeWeights.csv");

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
	while (coinsPositions.size() < 5) {
		Vector2D coinPosition = Vector2D(-1, -1);
		while (!grid->isValidCell(coinPosition)) {
			coinPosition = Vector2D((float)(rand() % grid->getNumCellX()), (float)(rand() % grid->getNumCellY()));
		}

		// Verificar que no esté demasiado cerca de otras monedas ya generadas
		bool tooClose = false;
		for (const Vector2D& existingCoin : coinsPositions) {
			if (Vector2D::Distance(coinPosition, existingCoin) < 3) {
				tooClose = true;
				break;
			}
		}

		if (!tooClose) {
			coinsPositions.push_back(coinPosition); // Añadir la posición al vector de monedas
		}
	}
	enemy = new Enemy(Vector2D(7, 6), Vector2D(15, 6), 1.f); // Se mueve cada 0.5 segundos
	enemy2 = new Enemy(Vector2D(23, 14), Vector2D(15, 14), 1.f); // Se mueve cada 0.5 segundos
	enemy3 = new Enemy(Vector2D(15, 20), Vector2D(15, 20), 1.f); // Se mueve cada 0.5 segundos

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

#pragma region UPDATE

void ScenePathFindingMouse::update(float dtime, SDL_Event* event) {
	if (event->type == SDL_KEYDOWN) {
		switch (event->key.keysym.sym) {
		case SDLK_b:
			currentAlgorithm = BFS;
			std::cout << "Cambiado a BFS." << std::endl;
			break;
		case SDLK_d:
			currentAlgorithm = DIJKSTRA;
			std::cout << "Cambiado a Dijkstra." << std::endl;
			break;
		case SDLK_a:
			currentAlgorithm = A;
			std::cout << "Cambiado a A*." << std::endl;
			break;
		case SDLK_g:
			currentAlgorithm = GBFS;
			std::cout << "Cambiado a GBFS." << std::endl;
			break;
		}
	}

	enemy->update(dtime);
	enemy2->update(dtime);
	enemy3->update(dtime);

	// Actualizar los pesos en la grilla
	Vector2D enemyPos = enemy->getPosition();
	grid->updateNodeWeights(enemyPos, 4, 50); // Rango de influencia y peso máximo

	Vector2D enemyPos2 = enemy2->getPosition();
	grid->updateNodeWeights(enemyPos2, 4, 50); 

	Vector2D enemyPos3 = enemy3->getPosition();
	grid->updateNodeWeights(enemyPos3, 4, 50);

	if (event->type == SDL_MOUSEBUTTONDOWN && !isStarted && !isClicking) {
		Vector2D clickedCell = grid->pix2cell(Vector2D((float)(event->button.x), (float)(event->button.y)));

		if (grid->isValidCell(clickedCell)) {
			Vector2D startCell = grid->pix2cell(agents[0]->getPosition());
			clickedTarget = grid->cell2pix(clickedCell);
			isClicking = true;

			if (currentAlgorithm == BFS) {
				BFSAlgorithm(startCell, clickedCell);
			} else if (currentAlgorithm == DIJKSTRA) {
				DijkstraAlgorithm(startCell, clickedCell);
			} else if (currentAlgorithm == A) {
				AStarAlgorithm(startCell, clickedCell);
			} else if (currentAlgorithm == GBFS) {
				GBFSAlgorithm(startCell, clickedCell);
			}
		}
	}

	if (currentAlgorithm == BFS && isStarted && SDL_GetTicks() - bfsStepTime > bfsDelay) {
		bfsStepTime = SDL_GetTicks();

		if (StepBestFirstSearch()) {
			std::cout << "Camino encontrado con BFS." << std::endl;

			std::vector<Vector2D> path;
			for (Vector2D step = target; step != Vector2D(-1, -1); step = cameFrom[(int)step.y][(int)step.x]) {
				path.push_back(grid->cell2pix(step));
			}

			std::reverse(path.begin(), path.end());
			agents[0]->clearPath();

			for (Vector2D point : path) {
				agents[0]->addPathPoint(point);
			}

			isStarted = false;
		}
	}

	if (currentAlgorithm == DIJKSTRA && isDijkstraRunning && SDL_GetTicks() - bfsStepTime > bfsDelay) {
		bfsStepTime = SDL_GetTicks();
		if (StepDijkstra()) {
			std::cout << "Camino encontrado con Dijkstra." << std::endl;
			
			std::vector<Vector2D> path;
			for (Vector2D step = dijkstraGoal; step != dijkstraCameFrom[step]; step = dijkstraCameFrom[step]) {
				path.push_back(grid->cell2pix(step));
			}
			std::reverse(path.begin(), path.end());

			agents[0]->clearPath();
			for (Vector2D point : path) {
				agents[0]->addPathPoint(point);
			}
			isDijkstraRunning = false;
		}
	}

	if (currentAlgorithm == A && isAStarRunning && SDL_GetTicks() - bfsStepTime > bfsDelay) {
		bfsStepTime = SDL_GetTicks();
		if (StepA()) {
			std::cout << "Camino encontrado con A*." << std::endl;
			// Reconstrucción del camino
			std::vector<Vector2D> path;
			for (Vector2D step = aStarGoal; step != aStarCameFrom[step]; step = aStarCameFrom[step]) {
				path.push_back(grid->cell2pix(step));
			}
			std::reverse(path.begin(), path.end());

			agents[0]->clearPath();
			for (Vector2D point : path) {
				agents[0]->addPathPoint(point);
			}
			isAStarRunning = false;
		}
	}

	if (currentAlgorithm == GBFS && isGBFSRunning && SDL_GetTicks() - bfsStepTime > bfsDelay) {
		bfsStepTime = SDL_GetTicks();
		if (StepGBFS()) {
			std::cout << "Camino encontrado con GBFS." << std::endl;
			// Reconstrucción del camino
			std::vector<Vector2D> path;
			for (Vector2D step = gbfsGoal; step != gbfsCameFrom[step]; step = gbfsCameFrom[step]) {
				path.push_back(grid->cell2pix(step));
			}
			std::reverse(path.begin(), path.end());

			agents[0]->clearPath();
			for (Vector2D point : path) {
				agents[0]->addPathPoint(point);
			}

			isGBFSRunning = false;
		}
	}
	
	agents[0]->update(dtime, event);
	if (!isStarted && !isDijkstraRunning && grid->pix2cell(agents[0]->getPosition()) == grid->pix2cell(clickedTarget)) {
		search_visualizer->reset();
		isClicking = false;
	}

	for (size_t i = 0; i < coinsPositions.size(); ++i) {
		if ((agents[0]->getCurrentTargetIndex() == -1) && (grid->pix2cell(agents[0]->getPosition()) == coinsPositions[i])) {
			// Moneda recogida, eliminarla del vector
			coinsPositions.erase(coinsPositions.begin() + i);

			if (!coinsPositions.empty()&&findAllCoins) {
				// Encontrar la moneda más cercana
				Vector2D currentPosition = grid->pix2cell(agents[0]->getPosition());
				Vector2D closestCoin = coinsPositions[0];
				float minDistance = heuristicManhattan(currentPosition, closestCoin);

				for (const Vector2D& coin : coinsPositions) {
					float distance = heuristicManhattan(currentPosition, coin);
					if (distance < minDistance) {
						closestCoin = coin;
						minDistance = distance;
					}
				}

				// Iniciar A* hacia la moneda más cercana
				AStarAlgorithm(currentPosition, closestCoin);
			}
			break; // Salimos del bucle porque hemos modificado el vector
		}
	}
}

#pragma endregion

#pragma region BFS LOGIC

void ScenePathFindingMouse::BFSAlgorithm(Vector2D start, Vector2D goal) {
	if (!grid->isValidCell(start) || !grid->isValidCell(goal)) return;
	isStarted = true;
	target = goal;

	search_visualizer->reset();

	visitedNodes = std::vector<std::vector<bool>>(grid->getNumCellY(), std::vector<bool>(grid->getNumCellX(), false));
	cameFrom = std::vector<std::vector<Vector2D>>(grid->getNumCellY(), std::vector<Vector2D>(grid->getNumCellX(), Vector2D(-1, -1)));
	visitedNodes[(int)start.y][(int)start.x] = true;

	search_visualizer->addToFrontier(start);
}

bool ScenePathFindingMouse::StepBestFirstSearch() {
	
	if (search_visualizer->isFrontierEmpty()) {
		isStarted = false;
		return false;
	}

	Vector2D current = search_visualizer->popFrontier();

	if (current == target) {
		isStarted = false;
		return true;
	}

	std::vector<Vector2D> neighbors = {
		Vector2D(current.x + 1, current.y), Vector2D(current.x - 1, current.y),
		Vector2D(current.x, current.y + 1), Vector2D(current.x, current.y - 1)
	};

	for (Vector2D next : neighbors) {
		if (next.x >= 0 && next.y >= 0 && next.x < grid->getNumCellX() && next.y < grid->getNumCellY()) {
			if (grid->isValidCell(next) && !visitedNodes[(int)next.y][(int)next.x]) {
				search_visualizer->addToFrontier(next);
				visitedNodes[(int)next.y][(int)next.x] = true;
				cameFrom[(int)next.y][(int)next.x] = current;
			}
		}
	}

	return false;
}

#pragma endregion

#pragma region DIJKSTRA LOGIC

void ScenePathFindingMouse::DijkstraAlgorithm(Vector2D start, Vector2D goal) {
	
	if (!grid->isValidCell(start) || !grid->isValidCell(goal)) {
		std::cerr << "Error: Las celdas inicial o final no son válidas." << std::endl;
		isDijkstraRunning = false;
		return;
	}
	
	search_visualizer->reset();
	while (!dijkstraFrontier.empty()) dijkstraFrontier.pop();
	dijkstraCameFrom.clear();
	dijkstraCostSoFar.clear();

	dijkstraFrontier.emplace(0.0f, start);
	dijkstraCameFrom[start] = start;
	dijkstraCostSoFar[start] = 0.0f;

	search_visualizer->addToFrontier(start);

	dijkstraGoal = goal;
	isDijkstraRunning = true;  // Activamos Dijkstra
}

bool ScenePathFindingMouse::StepDijkstra() {
	
	if (dijkstraFrontier.empty()) {
		isDijkstraRunning = false;
		return false;
	}

	Vector2D current = dijkstraFrontier.top().second;
	dijkstraFrontier.pop();

	if (current == dijkstraGoal) {
		isDijkstraRunning = false;
		return true;
	}
	
	for (Vector2D next : grid->getNeighbors(current)) {
		float newCost = dijkstraCostSoFar[current] + grid->getCost(current, next);
		if (dijkstraCostSoFar.find(next) == dijkstraCostSoFar.end() || newCost < dijkstraCostSoFar[next]) {
			dijkstraCostSoFar[next] = newCost;
			dijkstraFrontier.emplace(newCost, next);
			dijkstraCameFrom[next] = current;

			search_visualizer->addToFrontier(next);
		}
	}
	return false;
}

#pragma endregion

#pragma region A* LOGIC

void ScenePathFindingMouse::AStarAlgorithm(Vector2D start, Vector2D goal)
{
	if (!grid->isValidCell(start) || !grid->isValidCell(goal)) {
		std::cerr << "Error: Las celdas inicial o final no son válidas." << std::endl;
		isAStarRunning = false;
		return;
	}

	search_visualizer->reset();
	while (!aStarFrontier.empty()) aStarFrontier.pop();
	aStarCameFrom.clear();
	aStarCostSoFar.clear();

	aStarFrontier.emplace(0.0f, start);
	aStarCameFrom[start] = start;
	aStarCostSoFar[start] = 0.0f;

	search_visualizer->addToFrontier(start);

	aStarGoal = goal;
	isAStarRunning = true;  // Activamos A*
}

bool ScenePathFindingMouse::StepA()
{
	if (aStarFrontier.empty()) {
		isAStarRunning = false;  // Finalizamos si no hay más nodos por explorar
		return false;
	}

	Vector2D current = aStarFrontier.top().second;
	aStarFrontier.pop();

	if (current == aStarGoal) {
		isAStarRunning = false;
		return true;
	}

	// Explorar vecinos
	for (Vector2D next : grid->getNeighbors(current)) {
		float newCost = aStarCostSoFar[current] + grid->getCost(current, next);
		if (aStarCostSoFar.find(next) == aStarCostSoFar.end() || newCost < aStarCostSoFar[next]) {
			aStarCostSoFar[next] = newCost;

			float priority = newCost + heuristicManhattan(aStarGoal, next);
			aStarFrontier.emplace(priority, next);
			aStarCameFrom[next] = current;

			search_visualizer->addToFrontier(next);
		}
	}
	return false;
}

float ScenePathFindingMouse::heuristicManhattan(Vector2D goal, Vector2D next)
{
	return fabs(goal.x - next.x) + fabs(goal.y - next.y);  // Distancia Manhattan
}

#pragma endregion

#pragma region GBFS LOGIC

void ScenePathFindingMouse::GBFSAlgorithm(Vector2D start, Vector2D goal) {
	if (!grid->isValidCell(start) || !grid->isValidCell(goal)) {
		std::cerr << "Error: Las celdas inicial o final no son válidas." << std::endl;
		isGBFSRunning = false;
		return;
	}

	search_visualizer->reset();
	while (!gbfsFrontier.empty()) gbfsFrontier.pop();
	gbfsCameFrom.clear();

	gbfsFrontier.emplace(0.0f, start);
	gbfsCameFrom[start] = start;

	search_visualizer->addToFrontier(start);

	gbfsGoal = goal;
	isGBFSRunning = true;  // Activamos GBFS
}

bool ScenePathFindingMouse::StepGBFS()
{
	if (gbfsFrontier.empty()) {
		isGBFSRunning = false;  // Finalizamos si no hay más nodos por explorar
		return false;
	}

	Vector2D current = gbfsFrontier.top().second;
	gbfsFrontier.pop();

	if (current == gbfsGoal) {
		isGBFSRunning = false;
		return true;
	}

	// Explorar vecinos
	for (Vector2D next : grid->getNeighbors(current)) {
		if (gbfsCameFrom.find(next) == gbfsCameFrom.end()) {  // Evitar reexplorar nodos
			float priority = heuristicManhattan(gbfsGoal, next);  // Solo se usa la heurística
			gbfsFrontier.emplace(priority, next);
			gbfsCameFrom[next] = current;

			search_visualizer->addToFrontier(next);
		}
	}
	return false;
}

#pragma endregion

#pragma region DRAWABLE

void ScenePathFindingMouse::draw() {
	drawMaze();
	drawCoin();
	// Dibujar al enemigo
	Vector2D enemyPos = grid->cell2pix(enemy->getPosition());
	int offset = CELL_SIZE / 2;
	SDL_Rect rect = { (int)enemyPos.x - offset, (int)enemyPos.y - offset, CELL_SIZE, CELL_SIZE };
	SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 0, 0, 255); // Rojo
	SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect);

	Vector2D enemyPos2 = grid->cell2pix(enemy2->getPosition());
	int offset2 = CELL_SIZE / 2;
	SDL_Rect rect2 = { (int)enemyPos2.x - offset2, (int)enemyPos2.y - offset2, CELL_SIZE, CELL_SIZE };
	SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 0, 0, 255); // Rojo
	SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect2);

	Vector2D enemyPos3 = grid->cell2pix(enemy3->getPosition());
	int offset3 = CELL_SIZE / 2;
	SDL_Rect rect3 = { (int)enemyPos3.x - offset3, (int)enemyPos3.y - offset3, CELL_SIZE, CELL_SIZE };
	SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 0, 0, 255); // Rojo
	SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect3);


	for (Vector2D cell : search_visualizer->getDynamicFrontier()) {
		draw_circle(TheApp::Instance()->getRenderer(), (int)cell.x, (int)cell.y, 10, 255, 0, 0, 255);
	}
	agents[0]->draw();
}

void ScenePathFindingMouse::drawMaze() {
	SDL_Rect rect;
	Vector2D coords;
	for (int j = 0; j < grid->getNumCellY(); j++) {
		for (int i = 0; i < grid->getNumCellX(); i++) {
			if (!grid->isValidCell(Vector2D((float)i, (float)j))) {
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 5, 5, 5, 255);
				coords = grid->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
				rect = { (int)coords.x, (int)coords.y, CELL_SIZE, CELL_SIZE };
				SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect);
			}
			else {
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 130, 211,130, 255);
				coords = grid->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
				rect = { (int)coords.x, (int)coords.y, CELL_SIZE, CELL_SIZE };
				SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect);
			}
		}
	}



	SDL_Rect rect2;
	Vector2D coords2;
	for (int j = 0; j < grid->getNumCellY(); j++) {
		for (int i = 0; i < grid->getNumCellX(); i++) {

			switch (grid->getNode(i, j)->getWeight()) {
			case 13:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 97, 175, 97, 255);
				coords2 = grid->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
				rect2 = { (int)coords2.x, (int)coords2.y, CELL_SIZE, CELL_SIZE };
				SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect2);
				
				break;
			case 25:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 58, 132, 58, 255);
				coords2 = grid->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
				rect2 = { (int)coords2.x, (int)coords2.y, CELL_SIZE, CELL_SIZE };
				SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect2);
			
				break;
			case 38:
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 21, 81,21, 255);
				coords2 = grid->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
				rect2 = { (int)coords2.x, (int)coords2.y, CELL_SIZE, CELL_SIZE };
				SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect2);
				break;
		
			}
			
			
		
		}
	}
}

void ScenePathFindingMouse::drawCoin() {
	for (const Vector2D& coinPosition : coinsPositions) {
		Vector2D coin_coords = grid->cell2pix(coinPosition);
		int offset = CELL_SIZE / 2;
		SDL_Rect dstrect = { (int)coin_coords.x - offset, (int)coin_coords.y - offset, CELL_SIZE, CELL_SIZE };
		SDL_RenderCopy(TheApp::Instance()->getRenderer(), coin_texture, NULL, &dstrect);
	}
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

