#include "ScenePathFindingMouse.h"


using namespace std;

ScenePathFindingMouse::ScenePathFindingMouse()
{
	draw_grid = false;
	maze = new Grid("../res/maze.csv");

	loadTextures("../res/maze.png", "../res/coin.png");

	srand((unsigned int)time(NULL));

	Agent* agent = new Agent;
	agent->loadSpriteTexture("../res/soldier.png", 4);
	agent->setBehavior(new PathFollowing);
	agent->setTarget(Vector2D(-20, -20));
	agents.push_back(agent);

	// set agent position coords to the center of a random cell
	Vector2D rand_cell(-1, -1);
	while (!maze->isValidCell(rand_cell))
		rand_cell = Vector2D((float)(rand() % maze->getNumCellX()), (float)(rand() % maze->getNumCellY()));
	agents[0]->setPosition(maze->cell2pix(rand_cell));

	// set the coin in a random cell (but at least 3 cells far from the agent)
	coinPosition = Vector2D(-1, -1);
	while ((!maze->isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, rand_cell) < 3))
		coinPosition = Vector2D((float)(rand() % maze->getNumCellX()), (float)(rand() % maze->getNumCellY()));

}

ScenePathFindingMouse::~ScenePathFindingMouse()
{
	if (background_texture)
		SDL_DestroyTexture(background_texture);
	if (coin_texture)
		SDL_DestroyTexture(coin_texture);

	for (int i = 0; i < (int)agents.size(); i++)
	{
		delete agents[i];
	}
}

void ScenePathFindingMouse::update(float dtime, SDL_Event* event)
{
	/* Keyboard & Mouse events */
	switch (event->type) {
	case SDL_KEYDOWN:
		if (event->key.keysym.scancode == SDL_SCANCODE_SPACE)
			draw_grid = !draw_grid;
		break;
	case SDL_MOUSEMOTION:
	case SDL_MOUSEBUTTONDOWN:
		if (event->button.button == SDL_BUTTON_LEFT) {
			Vector2D clickedCell = maze->pix2cell(Vector2D((float)(event->button.x), (float)(event->button.y)));
			if (maze->isValidCell(clickedCell)) {
				// Calcula el camino usando BFS
				Vector2D startCell = maze->pix2cell(agents[0]->getPosition());
				std::vector<Vector2D> path = BFS(maze, startCell, clickedCell);

				// Actualiza el camino del agente
				agents[0]->clearPath();
				for (Vector2D point : path) {
					agents[0]->addPathPoint(point);
				}
			}
		}
		break;
	default:
		break;
	}

	agents[0]->update(dtime, event);

	// if we have arrived to the coin, replace it in a random cell!
	if ((agents[0]->getCurrentTargetIndex() == -1) && (maze->pix2cell(agents[0]->getPosition()) == coinPosition))
	{
		coinPosition = Vector2D(-1, -1);
		while ((!maze->isValidCell(coinPosition)) || (Vector2D::Distance(coinPosition, maze->pix2cell(agents[0]->getPosition())) < 3))
			coinPosition = Vector2D((float)(rand() % maze->getNumCellX()), (float)(rand() % maze->getNumCellY()));
	}

}

void ScenePathFindingMouse::draw()
{
	drawMaze();
	drawCoin();

	if (draw_grid)
	{
		SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 255, 255, 255, 127);
		for (int i = 0; i < SRC_WIDTH; i += CELL_SIZE)
		{
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), i, 0, i, SRC_HEIGHT);
		}
		for (int j = 0; j < SRC_HEIGHT; j = j += CELL_SIZE)
		{
			SDL_RenderDrawLine(TheApp::Instance()->getRenderer(), 0, j, SRC_WIDTH, j);
		}
	}

	agents[0]->draw();
}

const char* ScenePathFindingMouse::getTitle()
{
	return "SDL Path Finding :: PathFinding Mouse Demo";
}

void ScenePathFindingMouse::drawMaze()
{
	SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
	SDL_Rect rect;
	Vector2D coords;
	for (int j = 0; j < maze->getNumCellY(); j++)
	{
		for (int i = 0; i < maze->getNumCellX(); i++)
		{
			if (!maze->isValidCell(Vector2D((float)i, (float)j)))
			{
				SDL_SetRenderDrawColor(TheApp::Instance()->getRenderer(), 0, 0, 255, 255);
				coords = maze->cell2pix(Vector2D((float)i, (float)j)) - Vector2D((float)CELL_SIZE / 2, (float)CELL_SIZE / 2);
				rect = { (int)coords.x, (int)coords.y, CELL_SIZE, CELL_SIZE };
				SDL_RenderFillRect(TheApp::Instance()->getRenderer(), &rect);
			}
			else {
				// Do not draw if it is not necessary (bg is already black)
			}


		}
	}
	//Alternative: render a backgroud texture:
	//SDL_RenderCopy(TheApp::Instance()->getRenderer(), background_texture, NULL, NULL );
}

void ScenePathFindingMouse::drawCoin()
{
	Vector2D coin_coords = maze->cell2pix(coinPosition);
	int offset = CELL_SIZE / 2;
	SDL_Rect dstrect = { (int)coin_coords.x - offset, (int)coin_coords.y - offset, CELL_SIZE, CELL_SIZE };
	SDL_RenderCopy(TheApp::Instance()->getRenderer(), coin_texture, NULL, &dstrect);
}


bool ScenePathFindingMouse::loadTextures(char* filename_bg, char* filename_coin)
{
	SDL_Surface* image = IMG_Load(filename_bg);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	background_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	if (image)
		SDL_FreeSurface(image);

	image = IMG_Load(filename_coin);
	if (!image) {
		cout << "IMG_Load: " << IMG_GetError() << endl;
		return false;
	}
	coin_texture = SDL_CreateTextureFromSurface(TheApp::Instance()->getRenderer(), image);

	if (image)
		SDL_FreeSurface(image);

	return true;
}

std::vector<Vector2D> ScenePathFindingMouse::BFS(Grid* grid, Vector2D start, Vector2D goal) {
	std::queue<Vector2D> frontier;
	frontier.push(start);

	// Matriz para rastrear nodos visitados
	std::vector<std::vector<bool>> visited(grid->getNumCellY(), std::vector<bool>(grid->getNumCellX(), false));
	visited[(int)start.y][(int)start.x] = true;

	// Matriz para rastrear el camino
	std::vector<std::vector<Vector2D>> came_from(grid->getNumCellY(), std::vector<Vector2D>(grid->getNumCellX(), Vector2D(-1, -1)));

	while (!frontier.empty()) {
		Vector2D current = frontier.front();
		frontier.pop();

		if (current == goal)
			break;

		// Obtener vecinos v�lidos
		std::vector<Vector2D> neighbors = {
			Vector2D(current.x + 1, current.y),
			Vector2D(current.x - 1, current.y),
			Vector2D(current.x, current.y + 1),
			Vector2D(current.x, current.y - 1) };

		for (Vector2D next : neighbors) {
			if (grid->isValidCell(next) && !visited[(int)next.y][(int)next.x]) {
				frontier.push(next);
				visited[(int)next.y][(int)next.x] = true;
				came_from[(int)next.y][(int)next.x] = current;
			}
		}
	}

	// Reconstruir el camino
	std::vector<Vector2D> path;
	if (came_from[(int)goal.y][(int)goal.x] == Vector2D(-1, -1)) {
		std::cout << "No se encontr� un camino v�lido." << std::endl;
		return path; // Devuelve un camino vac�o si no hay conexi�n
	}

	for (Vector2D step = goal; step != start; step = came_from[(int)step.y][(int)step.x]) {
		path.push_back(grid->cell2pix(step));
	}
	path.push_back(grid->cell2pix(start));
	std::reverse(path.begin(), path.end());
	return path;
}



