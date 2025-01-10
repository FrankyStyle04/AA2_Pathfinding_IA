#pragma once
#include <string>
#include <unordered_map>

#include "Graph.h"
#include "Vector2D.h"

class BlackBoard
{
private:
    std::unordered_map<std::string, void*> actionMap;
    Graph* graph;
public:
    void setFloat(std::string key, float value);
    float getFloat(std::string key);

    void setInt(std::string key, int value);
    int getInt(std::string key);

    void setVector2D(std::string key, Vector2D value);
    Vector2D getVector2D(std::string key);

    void setGraphPtr(std::string key);
    Graph* getGraphPtr();
};
