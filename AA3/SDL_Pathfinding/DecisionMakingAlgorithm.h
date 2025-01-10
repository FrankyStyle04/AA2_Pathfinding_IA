#pragma once
#include "Agent.h"

class DecisionMakingAlgorithm
{
public:
    virtual void update(Agent* agent, float dtime) abstract;
};
