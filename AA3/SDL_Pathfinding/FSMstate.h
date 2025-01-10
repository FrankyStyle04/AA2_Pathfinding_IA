#pragma once
#include "Agent.h"

class FSMstate
{
public:
    void Enter(Agent* agent, float dtime);
    void Exit(Agent* agent, float dtime);
    virtual FSMstate* Update(Agent* agent, float dtime) abstract;;    
};
