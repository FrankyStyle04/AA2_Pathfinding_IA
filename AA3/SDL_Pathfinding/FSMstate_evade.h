#pragma once
#include "FSMstate.h"

class FSMstate_evade : public FSMstate
{
public:
    FSMstate* Update(Agent* agent, float dtime) override;
};
