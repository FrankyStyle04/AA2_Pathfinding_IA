#pragma once
#include "FSMstate.h"

class FSMstate_chase : FSMstate
{
public:
    FSMstate* Update(Agent* agent, float dtime) override;
};
