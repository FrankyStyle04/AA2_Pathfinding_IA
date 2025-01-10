#pragma once
#include "FSMstate.h"

class FMSstate_patrol : public FSMstate
{
public:
    FSMstate* Update(Agent* agent, float dtime) override;
};
