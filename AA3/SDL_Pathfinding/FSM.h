#pragma once
#include "DecisionMakingAlgorithm.h"
#include "FSMstate.h"

class FSM : public DecisionMakingAlgorithm
{
private:
    FSMstate* currentState;
    void ChangeState(FSMstate* newState);
public:
    void update(Agent* agent, float dtime) override;
};
