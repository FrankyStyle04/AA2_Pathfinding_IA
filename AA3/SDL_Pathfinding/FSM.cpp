#include "FSM.h"

void FSM::update(Agent* agent, float dtime)
{
    FSMstate* newState = currentState->Update(agent, dtime);
    if (newState != nullptr)
    {
        currentState->Exit(agent, dtime);
        ChangeState(newState);
        currentState->Enter(agent, dtime);
    }
}

void FSM::ChangeState(FSMstate* newState)
{
    currentState = newState;
}
