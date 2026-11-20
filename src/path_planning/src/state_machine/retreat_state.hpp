#ifndef RETREAT_STATE_H
#define RETREAT_STATE_H

#include "state_machine.hpp"
#include "vector2.hpp"

class RetreatState : StateMachineState {
public:
    vector2<float> get_desired_position(GameState& state) {
        return state.blue_team ? blue_alliance_supply_zone : red_alliance_supply_zone;
    }
    
    StateMachineState* try_transition(GameState& state) {
        return state.current_health == state.max_health ? healing_state : nullptr;
    }

private:
    vector2<float> blue_alliance_supply_zone = vector2<float>(0, 0);
    vector2<float> red_alliance_supply_zone = vector2<float>(0, 0);

    StateMachineState* healing_state;
};

#endif
