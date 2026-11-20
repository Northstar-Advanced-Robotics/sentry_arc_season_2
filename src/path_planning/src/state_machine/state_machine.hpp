#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "game_state.hpp"
#include "vector2.hpp"

class StateMachineState {
public:
    virtual vector2<float> get_desired_position(GameState& state) = 0;
    virtual StateMachineState* try_transition(GameState& state) = 0;
};

class StateMachine {
public:
    StateMachine();
    vector2<float> update_get_desired_position(GameState& state) {
        while (true) {
            StateMachineState* next_state = current_state->try_transition(state);
            if (next_state == nullptr) { break; }
            else { current_state = next_state; }
        }

        return current_state->get_desired_position(state);
    }

private:
    StateMachineState* current_state;
};

#endif
