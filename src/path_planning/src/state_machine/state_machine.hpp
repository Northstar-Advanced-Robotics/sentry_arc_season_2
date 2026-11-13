#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "vector2.hpp"
#include "state_machine_state.hpp"

class StateMachine {
public:
    StateMachine() {
        setup_state_machine();
    }

    vector2<float> update_state_and_get_target_pos() {
        while (true) {
            StateMachineState* tmp = currentState->try_transition();
            if (tmp != nullptr) { currentState = tmp; }
            else { break; }
        }

        return currentState->get_target_pos();
    }

private:
    void setup_state_machine();
    void update_state_machine();

    StateMachineState* currentState = nullptr;
};

#endif
