#ifndef STATE_MACHINE_STATE_HPP
#define STATE_MACHINE_STATE_HPP

#include "vector2.hpp"

class StateMachineState {
public:
    virtual StateMachineState* try_transition() = 0;
    virtual vector2<float> get_target_pos() = 0;
};

#endif
