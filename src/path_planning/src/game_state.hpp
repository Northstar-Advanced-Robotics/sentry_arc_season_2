#ifndef GAME_STATE_H
#define GAME_STATE_H

#include "vector2.hpp"

struct GameState {
    bool blue_team;
    int max_health;
    int current_health;
    vector2<float> current_robot_pos;
};

#endif
