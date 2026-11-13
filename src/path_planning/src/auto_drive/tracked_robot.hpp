#ifndef TRACKED_ROBOT_HPP
#define TRACKED_ROBOT_HPP

#include "vector2.hpp"

namespace src::AutoPathing
{

class TrackedRobot
{
public:
    vector2<int> position;
    const int nodeRadius;

    TrackedRobot(vector2<int> pos, int radius) : position(pos), nodeRadius(radius) {}

    inline bool isExpired(int64_t currentTimeMS) const
    {
        return (currentTimeMS - expireTimeMS) > EXPIRE_TIMER_MS;
    }

    inline void refreshExpireTimer(int64_t currentTimeMS)
    {
        expireTimeMS = currentTimeMS + EXPIRE_TIMER_MS;
    }

private:
    static constexpr int64_t EXPIRE_TIMER_MS = 5000;
    int64_t expireTimeMS;
};

}  // namespace src::AutoPathing

#endif
