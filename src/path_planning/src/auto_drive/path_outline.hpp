#include <vector>

#include "vector2.hpp"

class PathOutline
{
public:
    float MAX_POS_ERROR = 0.18f;

    PathOutline(std::vector<vector2<float>> outline, bool repeats)
        : outline(outline),
          repeats(repeats)
    {
    }

    PathOutline() : outline({}), repeats(false) {}

    vector2<float> evaluatePath(vector2<float> currentRobotWorldPos)
    {
        vector2<float> goal = outline[pathIndex];
        float distanceToGoal = currentRobotWorldPos.distance_to(goal);

        if (distanceToGoal <= MAX_POS_ERROR)
        {
            pathIndex++;
            if (pathIndex >= outline.size())
            {
                if (repeats)
                {
                    pathIndex = 0;
                }
                else
                {
                    pathIndex = outline.size() - 1;
                    pathFinished = true;
                }
            }

            goal = outline[pathIndex];
        }

        return goal;
    }

    void startPath()
    {
        pathIndex = 0;
        pathFinished = false;
    }

    bool isPathFinished() { return pathFinished; }

    int pointCount() { return outline.size(); }

private:
    int pathIndex = 0;
    bool repeats = false;
    bool pathFinished = false;

    std::vector<vector2<float>> outline;
};
