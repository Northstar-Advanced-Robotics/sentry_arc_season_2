#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP

#include <memory>
#include <vector>
#include <cstdlib>
#include <ctime>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Correct TF2 headers for ROS 2 Jazzy
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "auto_drive/path_outline.hpp"
#include "auto_drive/theta_star.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

namespace src::viz
{

enum class AutoDriveState
{
    GoingToBase,
    GoingToMid,
    InMid
};

enum class MidCycleType
{
    Mid,
    Top,
    Bottom
};

enum class TravelType
{
    Ramp,
    Bend,
    Bump
};

class RosVisualizer : public rclcpp::Node
{
public:
    RosVisualizer() : Node("path_planner_visualizer")
    {
        // Initialize TF2 Buffer and Listener
        tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("planner/map", 1);

        control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planner/control_points", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("planner/world_velocity_target", 10);

        robotid_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "uart/robotid",
            10,
            std::bind(&RosVisualizer::robotid_callback, this, std::placeholders::_1));

        health_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "uart/health",
            10,
            std::bind(&RosVisualizer::health_callback, this, std::placeholders::_1));

        thetaStar = new src::AutoPathing::ThetaStar();
        lastGridPublishTime = this->now();

        odom_timer_ = this->create_wall_timer(33ms, std::bind(&RosVisualizer::transform_callback, this));

        currentRobotWorldPos = vector2<float>(0, 0);

        std::srand(std::time(0));
    }

    void transform_callback()
    {
        geometry_msgs::msg::TransformStamped transform;

        try
        {
            transform = tf2_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            double x_offset = transform.transform.translation.x;
            double y_offset = transform.transform.translation.y;

            currentRobotWorldPos = vector2<float>(x_offset, y_offset);
            hasRobotPos = true;

            RCLCPP_DEBUG(
                this->get_logger(),
                "Robot Pos Updated: X: %f, Y: %f",
                currentRobotWorldPos.x,
                currentRobotWorldPos.y);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Could not get transform: %s",
                ex.what());
        }
    }

    void publishVelocityData(vector2<float> velocity, rclcpp::Time& currentTime) {
        geometry_msgs::msg::Vector3 msg;
        msg.set__x(velocity.x);
        msg.set__y(velocity.y);
        msg.set__z(0.0);

        velocity_pub_->publish(msg);
    }

    void publishGrid(rclcpp::Time& currentTime)
    {
        int gridSizeX = thetaStar->GetSizeX();
        int gridSizeY = thetaStar->GetSizeY();
        float realSizeX = thetaStar->GetRealSizeX();

        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header.stamp = currentTime;
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = realSizeX / gridSizeX;
        grid_msg.info.width = gridSizeX;
        grid_msg.info.height = gridSizeY;
        grid_msg.info.origin.position.x = -realSizeX / 2.0f;
        grid_msg.info.origin.position.y = -thetaStar->GetRealSizeY() / 2.0f;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
        for (int y = 0; y < gridSizeY; ++y)
        {
            for (int x = 0; x < gridSizeX; ++x)
            {
                int val = (int)thetaStar->getWorkingGrid(x, y);
                int index = x + y * gridSizeX;
                if (val == 0)
                    grid_msg.data[index] = 0;
                else if (val == 1)
                    grid_msg.data[index] = 100;
                else if (val == 2)
                    grid_msg.data[index] = 80;
                else if (val == 3)
                    grid_msg.data[index] = 60;
                else
                    grid_msg.data[index] = 20;
            }
        }
        grid_pub_->publish(grid_msg);
        lastGridPublishTime = currentTime;
    }

    void update()
    {
        hasRobotPos = true;
        if (!hasRobotPos)
        {
            return;
        }

        rclcpp::Time currentTime = this->now();
        thetaStar->updateDynamicObstacles();
        currentWorldGoalPos = calculateWorldGoalPos();
        currentGridGoalPos = thetaStar->ConvertWorldToGrid(currentWorldGoalPos);

        vector2<int> currentRobotGridPos = thetaStar->ConvertWorldToGrid({ currentRobotWorldPos.x, currentRobotWorldPos.y });
        if (thetaStar->isNodeValidAndEmpty(currentRobotGridPos)) { lastValidRobotGridPos = currentRobotGridPos; }

        currentWorldGoalVelocity = calculateVelocityToNextNode();
        publishVelocityData(currentWorldGoalPos, currentTime);
        
        if ((currentTime - lastGridPublishTime).seconds() > GRID_PUB_WAIT)
        {
            this->publishGrid(currentTime);
        }
    }

    vector2<float> calculateWorldGoalPos()
    {
        float healthPercentage = (float)currentHealth / (float)MAX_HEALTH;

        if (healthPercentage < 0.55f)
        {
            currentState = AutoDriveState::GoingToBase;
            try_start_path(GetRetreatPath());
        }
        else
        {
            if (currentState == AutoDriveState::GoingToBase && healthPercentage > 0.99f)
            {
                currentState = AutoDriveState::GoingToMid;
                try_start_path(GetPushPath());
            }

            if (currentState == AutoDriveState::GoingToMid && currentPathOutline->isPathFinished())
            {
                currentState = AutoDriveState::InMid;
                try_start_path(GetCyclePath());
            }
        }

        if (currentPathOutline == nullptr)
        {
            currentState = AutoDriveState::GoingToBase;
            try_start_path(GetRetreatPath());
        }
        return currentPathOutline->evaluatePath(currentRobotWorldPos);
    }

    vector2<float> calculateVelocityToNextNode() {
        std::vector<vector2<int>> fullGridPath = thetaStar->FindPath(lastValidRobotGridPos, currentGridGoalPos);
        return thetaStar->ConvertGridToWorld(fullGridPath[1].convert<float>() - currentRobotWorldPos).normalized();
    }
    
private:
    const MidCycleType currentMidCycleType = MidCycleType::Mid;
    const TravelType currentPushType = TravelType::Bend;
    const TravelType currentRetreatType = TravelType::Bend;

    const float GRID_PUB_WAIT = 3.0f;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_pub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robotid_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr health_sub_;

    int32_t currentHealth = 400;
    const int32_t MAX_HEALTH = 400;

    void robotid_callback(const std_msgs::msg::Int32::SharedPtr msg) { leftTeam = msg->data < 100; }
    void health_callback(const std_msgs::msg::Int32::SharedPtr msg) { currentHealth = msg->data; }

    void try_start_path(PathOutline* pathToStart)
    {
        if (currentPathOutline != pathToStart)
        {
            currentPathOutline = pathToStart;
            currentPathOutline->startPath();
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    src::AutoPathing::ThetaStar* thetaStar;

    rclcpp::Time lastGridPublishTime;

    vector2<float> currentWorldGoalPos;
    vector2<int> currentGridGoalPos;

    vector2<float> currentWorldGoalVelocity;

    bool hasRobotPos = false;
    vector2<float> currentRobotWorldPos;
    vector2<int> lastValidRobotGridPos;

    PathOutline* currentPathOutline = nullptr;
    AutoDriveState currentState = AutoDriveState::GoingToBase;

    bool leftTeam = true;

    const vector2<float> LEFT_ALL_ZONE = vector2<float>(-5.3f, 0.0f);
    const vector2<float> RIGHT_ALL_ZONE = vector2<float>(5.3f, 0.0f);

    int currentDemoIndex = 0;
    const vector2<float> DEMO_POINTS[4] = {
       vector2<float>(-0.45f, -3.0f),
       vector2<float>(-0.45f, 0.0f),
       vector2<float>(3.0f, -3.0f),
       vector2<float>(3.0f, -3.0f),
    };
    
    static constexpr float MID_SIZE = 0.6f;
    PathOutline* MID_CYCLE_PATH = new PathOutline(
        {vector2<float>(-MID_SIZE, MID_SIZE),
         vector2<float>(MID_SIZE, -MID_SIZE),
         vector2<float>(-MID_SIZE, -MID_SIZE),
         vector2<float>(MID_SIZE, MID_SIZE)},
        true);

    // CYCLE
    PathOutline* LEFT_MID_BOTTOM_CYCLE_PATH =
        new PathOutline({vector2<float>(-1.7f, -1.47f), vector2<float>(-2.02f, -0.71f)}, true);
    PathOutline* LEFT_MID_TOP_CYCLE_PATH =
        new PathOutline({vector2<float>(-0.6f, 1.82f), vector2<float>(-1.55f, 1.31f)}, true);
    // LEFT TO MID
    PathOutline* LEFT_TO_MID_RAMP_PATH = new PathOutline(
        {vector2<float>(-4.16f, 2.54f),
         vector2<float>(-0.93f, 2.54f),
         vector2<float>(-0.82f, 1.33f)},
        false);
    PathOutline* LEFT_TO_MID_BEND_PATH =
        new PathOutline({vector2<float>(-4.64f, 1.54f), vector2<float>(-1.88f, -0.69f)}, false);
    PathOutline* LEFT_TO_MID_BUMP_PATH =
        new PathOutline({vector2<float>(-4.17f, -2.58f), vector2<float>(-1.64f, -1.54f)}, false);
    // MID TO LEFT
    PathOutline* MID_TO_LEFT_RAMP_PATH = new PathOutline(
        {vector2<float>(-0.82f, 1.33f),
         vector2<float>(-0.93f, 2.54f),
         vector2<float>(-4.16f, 2.54f),
         LEFT_ALL_ZONE},
        false);
    PathOutline* MID_TO_LEFT_BEND_PATH = new PathOutline(
        {vector2<float>(-1.88f, -0.69f), vector2<float>(-4.64f, 1.54f), LEFT_ALL_ZONE},
        false);
    PathOutline* MID_TO_LEFT_BUMP_PATH = new PathOutline(
        {vector2<float>(-1.64f, -1.54f), vector2<float>(-4.17f, -2.58f), LEFT_ALL_ZONE},
        false);

    // CYCLE
    PathOutline* RIGHT_MID_BOTTOM_CYCLE_PATH =
        new PathOutline({vector2<float>(1.7f, -1.47f), vector2<float>(2.02f, -0.71f)}, true);
    PathOutline* RIGHT_MID_TOP_CYCLE_PATH =
        new PathOutline({vector2<float>(0.6f, 1.82f), vector2<float>(1.55f, 1.31f)}, true);
    // RIGHT TO MID
    PathOutline* RIGHT_TO_MID_RAMP_PATH = new PathOutline(
        {vector2<float>(4.16f, 2.54f),
         vector2<float>(0.93f, 2.54f),
         vector2<float>(0.82f, 1.33f)},
        false);
    PathOutline* RIGHT_TO_MID_BEND_PATH =
        new PathOutline({vector2<float>(4.64f, 1.54f), vector2<float>(1.88f, -0.69f)}, false);
    PathOutline* RIGHT_TO_MID_BUMP_PATH =
        new PathOutline({vector2<float>(4.17f, -2.58f), vector2<float>(1.64f, -1.54f)}, false);
    // MID TO RIGHT
    PathOutline* MID_TO_RIGHT_RAMP_PATH = new PathOutline(
        {vector2<float>(0.82f, 1.33f),
         vector2<float>(0.93f, 2.54f),
         vector2<float>(4.16f, 2.54f),
         RIGHT_ALL_ZONE},
        false);
    PathOutline* MID_TO_RIGHT_BEND_PATH = new PathOutline(
        {vector2<float>(1.88f, -0.69f), vector2<float>(4.64f, 1.54f), RIGHT_ALL_ZONE},
        false);
    PathOutline* MID_TO_RIGHT_BUMP_PATH = new PathOutline(
        {vector2<float>(1.64f, -1.54f), vector2<float>(4.17f, -2.58f), RIGHT_ALL_ZONE},
        false);

    PathOutline* GetCyclePath()
    {
        switch (currentMidCycleType)
        {
            case MidCycleType::Top:
                return leftTeam ? LEFT_MID_TOP_CYCLE_PATH : RIGHT_MID_TOP_CYCLE_PATH;
            case MidCycleType::Bottom:
                return leftTeam ? LEFT_MID_BOTTOM_CYCLE_PATH : RIGHT_MID_BOTTOM_CYCLE_PATH;
            default:
                return MID_CYCLE_PATH;
        }
    }

    PathOutline* GetPushPath()
    {
        switch (currentPushType)
        {
            case TravelType::Bump:
                return leftTeam ? LEFT_TO_MID_BUMP_PATH : RIGHT_TO_MID_BUMP_PATH;
            case TravelType::Bend:
                return leftTeam ? LEFT_TO_MID_BEND_PATH : RIGHT_TO_MID_BEND_PATH;
            default:
                return leftTeam ? LEFT_TO_MID_RAMP_PATH : RIGHT_TO_MID_RAMP_PATH;
        }
    }

    PathOutline* GetRetreatPath()
    {
        switch (currentRetreatType)
        {
            case TravelType::Bump:
                return leftTeam ? MID_TO_LEFT_BUMP_PATH : MID_TO_RIGHT_BUMP_PATH;
            case TravelType::Bend:
                return leftTeam ? MID_TO_LEFT_BEND_PATH : MID_TO_RIGHT_BEND_PATH;
            default:
                return leftTeam ? MID_TO_LEFT_RAMP_PATH : MID_TO_RIGHT_RAMP_PATH;
        }
    }
};

}  // namespace src::viz

#endif
