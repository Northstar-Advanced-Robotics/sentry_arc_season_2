#ifndef UART_PROTOCOL_HPP
#define UART_PROTOCOL_HPP

#include <cstdint>
namespace uart {

static constexpr std::size_t BAUD_RATE = 115'200;

#define PACKED __attribute__((packed))

enum MessageType : uint16_t {
  TURRET_AIM_DATA = 1,
  ROBOT_ID = 2,
  ALIVE = 3,
  ODOMETRY = 4,
  AUTO_PATH = 5,
  // REF_DATA = 6
  HEALTH = 6,
  REF_TURRET_DATA = 7,
  VISION_LOCALIZATION = 8,
  FLY_SKY_DATA = 9,
  VT13_DATA = 10,
  RESTART_DETECTOR = 11
};

struct TurretOdometryData {
  float pitch;
  float yaw;
  float roll;

  // float pitch_vel;
  float yaw_vel;
  // float roll_vel;

} PACKED;

enum class RobotId : uint16_t {
  INVALID = 0,

  RED_HERO = 1,
  RED_ENGINEER = 2,
  RED_SOLDIER_1 = 3,
  RED_SOLDIER_2 = 4,
  RED_SOLDIER_3 = 5,
  RED_DRONE = 6,
  RED_SENTINEL = 7,
  RED_DART = 8,
  RED_RADAR_STATION = 9,

  BLUE_HERO = 101,
  BLUE_ENGINEER = 102,
  BLUE_SOLDIER_1 = 103,
  BLUE_SOLDIER_2 = 104,
  BLUE_SOLDIER_3 = 105,
  BLUE_DRONE = 106,
  BLUE_SENTINEL = 107,
  BLUE_DART = 108,
  BLUE_RADAR_STATION = 109
};

struct ChassisOdometryData {
  // float pos_x;
  // float pos_y;
  // float pos_z;

  float vel_x;
  float vel_y;
  // float vel_z;

} PACKED;

struct OdometryData {
  uint32_t timestamp;
  ChassisOdometryData chassis_data;
  TurretOdometryData turret_data;
} PACKED;

struct AprilTagLocalizationData {
  float posX;
  float posY;
  float heading;
  uint32_t timestamp;
} PACKED;

struct TurretAimData {
  float yaw;
  float pitch;
  float distance;
  RobotId robotId;
  float maxErrorYaw;
  float maxErrorPitch;
};

} // namespace uart
#undef PACKED
#endif // #ifndef UART_PROTOCOL_HPP
