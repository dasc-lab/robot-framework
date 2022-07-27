#include <thread>
#include "DASCRobots.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "frame_transforms.h"
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

// Export functions
extern "C" {

    DASCRobot* init(std::string robot_name, uint8_t id) {return new DASCRobot(robot_name, id);}
    bool arm(DASCRobot *robot) {return robot->arm();}
    bool disarm(DASCRobot *robot) {return robot->disarmarm();}
    std::array<double, 3> getWorldPosition(DASCRobot *robot) {return robot->getWorldPosition();}
    std::array<double, 3> getWorldVelocity(DASCRobot *robot) {return robot->getWorldVelocity();}
    std::array<double, 3> getWorldAcceleration(DASCRobot *robot) {return robot->getWorldAcceleration();}
    std::array<double, 3> getBodyAcceleration(DASCRobot *robot) {return robot->getBodyAcceleration();}
    std::array<double, 3> getBodyRate(DASCRobot *robot) {return robot->getBodyRate();}
    bool getBodyQuaternion(DASCRobot *robot, std::array<double, 4>& quat, bool blocking) {return robot->getBodyQuaternion(quat, blocking);}
    bool setCmdMode(DASCRobot *robot, ControlMode mode) {return robot->setCmdMode(mode);}
    bool cmdWorldPosition(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldPosition(x, y, z, yaw, yaw_rate);}
    bool cmdWorldVelocity(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldVelocity(x, y, z, yaw, yaw_rate);}
    bool cmdLocalVelocity(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdLocalVelocity(x, y, z, yaw, yaw_rate);}
    bool cmdWorldAcceleration(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldAcceleration(x, y, z, yaw, yaw_rate);}
    bool cmdAttitude(DASCRobot *robot, double q_w, double q_x, double q_y, double q_z, double thrust) {return robot->cmdAttitude(q_w, q_x, q_y, q_z, thrust);}
    bool cmdRates(DASCRobot *robot, double roll, double pitch, double yaw, double thrust) {return robot->cmdRates(roll, pitch, yaw, thrust);}
    bool cmdOffboardMode(DASCRobot *robot) {return robot->cmdOffboardMode();}
    void emergencyStop(DASCRobot *robot) {return robot->emergencyStop();}
    void setGPSGlobalOrigin(DASCRobot *robot, double lat, double lon, double alt) {return robot->setGPSGlobalOrigin(lat, lon, alt);}

}