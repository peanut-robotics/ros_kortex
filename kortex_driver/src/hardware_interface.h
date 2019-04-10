#ifndef KORTEX_HARDWARE_INTERFACE_H
#define KORTEX_HARDWARE_INTERFACE_H

#include <cmath>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "controller_manager/controller_manager.h"

#include <BaseClientRpc.h>  // kinova
#include <BaseCyclicClientRpc.h>

// ros stuff
#include <ros/ros.h>
#include "ros/time.h"
#include "ros/duration.h"
#include <ros/console.h>

using namespace Kinova::Api;
using namespace Kinova::Api::BaseCyclic;
using namespace Kinova::Api::Base;

namespace kortex_hardware_interface
{
  class KortexHardwareInterface : hardware_interface::RobotHW
  {
  public:
    KortexHardwareInterface(BaseClient* pBase, BaseCyclicClient* pBaseCyclic);
    void read();
    void write();
    void update();
    ros::Time get_time();
    ros::Duration get_period();
    ~KortexHardwareInterface();
  private:
    ros::Time last_time;
    controller_manager::ControllerManager* cm;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    BaseClient* m_base;
    BaseCyclicClient* m_basecyclic;

    std::vector<std::string> joint_names = {"Actuator1", "Actuator2","Actuator3", "Actuator4","Actuator5", "Actuator6","Actuator7"};
    int NDOF = joint_names.size();
    double cmd[7];
    double prev_cmd[7];
    double pos[7];
    double vel[7];
    double eff[7];
  };
}


#endif