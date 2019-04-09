#include "hardware_interface.h"

using namespace kortex_hardware_interface;

KortexHardwareInterface::KortexHardwareInterface(BaseClient* pBase, BaseCyclicClient* pBaseCyclic) : m_base(pBase), m_basecyclic(pBaseCyclic)
{
    for (std::size_t i = 0; i < NDOF; ++i)
    {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle);

        // connect and register the joint velocity interface
        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_names[i]), &cmd[i]);
        jnt_vel_interface.registerHandle(vel_handle);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

    last_time = ros::Time::now();

    cm = new controller_manager::ControllerManager(&*this);

    // don't continue until ros control is up so we don't write stray commands
    ROS_DEBUG("Waiting for the controller spawner to be up...");
    ros::service::waitForService("/controller_spawner/get_loggers");
    ROS_DEBUG("Found controller spawner.");

}

void KortexHardwareInterface::update()
{
    cm->update(this->get_time(), this->get_period());
}

void KortexHardwareInterface::read()
{
    Feedback current_state;
    current_state = m_basecyclic->RefreshFeedback();
    for(int i = 0; i < current_state.actuators_size(); i++)
    {
        pos[i] = static_cast<double>(- current_state.actuators(i).position()/180.0*M_PI);
        vel[i] = static_cast<double>(- current_state.actuators(i).velocity()/180.0*M_PI);
        eff[i] = static_cast<double>(current_state.actuators(i).torque());
    }
}

void KortexHardwareInterface::write()
{
    if(cmd != vel) // only write when commanded velocity != current velocity
    {
        auto action = Action();
        action.set_name("regular velocity write");
        action.set_application_data("");

        auto jointSpeeds = action.mutable_send_joint_speeds();

        for(size_t i = 0 ; i < NDOF; ++i)
        {
            auto jointSpeed = jointSpeeds->add_joint_speeds();
            jointSpeed->set_joint_identifier(i);
            jointSpeed->set_value(- cmd[i]*180.0/M_PI);
            jointSpeed->set_duration(0.0);
        }

        try
        {
            m_base->SendJointSpeedsCommmand(*jointSpeeds);
        }
        catch (KDetailedException& ex)
        {
            ROS_WARN_THROTTLE(1, "Kortex exception");
            ROS_WARN_THROTTLE(1, "KINOVA exception error code: %d\n", ex.getErrorInfo().getError().error_code());
            ROS_WARN_THROTTLE(1, "KINOVA exception error sub code: %d\n", ex.getErrorInfo().getError().error_sub_code());
            ROS_WARN_THROTTLE(1, "KINOVA exception description: %s\n", ex.what());
        }
        catch (std::runtime_error& ex2)
        {
            ROS_INFO("Other Kortex exception");
        }

    }
}

ros::Time KortexHardwareInterface::get_time()
{
    return ros::Time::now();
}

ros::Duration KortexHardwareInterface::get_period()
{
    ros::Time current_time = ros::Time::now();
    ros::Duration period = current_time - last_time;
    last_time = current_time;
    return period;
}

KortexHardwareInterface::~KortexHardwareInterface() {}
