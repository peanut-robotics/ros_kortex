#include "hardware_interface.h"

using namespace kortex_hardware_interface;

KortexHardwareInterface::KortexHardwareInterface(BaseClient* pBase, BaseCyclicClient* pBaseCyclic) : m_base(pBase), m_basecyclic(pBaseCyclic)
{
    std::cout << "hello achille" << std::endl;
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

//    ros::Time::init();  // not in a ROS node so need to initialize
    last_time = ros::Time::now();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    controller_manager::ControllerManager cm(&*this);

    ros::Rate loop_rate(40);
    while (ros::ok())
    {
        this->read();
        cm.update(this->get_time(), this->get_period());
        this->write();
        loop_rate.sleep();
    }
}

void KortexHardwareInterface::read()
{
    Feedback current_state;
    current_state = m_basecyclic->RefreshFeedback();
    for(int i = 0; i < current_state.actuators_size(); i++)
    {
        pos[i] = static_cast<double>(- current_state.actuators(i).position()/180.0*3.1415);
        vel[i] = static_cast<double>(- current_state.actuators(i).velocity()/180.0*3.1415);
        eff[i] = static_cast<double>(current_state.actuators(i).torque());
    }
}

void KortexHardwareInterface::write()
{
    auto action = Action();
    action.set_name("regular velocity write");
    action.set_application_data("");

    auto jointSpeeds = action.mutable_send_joint_speeds();

    for(size_t i = 0 ; i < NDOF; ++i)
    {
        auto jointSpeed = jointSpeeds->add_joint_speeds();
        jointSpeed->set_joint_identifier(i);
        jointSpeed->set_value(- cmd[i]*180.0/3.1415);
        jointSpeed->set_duration(0.0);
    }

    m_base->SendJointSpeedsCommmand(*jointSpeeds);
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
