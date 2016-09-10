#include "simple_torque_controller/torque_controller.h"
#include <pluginlib/class_list_macros.h>
#include <string>
//#include <mutex>

namespace simple_torque_controller {


/// Controller initialization in non-realtime
bool SimpleTorqueController::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{

    frame_count_ = 0;
    frames_per_pub_ = 50;

    robot_ = robot;
    ROS_INFO("torque controller initialized!");
    new_torque_ = false;
    writing_new_torque_ = false;
    reading_new_torque_ = false;

    root_name_ = "base_footprint";
    tip_name_ = "l_gripper_tool_frame";

    // hard coded for left arm
    torso_lift_joint_name_ = "torso_lift_joint";
    joint_names_.push_back("l_shoulder_pan_joint");
    joint_names_.push_back("l_shoulder_lift_joint");
    joint_names_.push_back("l_upper_arm_roll_joint");
    joint_names_.push_back("l_elbow_flex_joint");
    joint_names_.push_back("l_forearm_roll_joint");
    joint_names_.push_back("l_wrist_flex_joint");
    joint_names_.push_back("l_wrist_roll_joint");


    if(!arm_chain_.init(robot_, root_name_, tip_name_)) {
        ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name_.c_str(), tip_name_.c_str());
        return false;
    }

    arm_chain_.toKDL(arm_fk_chain_);
    arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(arm_fk_chain_));
    full_joint_array_.resize(8);

    torso_lift_joint_state_ = robot_->getJointState(torso_lift_joint_name_);

    for (int i=0; i<7; i++){
        pr2_mechanism_model::JointState* joint_state = robot_->getJointState(joint_names_[i]);
        joint_states_.push_back(joint_state);
        current_torque_vector_.push_back(0.0);
        new_torque_vector_.push_back(0.0);
    }

    obs_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(n, "/rllab_obs", 1));
    while(!obs_publisher_->trylock());
    // position, velocity, and end effector pos
    for (int i = 0; i<7*2 + 3; i++)
    {
        obs_publisher_->msg_.data.push_back(0.0);
    }
    obs_publisher_->unlock();


    torque_subscriber_ = n.subscribe("/rllab_torque", 1, &SimpleTorqueController::torque_subscriber_callback, this);


    return true;
}

void SimpleTorqueController::torque_subscriber_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // write into new torque buffer if the main loop is not currently reading it
    // possible to miss a msg but this way we don't need c++11 with mutexes (which pr2 doesn't have)
    if (!reading_new_torque_)
    {
        writing_new_torque_ = true;
        for (int i=0; i<7; i++)
        {
            new_torque_vector_[i] = msg->data[i];
        }
        new_torque_ = true;
        writing_new_torque_ = false;
    }
    else 
    {
        ROS_INFO_STREAM("missed writing a torque");
    }
}

/// Controller startup in realtime
void SimpleTorqueController::starting()
{
    new_torque_ = false;
    writing_new_torque_ = false;
    reading_new_torque_ = false;
    for (int i=0; i<7; i++){
        current_torque_vector_[i] = 0.0;
    }
}


/// Controller update loop in realtime
void SimpleTorqueController::update()
{
    // get new torque if one exists with boolean locks
    if (new_torque_) 
    {
        if (!writing_new_torque_)
        {
            reading_new_torque_ = true;
            for (int i=0; i<7; i++)
            {
                current_torque_vector_[i] = new_torque_vector_[i];
            }
            new_torque_ = false;
            reading_new_torque_ = false;
        }

    }

    for (int i=0; i<7; i++)
    {
        joint_states_[i]->commanded_effort_ = current_torque_vector_[i];

        // debug joint torque and names
        //ROS_INFO_STREAM(joint_names_[i]);
        //ROS_INFO_STREAM(current_torque_vector_[i]);
    }

    full_joint_array_(0) = torso_lift_joint_state_->position_;
    for (int i = 0; i<7; i++)
    {
        full_joint_array_(i + 1) = joint_states_[i]->position_;
    }
    arm_fk_solver_->JntToCart(full_joint_array_, output_frame_);

    if (frame_count_ == frames_per_pub_ - 1)
    {
        while(!obs_publisher_->trylock());

        for (int i = 0; i<7; i++)
        {
            obs_publisher_->msg_.data[i]     = full_joint_array_(i + 1);
            obs_publisher_->msg_.data[i + 7] = joint_states_[i]->velocity_;
        }
        for (int i = 0; i<3; i++)
        {
            obs_publisher_->msg_.data[i + 14] = output_frame_.p[i];
        }

        obs_publisher_->unlockAndPublish();
    }
    frame_count_ = (frame_count_ + 1) % frames_per_pub_;

}


/// Controller stopping in realtime
void SimpleTorqueController::stopping()
{}
} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(simple_torque_controller, SimpleTorqueController, 
                         simple_torque_controller::SimpleTorqueController, 
                         pr2_controller_interface::Controller)
