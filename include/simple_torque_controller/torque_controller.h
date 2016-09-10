#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "std_msgs/Float64MultiArray.h"
#include <realtime_tools/realtime_publisher.h>

namespace simple_torque_controller{

class SimpleTorqueController: public pr2_controller_interface::Controller
{
private:
    pr2_mechanism_model::Chain arm_chain_;
    pr2_mechanism_model::RobotState* robot_;
    KDL::Chain arm_fk_chain_;
    boost::shared_ptr<KDL::ChainFkSolverPos> arm_fk_solver_;
    std::vector<pr2_mechanism_model::JointState*> joint_states_;
    std::string torso_lift_joint_name_;
    pr2_mechanism_model::JointState* torso_lift_joint_state_;
    std::vector<std::string> joint_names_;

    KDL::JntArray full_joint_array_;
    KDL::Frame output_frame_;

    boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > obs_publisher_;

    std::vector<double> current_torque_vector_;
    std::vector<double> new_torque_vector_;
    std::string root_name_;
    std::string tip_name_;
    bool new_torque_;
    bool writing_new_torque_;
    bool reading_new_torque_;

    int frames_per_pub_;
    int frame_count_;
  

    ros::Subscriber torque_subscriber_;

public:
    virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
    virtual void starting();
    virtual void update();
    virtual void stopping();

    virtual void torque_subscriber_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
};
} 

