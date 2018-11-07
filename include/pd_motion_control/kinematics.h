#ifndef PD_CONTROL_KINEMATIC_CHAIN_H_
#define PD_CONTROL_KINEMATIC_CHAIN_H_

// ROS
#include <ros/ros.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_kdl.h>

// MoveIt!
#include <moveit_msgs/KinematicSolverInfo.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// Boost
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

// PD_CONTROL
#include <pd_motion_control/RobotState.h>
#include <pd_motion_control/ComputeFK.h>
#include <pd_motion_control/ComputeJacobian.h>

//#include <trac_ik/trac_ik.hpp>

class KinematicChain
{
  public:
    /** @brief KinematicChain: constructor of the class */
    KinematicChain(const ros::NodeHandle node_handle) : nh_(node_handle)
    {
        ;
    }

    /** @brief KinematicChain: distructor of the class */
    ~KinematicChain()
    {
        ;
    }

    bool initialize(const std::string& robot_description = "robot_description", const std::string& base_frame = "",
                    const std::string& tip_frame = "");

    bool getPositionFK(std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                       std::vector<geometry_msgs::Pose>& poses);

    bool getPositionJacobian(const std::vector<double>& joint_angles, KDL::Jacobian& jac_matrix);

    bool getPositionIK(const std::vector<double>& joint_angles, const geometry_msgs::PoseStamped& eef_pose,
                       std::vector<double>& return_joints);

    bool computeFKCallBack(pd_motion_control::ComputeFK::Request& request,
                           pd_motion_control::ComputeFK::Response& response);

    bool computeJacobianCallBack(pd_motion_control::ComputeJacobian::Request& request,
                                 pd_motion_control::ComputeJacobian::Response& response);

  private:
    const std::vector<std::string>& getJointNames() const;

    const std::vector<std::string>& getLinkNames() const;

    const moveit_msgs::KinematicSolverInfo& getKinematicChainInfo() const;

    int getJointIndex(const std::string& name) const;

    int getSegmentIndex(const std::string& name) const;

    /** data member */
    ros::NodeHandle nh_;

    ros::ServiceServer compute_fk_srv_; /** service server for compute fk service */

    ros::ServiceServer compute_jac_srv_; /** service server for compute jacobian service */

    int dimension_; /** segment/joints into kinematic chain */

    moveit_msgs::KinematicSolverInfo fk_chain_info_; /** Store information for the forward kinematics solver */

    KDL::Chain kdl_chain_;

    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

    // KDL::JntArray joint_min_, joint_max_; /** Joint limits */

    // bool position_ik_; /** whether this solver is only being used for position ik*/

    // urdf::ModelInterfaceConstSharedPtr urdf_model_; /** urdf model */
};

#endif
