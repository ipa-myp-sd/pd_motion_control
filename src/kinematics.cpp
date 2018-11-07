
#include <pd_motion_control/kinematics.h>
#include <algorithm>

bool KinematicChain::initialize(const std::string& robot_description, const std::string& base_frame,
                                const std::string& tip_frame)
{
    ros::WallTime start = ros::WallTime::now();

    // get urdf model from robot description
    urdf::ModelInterfaceConstSharedPtr urdf_model_; /** urdf model */
    urdf::Model* umodel = new urdf::Model();
    if (!umodel->initParam("/robot_description"))  // || (!umodel->initString("/robot_description"))
    {
        ROS_ERROR_NAMED("KinematicChain::initialize", "Unable to parse URDF from parameter '%s'",
                        robot_description.c_str());
        return false;
    }

    urdf_model_.reset(umodel);

    ROS_DEBUG_STREAM_NAMED("KinematicChain::initialize", "Loaded robot model in "
                                                             << (ros::WallTime::now() - start).toSec() << " seconds");

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree))
    {
        ROS_ERROR_NAMED("KinematicChain::initialize", "Could not initialize tree object");
        return false;
    }

    if (!kdl_tree.getChain(base_frame, tip_frame, kdl_chain_))
    {
        ROS_ERROR_NAMED("KinematicChain::initialize", "Could not initialize chain object");
        return false;
    }

    dimension_ = kdl_chain_.getNrOfSegments();
    ROS_WARN_STREAM("Segments: " << kdl_chain_.getNrOfSegments() << "\t Jonits: " << kdl_chain_.getNrOfJoints());

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    // chain information
    fk_chain_info_.joint_names.clear();
    fk_chain_info_.link_names.clear();
    for (std::size_t i = 0; i < kdl_tree.getNrOfJoints(); i++)
    {
        const std::string& joint_name = kdl_chain_.getSegment(i).getJoint().getName();
        fk_chain_info_.joint_names.push_back(joint_name);

        fk_chain_info_.link_names.push_back(urdf_model_->getJoint(joint_name).get()->child_link_name);
        moveit_msgs::JointLimits limit;
        limit.joint_name = joint_name;

        boost::shared_ptr<urdf::JointLimits> urdf_limit = urdf_model_->getJoint(joint_name).get()->limits;
        limit.has_position_limits = (urdf_limit->lower || urdf_limit->upper) == double(0.) ? false : true;
        limit.has_velocity_limits = urdf_limit->velocity == double(0.) ? false : true;
        limit.has_acceleration_limits = urdf_limit->effort == double(0.) ? false : true;
        limit.min_position = urdf_limit->lower;
        limit.max_position = urdf_limit->upper;
        limit.max_velocity = urdf_limit->velocity;
        limit.max_acceleration = urdf_limit->effort;
        fk_chain_info_.limits.push_back(limit);

        ROS_INFO_STREAM_NAMED("KinematicChain::initialize",
                              "joint name: " << joint_name.c_str() << "  limit.min_position: " << limit.min_position
                                             << "  limit.max_position: " << limit.max_position << "  limit.max_velocity"
                                             << limit.max_velocity);
    }

    compute_fk_srv_ = nh_.advertiseService("ComputeFK", &KinematicChain::computeFKCallBack, this);

    compute_jac_srv_ = nh_.advertiseService("ComputeJacobian", &KinematicChain::computeJacobianCallBack, this);

    ROS_INFO_STREAM_NAMED("KinematicChain::initialize", " KinematicChain::initialize Successful!! ");
    return true;
}

int KinematicChain::getJointIndex(const std::string& name) const
{
    // return (std::find(fk_chain_info_.joint_names.begin(), fk_chain_info_.joint_names.end(), name));
    for (unsigned int i = 0; i < fk_chain_info_.joint_names.size(); i++)
    {
        if (fk_chain_info_.joint_names[i] == name)
            return i;
    }
    return -1;
}

int KinematicChain::getSegmentIndex(const std::string& name) const
{
    int i = 0;
    while (i < (int)kdl_chain_.getNrOfSegments())
    {
        if (kdl_chain_.getSegment(i).getName() == name)
        {
            return i + 1;
        }
        i++;
    }
    return -1;
}

/*bool KinematicChain::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                   std::vector<geometry_msgs::Pose>& poses)*/
bool KinematicChain::getPositionFK(std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                   std::vector<geometry_msgs::Pose>& poses)
{
    ros::WallTime n1 = ros::WallTime::now();

    // if link name is empty than takes as default one
    // Note: make sure link_names varibles going to be change
    link_names = link_names.empty() ? fk_chain_info_.link_names : link_names;

    poses.resize(link_names.size());
    if (joint_angles.size() != dimension_)
    {
        ROS_ERROR_NAMED("KinematicChain::getPositionFK", "Joint angles vector must have size: %d", dimension_);
        return false;
    }

    KDL::Frame p_out;

    KDL::JntArray jnt_pose_in(dimension_);
    for (unsigned int i = 0; i < dimension_; i++)
    {
        jnt_pose_in(i) = joint_angles[i];
    }

    // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

    bool valid = true;
    for (unsigned int i = 0; i < poses.size(); i++)
    {
        ROS_DEBUG_NAMED("KinematicChain::getPositionFK", "End effector index: %d", getSegmentIndex(link_names[i]));
        if (fk_solver_->JntToCart(jnt_pose_in, p_out, getSegmentIndex(link_names[i])) >= 0)
        {
            tf::poseKDLToMsg(p_out, poses[i]);
        }
        else
        {
            ROS_ERROR_NAMED("KinematicChain::getPositionFK", "Could not compute FK for %s", link_names[i].c_str());
            valid = false;
        }
    }
    return valid;
}

bool KinematicChain::computeFKCallBack(pd_motion_control::ComputeFK::Request& request,
                                       pd_motion_control::ComputeFK::Response& response)
{
    ROS_INFO_NAMED("KinematicChain::computeFKCallBack ", "Request receive for FK calculation");
    std::vector<geometry_msgs::Pose> poses;
    std::vector<std::string> link_names =
        request.fk_link_names.empty() ? fk_chain_info_.link_names : request.fk_link_names;

    bool fk_success = getPositionFK(link_names, request.robot_state.position,
                                    poses);  // request.robot_state.joint_state.position

    // todo: can be improve by checking condition of jac_success and jac_matrix empty, avoiding below execution

    response.fk_pose_stamped.clear();  // resize(poses.size());
    response.fk_link_names = link_names;

    if (fk_success == true && !poses.empty())
    {
        for (std::size_t i = 0; i < poses.size(); ++i)
        {
            geometry_msgs::PoseStamped stamped;
            stamped.header = request.header;
            stamped.pose = poses[i];
            response.fk_pose_stamped.push_back(stamped);
            ROS_DEBUG_STREAM_NAMED("KinematicChain::computeFKCallBack ", "stamped: " << stamped);
        }

        response.eef_pose_stamped = response.fk_pose_stamped[poses.size() - 1];
    }

    response.message = (poses.empty() || fk_success == false) ? "computation of FK failed!, compute empty poses" :
                                                                " computation of FK "
                                                                "successed";
    response.fk_success = (poses.empty() || fk_success == false) ? false : true;

    return response.fk_success;
}

// Assume that we are getting jacobian matrix of one link only
bool KinematicChain::getPositionJacobian(const std::vector<double>& joint_angles, KDL::Jacobian& jac_matrix)
{
    // clear existing entry on the matrix, make sure properly feed data
    jac_matrix = KDL::Jacobian();

    if (joint_angles.size() != dimension_)
    {
        ROS_ERROR_NAMED("KinematicChain::getPositionJacobian", "Joint angles vector must have size: %d", dimension_);
        return false;
    }

    KDL::Jacobian jac_out(dimension_);

    KDL::JntArray jnt_pose_in(dimension_);
    for (unsigned int i = 0; i < dimension_; i++)
    {
        jnt_pose_in(i) = joint_angles[i];
    }

    ROS_DEBUG_STREAM_NAMED("KinematicChain::getPositionJacobian", " Joint angle: " << jnt_pose_in.data);
    bool valid = true;
    if (jac_solver_->JntToJac(jnt_pose_in, jac_out) >= 0)
    {
        jac_matrix = jac_out;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("KinematicChain::getPositionJacobian", "Could not compute Jacobian with joint angle"
                                                                          << jnt_pose_in.data);
        valid = false;
    }
    return valid;
}

// In the service result, we can send only one link information. Therefore, we assume that we get jacobian matrix with
// only one link
bool KinematicChain::computeJacobianCallBack(pd_motion_control::ComputeJacobian::Request& request,
                                             pd_motion_control::ComputeJacobian::Response& response)
{
    ROS_INFO_NAMED("KinematicChain::computeJacobianCallBack ", "Request receive for Jacobian calculation");
    KDL::Jacobian jac_matrix;

    bool jac_success =
        getPositionJacobian(request.robot_state.position, jac_matrix);  // request.robot_state.joint_state.position

    ROS_DEBUG_STREAM_NAMED("KinematicChain::computeJacobianCallBack", "Jacobian Matrix: \n" << jac_matrix.data);

    response.jac_linear_vel.clear();   // resize(dimension_);
    response.jac_angular_vel.clear();  // resize(dimension_);

    // jacobian calculation is succeed than store data into response else it is empty
    if (jac_success == true && ((jac_matrix.columns() || jac_matrix.rows()) != 0))
    {
        // feed velocity componet from jacobian
        for (std::size_t dof = 0; dof < dimension_; ++dof)
        {
            // get linear velocity data from jac matrix
            geometry_msgs::Vector3 lin_vel;
            lin_vel.x = jac_matrix.data(0, dof);
            lin_vel.y = jac_matrix.data(1, dof);
            lin_vel.z = jac_matrix.data(2, dof);
            response.jac_linear_vel.push_back(lin_vel);

            // get angular velocity data from jac matrix
            geometry_msgs::Vector3 ang_vel;
            ang_vel.x = jac_matrix.data(3, dof);
            ang_vel.y = jac_matrix.data(4, dof);
            ang_vel.z = jac_matrix.data(5, dof);
            response.jac_angular_vel.push_back(ang_vel);
        }
    }

    response.message = ((jac_matrix.columns() || jac_matrix.rows()) == 0) ?
                           "computation of Jacobian failed!, compute empty jacobian" :
                           " computation of Jacobian successed";
    response.jac_success = jac_success;

    return jac_success;
}

bool KinematicChain::getPositionIK(const std::vector<double>& joint_angles, const geometry_msgs::PoseStamped& eef_pose,
                                   std::vector<double>& return_joints)
{
    /*
      ros::WallTime n1 = ros::WallTime::now();

      return_joints.resize(joint_angles.size());
      if (joint_angles.size() != dimension_)
      {
          ROS_ERROR_NAMED("KinematicChain::getPositionIK", "Joint angles vector must have size: %d", dimension_);
          return false;
      }

      KDL::JntArray jnt_pose_in(dimension_);
      for (unsigned int i = 0; i < dimension_; i++)
      {
          jnt_pose_in(i) = joint_angles[i];
      }

      KDL::JntArray lower_joint_limits(dimension_);
      KDL::JntArray upper_joint_limits(dimension_);
      for (unsigned int i = 0; i < dimension_; i++)
      {
          lower_joint_limits(i) = fk_chain_info_.limits[i].min_position;
          upper_joint_limits(i) = fk_chain_info_.limits[i].max_position;
      }

      KDL::Frame eef_pose_in;
      tf::poseMsgToKDL(eef_pose.pose, eef_pose_in);

      KDL::JntArray jnt_pose_out(dimension_);

      KDL::Twist tolerance;
      tolerance.rot.x(0.005);
      tolerance.rot.y(0.005);
      tolerance.rot.z(0.005);

      tolerance.vel.x(0.005);
      tolerance.vel.y(0.005);
      tolerance.vel.z(0.005);

      TRAC_IK::TRAC_IK ik_solver(kdl_chain_, lower_joint_limits, upper_joint_limits);
      // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);

      bool valid = true;
      ROS_DEBUG_STREAM_NAMED("KinematicChain::getPositionIK", "End effector pose: "<< eef_pose);
      if (ik_solver.CartToJnt(jnt_pose_in, eef_pose_in, jnt_pose_out, tolerance) >= 0)
      {
        // convert kdl vector to std::vector
        for (unsigned int i = 0; i < dimension_; i++)
        {
            return_joints[i] = jnt_pose_out(i);
        }
        ROS_DEBUG_STREAM_NAMED("KinematicChain::getPositionIK"," Successfully compute IK with new joint angles: "<<
      jnt_pose_out.data);

      }
      else
      {
          ROS_ERROR_STREAM_NAMED("KinematicChain::getPositionIK", "Could not compute IK for " << jnt_pose_in.data);
          valid = false;
      }
      return valid;*/
}

const std::vector<std::string>& KinematicChain::getJointNames() const
{
    return fk_chain_info_.joint_names;
}

const std::vector<std::string>& KinematicChain::getLinkNames() const
{
    return fk_chain_info_.link_names;
}

const moveit_msgs::KinematicSolverInfo& KinematicChain::getKinematicChainInfo() const
{
    return fk_chain_info_;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    std::string base_frame, tip_frame, rbt_description;

    if (!nh.searchParam("/robot_description", rbt_description) || !nh.getParam("/robot_description", rbt_description))
    {
        ROS_WARN_NAMED("kinematic_chain::Main", "Robot model parameter not found! Did you remap '%s'?",
                       rbt_description.c_str());
        // return -1;
    }

    if (!nh.searchParam("/arm/base_link", base_frame) || !nh.getParam("/arm/base_link", base_frame))
    {
        ROS_ERROR_NAMED("kinematic_chain::Main", "base link parameter not found! '%s'?", base_frame.c_str());
        return -1;
    }

    if (!nh.searchParam("/arm/tip_link", tip_frame) || !nh.getParam("/arm/tip_link", tip_frame))
    {
        ROS_ERROR_NAMED("kinematic_chain::Main", "tip link parameter not found! '%s'?", tip_frame.c_str());
        return -1;
    }

    boost::scoped_ptr<KinematicChain> kinematic_chain_;
    kinematic_chain_.reset(new KinematicChain(nh));

    kinematic_chain_->initialize(rbt_description, base_frame, tip_frame);
    ros::spin();

    return 0;
}
