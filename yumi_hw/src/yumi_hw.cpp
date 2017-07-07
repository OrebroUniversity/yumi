#include<yumi_hw/yumi_hw.h>

void YumiHW::create(std::string name, std::string urdf_string)
{
    ROS_INFO_STREAM("Creating a Yumi HW interface for: " << name <<" with "<<n_joints_<<" joints");

    // SET NAME AND MODEL
    robot_namespace_ = name;
    urdf_string_ = urdf_string;

    // ALLOCATE MEMORY

    // JOINT NAMES ARE TAKEN FROM URDF NAME CONVENTION
    joint_names_.push_back( robot_namespace_ + std::string("_joint_1_l") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_2_l") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_3_l") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_4_l") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_5_l") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_6_l") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_7_l") );
    
    joint_names_.push_back( robot_namespace_ + std::string("_joint_1_r") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_2_r") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_3_r") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_4_r") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_5_r") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_6_r") );
    joint_names_.push_back( robot_namespace_ + std::string("_joint_7_r") );
    
    // VARIABLES
    joint_position_.resize(n_joints_);
    joint_position_prev_.resize(n_joints_);
    joint_velocity_.resize(n_joints_);
    joint_effort_.resize(n_joints_);
    joint_position_command_.resize(n_joints_);
    joint_velocity_command_.resize(n_joints_);

    joint_lower_limits_.resize(n_joints_);
    joint_upper_limits_.resize(n_joints_);

    // RESET VARIABLES
    reset();

    ROS_INFO("Parsing transmissions from the URDF...");

    // GET TRANSMISSIONS THAT BELONG TO THIS LWR 4+ ARM
    if (!parseTransmissionsFromURDF(urdf_string_))
    {
	ROS_ERROR("Error parsing URDF in yumi_hw.");
	return;
    }

    ROS_INFO("Registering interfaces...");

    const urdf::Model *const urdf_model_ptr = urdf_model_.initString(urdf_string_) ? &urdf_model_ : NULL;
    registerInterfaces(urdf_model_ptr, transmissions_);

    //std::cout << "Initializing KDL variables..." << std::endl;
    // INIT KDL STUFF
    //initKDLdescription(urdf_model_ptr);

    ROS_INFO("Succesfully created an abstract Yumi with interfaces to ROS control");
}

// reset values
void YumiHW::reset()
{
    for (int j = 0; j < n_joints_; ++j)
    {
	joint_position_[j] = 0.0;
	joint_position_prev_[j] = 0.0;
	joint_velocity_[j] = 0.0;
	joint_effort_[j] = 0.0;

	joint_position_command_[j] = 0.0;
	joint_velocity_command_[j] = 0.0;
    }

    current_strategy_ = JOINT_POSITION;

    return;
}


void YumiHW::registerInterfaces(const urdf::Model *const urdf_model,
	std::vector<transmission_interface::TransmissionInfo> transmissions)
{

    // Check that this transmission has one joint
    if( transmissions.empty() )
    {
	ROS_ERROR("No drivable joints in urdf");
	return;
    }

    // Initialize values
    for(int j=0; j < n_joints_; j++)
    {
	// Check that this transmission has one joint
	if(transmissions[j].joints_.size() == 0)
	{
	    ROS_WARN_STREAM("Transmission " << transmissions[j].name_
		<< " has no associated joints." << std::endl);
	    continue;
	}
	else if(transmissions[j].joints_.size() > 1)
	{
	    ROS_WARN_STREAM("Transmission " << transmissions[j].name_
		<< " has more than one joint, and they can't be controlled simultaneously"
		<< std::endl);
	    continue;
	}

	std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

	if( joint_interfaces.empty() )
	{
	    ROS_WARN_STREAM("Joint " << transmissions[j].joints_[0].name_ <<
		" of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
		"You need to, otherwise the joint can't be controlled." << std::endl);
	    continue;
	}

	const std::string& hardware_interface = joint_interfaces.front();

	// Debug //FIXME
	std::cout << "\x1B[37m" << "lwr_hw: " << "Loading joint '" << joint_names_[j]
	    << "' of type '" << hardware_interface << "'" << "\x1B[0m" << std::endl;

	// Create joint state interface for all joints
	state_interface_.registerHandle(hardware_interface::JointStateHandle(
		    joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

	//No control in effort space at the moment TODO
	// Decide what kind of command interface this actuator/joint has
	/*	hardware_interface::JointHandle joint_handle_effort;
	joint_handle_effort = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
		&joint_effort_command_[j]);
	effort_interface_.registerHandle(joint_handle_effort); */

	// position handle
	hardware_interface::JointHandle joint_handle_position;
	joint_handle_position = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
		&joint_position_command_[j]);
	position_interface_.registerHandle(joint_handle_position);

	// velocity command handle
	hardware_interface::JointHandle joint_handle_velocity;
	joint_handle_velocity = hardware_interface::JointHandle(state_interface_.getHandle(joint_names_[j]),
		&joint_velocity_command_[j]);
	velocity_interface_.registerHandle(joint_handle_velocity);

	registerJointLimits(joint_names_[j],
		//joint_handle_effort,
		joint_handle_position,
		joint_handle_velocity,
		urdf_model,
		&joint_lower_limits_[j], &joint_upper_limits_[j]);
    }

    // Register interfaces
    registerInterface(&state_interface_);
    //registerInterface(&effort_interface_);
    registerInterface(&position_interface_);
    registerInterface(&velocity_interface_);
}

// Register the limits of the joint specified by joint_name and\ joint_handle. The limits are
// retrieved from the urdf_model.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void YumiHW::registerJointLimits(const std::string& joint_name,
	//const hardware_interface::JointHandle& joint_handle_effort,
	const hardware_interface::JointHandle& joint_handle_position,
	const hardware_interface::JointHandle& joint_handle_velocity,
	const urdf::Model *const urdf_model,
	double *const lower_limit, double *const upper_limit)
{
    *lower_limit = -std::numeric_limits<double>::max();
    *upper_limit = std::numeric_limits<double>::max();

    joint_limits_interface::JointLimits limits;
    bool has_limits = false;
    joint_limits_interface::SoftJointLimits soft_limits;
    bool has_soft_limits = false;

    if (urdf_model != NULL)
    {
	const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
	const boost::shared_ptr<const urdf::Joint> urdf_joint_sitffness = urdf_model->getJoint(joint_name + std::string("_stiffness"));
	if (urdf_joint != NULL)
	{
	    // Get limits from the URDF file.
	    if (joint_limits_interface::getJointLimits(urdf_joint, limits))
		has_limits = true;
	    if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
		has_soft_limits = true;
	}
    }

    if (!has_limits)
    {
	return;
    }

    if (limits.has_position_limits)
    {
	*lower_limit = limits.min_position;
	*upper_limit = limits.max_position;
    }


    if (has_soft_limits)
    {
	const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle_position(joint_handle_position, limits, soft_limits);
	pj_limits_interface_.registerHandle(limits_handle_position);
	const joint_limits_interface::VelocityJointSoftLimitsHandle limits_handle_velocity(joint_handle_velocity, limits, soft_limits);
	vj_limits_interface_.registerHandle(limits_handle_velocity);

    }
    else
    {
	const joint_limits_interface::PositionJointSaturationHandle sat_handle_position(joint_handle_position, limits);
	pj_sat_interface_.registerHandle(sat_handle_position);
	const joint_limits_interface::VelocityJointSaturationHandle sat_handle_velocity(joint_handle_velocity, limits);
	vj_sat_interface_.registerHandle(sat_handle_velocity);
    }

}

// Get Transmissions from the URDF
bool YumiHW::parseTransmissionsFromURDF(const std::string& urdf_string)
{
    std::vector<transmission_interface::TransmissionInfo> transmissions;

    // Only *standard* transmission_interface are parsed
    transmission_interface::TransmissionParser::parse(urdf_string, transmissions);

    // Now iterate and save only transmission from this robot
    for (int j = 0; j < n_joints_; ++j)
    {
	// std::cout << "Check joint " << joint_names_[j] << std::endl;
	std::vector<transmission_interface::TransmissionInfo>::iterator it = transmissions.begin();
	for(; it != transmissions.end(); ++it)
	{
	    // std::cout << "With transmission " << it->name_ << std::endl;
	    if (joint_names_[j].compare(it->joints_[0].name_) == 0)
	    {
		transmissions_.push_back( *it );
		// std::cout << "Found a match for transmission " << it->name_ << std::endl;
	    }
	}
    }

    if( transmissions_.empty() )
	return false;

    return true;
}

#if 0
// Init KDL stuff
bool YumiHW::initKDLdescription(const urdf::Model *const urdf_model)
{
    // KDL code to compute f_dyn(q)
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*urdf_model, kdl_tree))
    {
	ROS_ERROR("Failed to construct kdl tree");
	return false;
    }

    std::cout << "LWR kinematic successfully parsed with "
	<< kdl_tree.getNrOfJoints()
	<< " joints, and "
	<< kdl_tree.getNrOfJoints()
	<< " segments." << std::endl;

    // Get the info from parameters
    std::string root_name;
    ros::param::get(std::string("/") + robot_namespace_ + std::string("/root"), root_name);
    if( root_name.empty() )
	root_name = kdl_tree.getRootSegment()->first; // default

    std::string tip_name;
    ros::param::get(std::string("/") + robot_namespace_ + std::string("/tip"), tip_name);
    if( tip_name.empty() )
	tip_name = robot_namespace_ + std::string("_7_link"); ; // default

    std::cout << "Using root: " << root_name << " and tip: " << tip_name << std::endl;

    // this depends on how the world frame is set, in all our setups, world has always positive z pointing up.
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Extract the chain from the tree
    if(!kdl_tree.getChain(root_name, tip_name, lwr_chain_))
    {
	ROS_ERROR("Failed to get KDL chain from tree: ");
	return false;
    }

    ROS_INFO("Number of segments: %d", lwr_chain_.getNrOfSegments());
    ROS_INFO("Number of joints in chain: %d", lwr_chain_.getNrOfJoints());

    f_dyn_solver_.reset(new KDL::ChainDynParam(lwr_chain_,gravity_));

    joint_position_kdl_ = KDL::JntArray(lwr_chain_.getNrOfJoints());
    gravity_effort_ = KDL::JntArray(lwr_chain_.getNrOfJoints());

    return true;
}
#endif


bool YumiHW::canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const
{
    std::vector<ControlStrategy> desired_strategies;

    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
    {
	if( it->type.compare( std::string("hardware_interface::VelocityJointInterface") ) == 0 )
	{
	    desired_strategies.push_back( JOINT_VELOCITY );
	    ROS_WARN("Uncharted teritories here: switching to VelocityInterface\n");
	}
	else if( it->type.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
	{
	    desired_strategies.push_back( JOINT_POSITION );
	    ROS_INFO("Switching to Positon Control mode");
	}
	else if( it->type.compare( std::string("hardware_interface::EffortJointInterface") ) == 0 )
	{
	    ROS_WARN("Effort not implemented!");
	}
	else
	{
	    ROS_INFO("Controller of type %s?", it->type.c_str());
	    // Debug
	    // std::cout << "This controller does not use any command interface, so it is only sensing, no problem" << std::endl;
	}
    }

    if( desired_strategies.size() > 1 )
    {
	ROS_ERROR("Only a single controller can be active at a time. Choose one control strategy only");
	return false;
    }

    return true;
}

void YumiHW::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list)
{
    ControlStrategy desired_strategy = JOINT_POSITION; // default

    bool wantsPosition = false;
    bool wantsVelocity = false;

    for ( std::list<hardware_interface::ControllerInfo>::const_iterator it = start_list.begin(); it != start_list.end(); ++it )
    {
#if ROS_VERSION_MINIMUM(1,12,0)
	    //jade and karmic
	    for(int i=0; i<it->claimed_resources.size(); i++) {

		if( it->claimed_resources[i].hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
		{
		    ROS_INFO("Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)");
		    wantsPosition = true;
		}
		else if( it->claimed_resources[i].hardware_interface.compare( std::string("hardware_interface::VelocityJointInterface") ) == 0 )
		{
		    ROS_INFO("Request to switch to hardware_interface::VelocityJointInterface (JOINT_VELOCITY)");
		    wantsVelocity = true;
		} 
		else
		{
		    ROS_INFO("Controller of type %s, requested interface of type %s. Impossible, sorry.\n", 
			    it->type.c_str(), it->claimed_resources[i].hardware_interface.c_str());
		}
	    }
#else

	    //indigo and below
	    if( it->hardware_interface.compare( std::string("hardware_interface::PositionJointInterface") ) == 0 )
	    {
		ROS_INFO("Request to switch to hardware_interface::PositionJointInterface (JOINT_POSITION)");
		desired_strategy = JOINT_POSITION;
		break;
	    }
	    else if( it->hardware_interface.compare( std::string("hardware_interface::VelocityJointInterface") ) == 0 )
	    {
		ROS_INFO("Request to switch to hardware_interface::VelocityJointInterface (JOINT_VELOCITY)");
		desired_strategy = JOINT_VELOCITY;
		break;
	    }
#endif
    }
    if(wantsPosition) {		
	desired_strategy = JOINT_POSITION;
    }
    if(wantsVelocity) {
	desired_strategy = JOINT_VELOCITY;
    }

    if(wantsPosition && wantsVelocity) {
	ROS_ERROR("Cannot have both position and velocity interface. Will assume Velocity. Beware!");
    }

    for (int j = 0; j < n_joints_; ++j)
    {
	///semantic Zero
	joint_position_command_[j] = joint_position_[j];
	joint_velocity_command_[j] = 0.0;
	//joint_effort_command_[j] = 0.0;

	///call setCommand once so that the JointLimitsInterface receive the correct value on their getCommand()!
	try{  position_interface_.getHandle(joint_names_[j]).setCommand(joint_position_command_[j]);  }
	catch(const hardware_interface::HardwareInterfaceException&){}
	//try{  effort_interface_.getHandle(joint_names_[j]).setCommand(joint_effort_command_[j]);  }
	//catch(const hardware_interface::HardwareInterfaceException&){}
	try{  velocity_interface_.getHandle(joint_names_[j]).setCommand(joint_velocity_command_[j]);  }
	catch(const hardware_interface::HardwareInterfaceException&){}

	///reset joint_limit_interfaces
	pj_sat_interface_.reset();
	pj_limits_interface_.reset();
    }

    if(desired_strategy == getControlStrategy())
    {
	std::cout << "The ControlStrategy didn't change, it is already: " << getControlStrategy() << std::endl;
    }
    else
    {
	setControlStrategy(desired_strategy);
	std::cout << "The ControlStrategy changed to: " << getControlStrategy() << std::endl;
    }
}

void YumiHW::enforceLimits(ros::Duration period)
{
    vj_sat_interface_.enforceLimits(period);
    vj_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
}

