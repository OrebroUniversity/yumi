#include <demo_grasping/demo_grasping.h>

namespace demo_grasping
{
  //----------------------------------------------------------------------------------
  bool DemoGrasping::gimmeBeer(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    std_srvs::Empty srv;
    deactivateHQPControl();
    resetState();
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();

    if(!loadPersistentTasks())
      {
	ROS_ERROR("Could not load persistent tasks!");
	safeShutdown();
	return false;
      }

    if(!with_gazebo_)
      {
	//VELVET INITIAL POSE
	velvet_interface_node::VelvetToPos poscall;
	poscall.request.angle = 0.3;

	if(!velvet_pos_clt_.call(poscall))
	  {
	    ROS_ERROR("could not call velvet to pos");
	    ROS_BREAK();
	  }
      }

    bool grasp_success = false;
    while(!grasp_success)
      {
	{//MANIPULATOR SENSING CONFIGURATION
	  ROS_INFO("Trying to put the manipulator in sensing configuration.");
	  boost::mutex::scoped_lock lock(manipulator_tasks_m_);
	  task_status_changed_ = false;
	  task_success_ = false;
	  deactivateHQPControl();
	  if(!resetState())
	    {
	      ROS_ERROR("Could not reset the state!");
	      safeShutdown();
	      return false;
	    }
	  if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
	    {
	      safeShutdown();
	      return false;
	    }

	  if(!setJointConfiguration(sensing_config_))
	    {
	      ROS_ERROR("Could not set manipulator sensing state!");
	      safeShutdown();
	      return false;
	    }
	  task_error_tol_ = 1e-2;
	  activateHQPControl();

	  while(!task_status_changed_)
	    cond_.wait(lock);

	  if(!task_success_)
	    {
	      ROS_ERROR("Could not complete the manipulator sensing state tasks!");
	      safeShutdown();
	      return false;
	    }
	  ROS_INFO("Manipulator sensing state tasks executed successfully.");
	}

	{//GRASP APPROACH
	  ROS_INFO("Trying grasp approach.");
	  boost::mutex::scoped_lock lock(manipulator_tasks_m_);
	  task_status_changed_ = false;
	  task_success_ = false;
	  deactivateHQPControl();
	  if(!resetState())
	    {
	      ROS_ERROR("Could not reset the state!");
	      safeShutdown();
	      return false;
	    }

	  if(!with_gazebo_)
	    if(!getGraspInterval())
	      ROS_WARN("Could not obtain the grasp intervall - using default interval!");
#if 0
#endif

	  if(!setCartesianStiffness(1000, 1000, 100, 100, 100, 100))
	    {
	      safeShutdown();
	      return false;
	    }

	  if(!setGraspApproach())
	    {
	      ROS_ERROR("Could not set the grasp approach!");
	      safeShutdown();
	      return false;
	    }
	  task_error_tol_ =  1e-3;
	  activateHQPControl();

	  while(!task_status_changed_)
	    cond_.wait(lock);

	  if(!task_success_)
	    {
	      ROS_ERROR("Could not complete the grasp approach tasks!");
	      safeShutdown();
	      return false;
	    }

	  ROS_INFO("Grasp approach tasks executed successfully.");
	}


	if(!with_gazebo_)
	  {
	    //SET GRASP STIFFNESS
	    if(!setCartesianStiffness(1000, 50, 30, 100, 100, 10))
	      {
		safeShutdown();
		return false;
	      }

	    deactivateHQPControl();
	    //VELVET GRASP_
	    velvet_interface_node::SmartGrasp graspcall;
	    graspcall.request.current_threshold_contact = 20;
	    graspcall.request.current_threshold_final = 35;
	    graspcall.request.max_belt_travel_mm = 90;
	    graspcall.request.phalange_delta_rad = 0.02;
	    graspcall.request.gripper_closed_thresh = 1.5;
	    graspcall.request.check_phalanges = true;

	    if(!velvet_grasp_clt_.call(graspcall)) {
	      ROS_ERROR("could not call grasping");
	      ROS_BREAK();
	    }
	    if(!graspcall.response.success)
	      ROS_ERROR("Grasp failed!");
	    else
	      {
		grasp_success = true;
		ROS_INFO("Grasp aquired.");
	      }

	  }
	else
	  grasp_success = true;

      }

    {//OBJECT EXTRACT
      ROS_INFO("Trying object extract.");
      boost::mutex::scoped_lock lock(manipulator_tasks_m_);
      task_status_changed_ = false;
      task_success_ = false;
      deactivateHQPControl();
      if(!resetState())
	{
	  ROS_ERROR("Could not reset the state!");
	  safeShutdown();
	  return false;
	}
      if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
	{
	  safeShutdown();
	  return false;
	}
      if(!setObjectExtract())
	{
	  ROS_ERROR("Could not set the object extract!");
	  safeShutdown();
	  return false;
	}

      task_error_tol_ = 5 * 1e-3;
      activateHQPControl();

      while(!task_status_changed_)
	cond_.wait(lock);

      if(!task_success_)
	{
	  ROS_ERROR("Could not complete the object extract tasks!");
	  safeShutdown();
	  return false;
	}
      ROS_INFO("Object extract tasks executed successfully.");
    }

    {//MANIPULATOR GIMME BEER CONFIGURATION
      ROS_INFO("Trying to put the manipulator in gimme beer configuration.");

      boost::mutex::scoped_lock lock(manipulator_tasks_m_);
      task_status_changed_ = false;
      task_success_ = false;
      deactivateHQPControl();
      if(!resetState())
	{
	  ROS_ERROR("Could not reset the state!");
	  safeShutdown();
	  return false;
	}
      if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
	{
	  safeShutdown();
	  return false;
	}

      if(!setJointConfiguration(gimme_beer_config_))
	{
	  ROS_ERROR("Could not set manipulator sensing configuration!");
	  safeShutdown();
	  return false;
	}
      task_error_tol_ = 1e-2;
      activateHQPControl();

      while(!task_status_changed_)
	cond_.wait(lock);

      if(!task_success_)
	{
	  ROS_ERROR("Could not complete the manipulator gimme beer configuration tasks!");
	  safeShutdown();
	  return false;
	}
      ROS_INFO("Manipulator gimme beer configuration tasks executed successfully.");
    }

    if(!with_gazebo_)
      {
	velvet_interface_node::VelvetToPos poscall2;
	poscall2.request.angle = 0.2;

	if(!velvet_pos_clt_.call(poscall2))
	  {
	    ROS_ERROR("could not call velvet to pos");
	    ROS_BREAK();
	  }
      }


    deactivateHQPControl();
    resetState();
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();

#if 0
#endif
    ROS_INFO("GIMME BEER FINISHED.");

    return true;
  }
  //---------------------------------------------------------------------------------------------------
} //end namespace
