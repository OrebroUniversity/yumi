#include <demo_grasping/demo_grasping.h>

namespace demo_grasping
{
  //----------------------------------------------------------------------------------
  bool DemoGrasping::letsDance(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
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

    // if(!with_gazebo_)
    //   {
    // 	if(!setCartesianStiffness(800, 800, 800, 100, 100, 100))
    // 	  {
    // 	    safeShutdown();
    // 	    return false;
    // 	  }
    //   }

    // for (unsigned int i=0; i<3; i++)
    //   {

    // 	if(!with_gazebo_)
    // 	  {
	    // //VELVET POSE
	    // velvet_interface_node::VelvetToPos poscall;

	    // poscall.request.angle = 0.1;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }

	    // poscall.request.angle = 1.45;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }
    //	  }

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
	    // //VELVET POSE
	    // velvet_interface_node::VelvetToPos poscall;

	    // poscall.request.angle = 0.1;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }

	    // poscall.request.angle = 1.45;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }
	  }

	{//MANIPULATOR TRANSFER CONFIGURATION
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

	  if(!setJointConfiguration(transfer_config_))
	    {
	      ROS_ERROR("Could not set manipulator transfer configuration!");
	      safeShutdown();
	      return false;
	    }
	  task_error_tol_ = 1e-2;
	  activateHQPControl();

	  while(!task_status_changed_)
	    cond_.wait(lock);

	  if(!task_success_)
	    {
	      ROS_ERROR("Could not complete the manipulator transfer configuration tasks!");
	      safeShutdown();
	      return false;
	    }
	  ROS_INFO("Manipulator transfer configuration tasks executed successfully.");
	}


	if(!with_gazebo_)
	  {
	    // //VELVET POSE
	    // velvet_interface_node::VelvetToPos poscall;

	    // poscall.request.angle = 0.1;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }

	    // poscall.request.angle = 1.45;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }
	  }
#if 0
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

	  if(!setJointConfiguration(sensing_config_))
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
	      ROS_ERROR("Could not complete the manipulator sensing configuration tasks!");
	      safeShutdown();
	      return false;
	    }
	  ROS_INFO("Manipulator sensing configuration tasks executed successfully.");
	}

	if(!with_gazebo_)
	  {
	    // //VELVET POSE
	    // velvet_interface_node::VelvetToPos poscall;

	    // poscall.request.angle = 0.1;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }

	    // poscall.request.angle = 1.45;
	    // if(!velvet_pos_clt_.call(poscall))
	    //   {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    //   }
	  }
#endif

	{//MANIPULATOR LOOK BEER  CONFIGURATION
	  ROS_INFO("Trying to put the manipulator in look beer configuration.");

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

	  if(!setJointConfiguration(look_beer_config_))
	    {
	      ROS_ERROR("Could not set manipulator look beer configuration!");
	      safeShutdown();
	      return false;
	    }
	  task_error_tol_ = 1e-2;
	  activateHQPControl();

	  while(!task_status_changed_)
	    cond_.wait(lock);

	  if(!task_success_)
	    {
	      ROS_ERROR("Could not complete the manipulator look beer configuration tasks!");
	      safeShutdown();
	      return false;
	    }
	  ROS_INFO("Manipulator look beer configuration tasks executed successfully.");
	}

 

    deactivateHQPControl();
    resetState();
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();

#if 0
#endif
    ROS_INFO("LETS DANCE FINISHED.");

    return true;
  }
  //---------------------------------------------------------------------------------------------------
} //end namespace
