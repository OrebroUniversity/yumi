#include <demo_grasping/demo_grasping.h>
#include <std_msgs/Bool.h>

namespace demo_grasping
{
  //----------------------------------------------------------------------------------
  bool DemoGrasping::gimmeBeer(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
//     ROS_INFO("gimme beer callback");
//     ros::Time now=ros::Time::now();
//     bag_name_ = bag_path_+"/"+boost::lexical_cast<std::string>(now.toSec())+".bag";

//     ROS_INFO("opening bag file");
//     bag_.open(bag_name_, rosbag::bagmode::Write);
//     controller_manager_msgs::SwitchController switch_msg;

// #if 0
//     if(!with_gazebo_)
//       {
// 	//SWITCH TO VELOCITY CONTROL
// 	switch_msg.request.start_controllers.push_back("hqp_vel_controller");
// 	switch_msg.request.stop_controllers.push_back("joint_trajectory_controller");
// 	switch_msg.request.strictness=2;   
// 	switch_msg.response.ok=false;

// 	ROS_INFO("Switching to hqp velocity control.");
// 	deactivateHQPControl();
// 	switch_controller_clt_.call(switch_msg);
// 	if(!switch_msg.response.ok)
// 	  {
// 	    ROS_ERROR("Could not switch to the hqp velocity controller!");
// 	    safeShutdown();
// 	    return false;
// 	  }
// 	sleep(2);
//       }
// #endif

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
	// //VELVET INITIAL POSE
	// velvet_interface_node::VelvetToPos poscall;
	// poscall.request.angle = 0.3;

	// if(!velvet_pos_clt_.call(poscall))
	//   {
	//     ROS_ERROR("could not call velvet to pos");
	//     ROS_BREAK();
	//   }
	// write_img_ = true;
      }

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

    if(!with_gazebo_)
      {
	//RESET THE MAP
	if(!reset_map_clt_.call(srv))
	  {
	    ROS_ERROR("could not reset gplanner map");
	    ROS_BREAK();
	  }
      }

    {//GRASP APPROACH
      //write_jnts_=true;
      //write_tf_=true;
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
	{
	  //write_cluster_ = true;
	  if(!getGraspInterval())
	    ROS_WARN("Could not obtain the grasp intervall - using default interval!");

	}
    
	if(!with_gazebo_)
	{
	    // //VELVET pre-grasp configuration 
	    // velvet_interface_node::VelvetToPos poscall;
	    // poscall.request.angle = grasp_.angle;

	    // if(!velvet_pos_clt_.call(poscall))
	    // {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    // }
	}

      if(!setGraspApproach())
	{
	  ROS_ERROR("Could not set the grasp approach!");
	  safeReset();
	  //	  bag_.close();
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
      //write_jnts_=false;
    }

    if(!with_gazebo_)
      {
	// //VELVET GRASP_
	// velvet_interface_node::SmartGrasp graspcall;
	// graspcall.request.current_threshold_contact = 30;
	// graspcall.request.current_threshold_final = 60;
	// graspcall.request.max_belt_travel_mm = -180;
	// graspcall.request.phalange_delta_rad = 0.02;
	// graspcall.request.gripper_closed_thresh = 1.5;
	// graspcall.request.check_phalanges = false;

	// if(!velvet_grasp_clt_.call(graspcall)) {
	//   ROS_ERROR("could not call grasping");
	//   ROS_BREAK();
	// }
	// if(!graspcall.response.success)
	//   ROS_ERROR("Grasp failed!");
	// else
	//   ROS_INFO("Grasp aquired.");
      }


    {//OBJECT EXTRACT
      //write_tf_=true;
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
      // if(!setCartesianStiffness(1000, 1000, 1000, 100, 100, 100))
      // 	{
      // 	  safeShutdown();
      // 	  return false;
      // 	}
      if(!setObjectExtract())
	{
	  ROS_ERROR("Could not set the object extract!");
	  safeShutdown();
	  return false;
	}

      task_error_tol_ = 0.2 * 1e-2;
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
      //write_tf_=false;
    }
    
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

    if(!with_gazebo_)
      {
	// velvet_interface_node::VelvetToPos poscall2;
	// poscall2.request.angle = 0.2;

	// if(!velvet_pos_clt_.call(poscall2))
	//   {
	//     ROS_ERROR("could not call velvet to pos");
	//     ROS_BREAK();
	//   }
      }
#if 0
#endif

    deactivateHQPControl();
    resetState();
    reset_hqp_control_clt_.call(srv);
    pers_task_vis_ids_.clear();

    ROS_INFO("GIMME BEER FINISHED.");

    // bag_.close();

    return true;
  }

 //----------------------------------------------------------------------------------
  bool DemoGrasping::approachBeer(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
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
	// //VELVET INITIAL POSE
	// velvet_interface_node::VelvetToPos poscall;
	// poscall.request.angle = 0.3;

	// if(!velvet_pos_clt_.call(poscall))
	//   {
	//     ROS_ERROR("could not call velvet to pos");
	//     ROS_BREAK();
	//   }
	// write_img_ = true;
      }

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

    if(!with_gazebo_)
      {
	//RESET THE MAP
	if(!reset_map_clt_.call(srv))
	  {
	    ROS_ERROR("could not reset gplanner map");
	    ROS_BREAK();
	  }
      }

    {//GRASP APPROACH
      //write_jnts_=true;
      //write_tf_=true;
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
	{
	  //write_cluster_ = true;
	  if(!getGraspInterval())
	    ROS_WARN("Could not obtain the grasp intervall - using default interval!");

	}
    
	if(!with_gazebo_)
	{
	    // //VELVET pre-grasp configuration 
	    // velvet_interface_node::VelvetToPos poscall;
	    // poscall.request.angle = grasp_.angle;

	    // if(!velvet_pos_clt_.call(poscall))
	    // {
	    // 	ROS_ERROR("could not call velvet to pos");
	    // 	ROS_BREAK();
	    // }
	}

      if(!setGraspApproach())
	{
	  ROS_ERROR("Could not set the grasp approach!");
	  safeReset();
	  //	  bag_.close();
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
      // write_jnts_=false;
    }

    deactivateHQPControl();
    ROS_INFO("APPROACH BEER FINISHED.");

    // bag_.close();

    return true;
  }
  //----------------------------------------------------------------------------------
  bool DemoGrasping::extractBeer(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    ROS_INFO("extract beer callback");

    // controller_manager_msgs::SwitchController switch_msg;

    // if(!with_gazebo_)
    //   {
    // 	//SWITCH TO VELOCITY CONTROL
    // 	switch_msg.request.start_controllers.push_back("cartesian_impedance_controller");
    // 	switch_msg.request.stop_controllers.push_back("joint_trajectory_controller");
    // 	switch_msg.request.strictness=2;   
    // 	switch_msg.response.ok=false;

    // 	ROS_INFO("Switching to cartesian impedance control.");
    // 	deactivateHQPControl();
    // 	switch_controller_clt_.call(switch_msg);
    // 	if(!switch_msg.response.ok)
    // 	  {
    // 	    ROS_ERROR("Could not switch to the cartesian impedance controller!");
    // 	    safeShutdown();
    // 	    return false;
    // 	  }

	// //VELVET GRASP_
	// velvet_interface_node::SmartGrasp graspcall;
	// graspcall.request.current_threshold_contact = 30;
	// graspcall.request.current_threshold_final = 65;
	// graspcall.request.max_belt_travel_mm = -180;
	// graspcall.request.phalange_delta_rad = 0.02;
	// graspcall.request.gripper_closed_thresh = 1.5;
	// graspcall.request.check_phalanges = false;

	// if(!velvet_grasp_clt_.call(graspcall)) {
	//   ROS_ERROR("could not call grasping");
	//   ROS_BREAK();
	// }
	// if(!graspcall.response.success)
	//   ROS_ERROR("Grasp failed!");
	// else
	//   ROS_INFO("Grasp aquired.");

    // }

    ROS_INFO("EXTRACT BEER FINISHED.");


    return true;
  }

  //---------------------------------------------------------------------------------------------------
    bool DemoGrasping::expOutcome(hqp_controllers_msgs::ActivateHQPControl::Request & req, hqp_controllers_msgs::ActivateHQPControl::Response &res)
{
    // bag_.open(bag_name_, rosbag::bagmode::Append);
    // std_msgs::Bool outcome;
    // outcome.data=req.active;

    // bag_.write("experiment_outcome",ros::Time::now(),outcome);
    // bag_.close();
    return true;
}
  //---------------------------------------------------------------------------------------------------
} //end namespace
