/* System dependencies */
#include <sys/mman.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <stdexcept>

/* ROS headers */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <boost/interprocess/smart_ptr/unique_ptr.hpp>

/* The Yumi EGM and RWS interfaces */
#include "yumi_hw/yumi_hw_rapid.h"
#include <yumi_hw/yumi_hw_egm.h>

bool g_quit = false;

void quitRequested(int sig)
{
  g_quit = true;
}

/* Get the URDF XML from the parameter server */
std::string getURDF(ros::NodeHandle &model_nh_, std::string param_name)
{
  std::string urdf_string;
  std::string robot_description = "/robot_description";

  /* Search and wait for robot_description on param server */
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (model_nh_.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("yumi_hw_ifce_node", "Yumi Interface node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      model_nh_.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("yumi_hw_ifce_node", "Yumi Interface Node is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description.c_str());

      model_nh_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("yumi_hw_ifce_node", "Received URDF from param server, parsing...");

  return urdf_string;
}

int main( int argc, char** argv )
{
  /* Init ROS node */
  ros::init(argc, argv, "yumi_hw_interface", ros::init_options::NoSigintHandler);

  /* ROS spinner */
  ros::AsyncSpinner spinner(4);
  spinner.start();

  /* Custom operating system signal handlers */
  signal(SIGTERM, quitRequested);
  signal(SIGINT, quitRequested);
  signal(SIGHUP, quitRequested);

  /* Create node handler */
  ros::NodeHandle yumi_nh ("~");

  /* Get params or give default values */
  int port;
  std::string ip;
  std::string name;
  bool use_egm;
  yumi_nh.param("port", port, 80);
  yumi_nh.param("ip", ip, std::string("192.168.125.1") );
  yumi_nh.param("name", name, std::string("yumi"));
  yumi_nh.param("use_egm", use_egm, false);

  /* Get the general robot description, the YumiHW class will take care of parsing what's useful to itself */
  std::string urdf_string = getURDF(yumi_nh, "/robot_description");

  YumiHW* yumi_robot;

  if(!use_egm)
  {
      yumi_robot = new YumiHWRapid();
      YumiHWRapid* yumi_robot_rapid = dynamic_cast<YumiHWRapid*>(yumi_robot);
      yumi_robot_rapid->setup(ip);

      float sampling_time = yumi_robot_rapid->getSampleTime();
      ROS_INFO("No EGM. Sampling time on robot: %f", sampling_time);
  }
  else
  {
      yumi_robot = new YumiHWEGM();
      YumiHWEGM* yumi_robot_egm = dynamic_cast<YumiHWEGM*>(yumi_robot);
      std::stringstream port_ss;
      port_ss << port;
      yumi_robot_egm->setup(ip, port_ss.str());
      ROS_INFO("Setting up EGM");
  }

  yumi_robot->create(name, urdf_string);

  if(!yumi_robot->init())
  {
    ROS_FATAL_NAMED("yumi_hw","Could not initialize robot real interface");
    return -1;
  }

  /* Timer variables */
  struct timespec ts = {0, 0};
  ros::Time last(ts.tv_sec, ts.tv_nsec), now(ts.tv_sec, ts.tv_nsec);
  ros::Duration period(1.0);

  /* The controller manager */
  controller_manager::ControllerManager manager(yumi_robot);

  /* Publisher of the control loop period */
  ros::Publisher control_period_pub;
  control_period_pub = yumi_nh.advertise<std_msgs::Float64>("/yumi/egm_control_period", 1000);
  

  /* Main control loop */
  while( !g_quit )
  {
    // get the time / period
    if (!clock_gettime(CLOCK_MONOTONIC, &ts))
    {
      now = ros::Time::now();	
      now.sec = ts.tv_sec;
      now.nsec = ts.tv_nsec;
      period = now - last;
      last = now;
    } 
    else
    {
      ROS_FATAL("Failed to poll realtime clock!");
      break;
    }

    /* Read the state from YuMi */
    yumi_robot->read(now, period);
    
    /* Update the controllers */
    manager.update(now, period);

    /* Write the command to YuMi */
    yumi_robot->write(now, period);

    // std::cout << "Control loop period is " << period.toSec() * 1000 << " ms" << std::endl;
    control_period_pub.publish(period.toSec());

  }

  delete yumi_robot;

  std::cerr<<"Stopping spinner..."<<std::endl;
  spinner.stop();

  std::cerr<<"Bye!"<<std::endl;

  return 0;
}
