/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Francisco Vina
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <yumi_hw/yumi_hw_egm.h>
#include <abb_egm_interface/egm_common.h>


void YumiEGMInterface::YumiEGMInterface() :
    has_params_(false), rws_connection_ready_(false)
{
    getParams();
}


void YumiEGMInterface::~YumiEGMInterface()
{

}

void YumiEGMInterface::getParams()
{

    ros::NodeHandle nh("~");
    if(nh.hasParam("rws/ip"))
    {
        nh.param("rws/ip", rws_ip_, std::string("192.168.1.25"));
    }

    else
    {
        ROS_ERROR(ros::this_node::getName() + "/rws/ip param not found.");
        return;
    }

    nh.param("rws/port", rws_port_, std::string("80"));
    nh.param("rws/delay_time", rws_delay_time_, 0.01);
    nh.param("rws/max_signal_retries", rws_max_signal_retries_, 5);

    nh.param("egm/default_condition_time", egm_default_condition_time_, 10.0);

    nh.param("max_joint_velocity", max_joint_velocity_, 400.0*M_PI/180.0);



    has_params_ = true;
}


bool YumiEGMInterface::init()
{
    if (!has_params_){

        ROS_ERROR(ros::this_node::getName() + ": missing EGM/RWS parameters.");
        return false;
    }

    if (!initEGM()) return false;

    if(!initRWS()) return false;

}

bool YumiEGMInterface::stop()
{
    if(!stopEGM()) return false;

    io_service.stop();
    io_service_threads_.join_all();
}


bool YumiEGMInterface::initRWS()
{
    ROS_INFO(ros::this_node::getName() + ": starting RWS connection with IP & PORT: " + rws_ip_ + " / " + rws_port_);

    rws_interface_yumi_.reset(new RWSInterfaceYuMi(rws_ip_, rws_port_));
    ros::Duration(rws_delay_time_).sleep();

    // Check that RAPID is running on the robot and that robot is in AUTO mode
    if(!rws_interface_yumi_->isRAPIDRunning())
    {
        ROS_ERROR(ros::this_node::getName() + ": robot unavailable, make sure that the RAPID program is running on the flexpendant.");
        return false;
    }

    ros::Duration(rws_delay_time_).sleep();

    if(!rws_interface_yumi_->isModeAuto())
    {
        ROS_ERROR(ros::this_node::getName() + ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
        return false;
    }

    ros::Duration(rws_delay_time_).sleep();

    DualEGMData egm_data;

    if(!rws_interface_yumi_->getData(&egm_data))
    {
        ROS_ERROR(ros::this_node::getName() + ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
        return false;
    }

    egm_data.left.setCondTime(egm_default_condition_time_);
    egm_data.left.setMaxSpeedDeviation(max_joint_velocity_*egm_common_values::conversions::RAD_TO_DEG);
    egm_data.right.setCondTime(egm_default_condition_time_);
    egm_data.right.setMaxSpeedDeviation(max_joint_velocity_*egm_common_values::conversions::RAD_TO_DEG);
    rws_interface_yumi_->setData(egm_data);

    rws_connection_ready_ = true;

    if(!startEGM()) return false;

    return true;
}

bool YumiEGMInterface::initEGM()
{
    left_arm_egm_interface_.reset(new EGMInterfaceDefault(io_service_, egm_common_values::communication::DEFAULT_PORT_NUMBER));
    right_arm_egm_interface_.reset(new EGMInterfaceDefault(io_service_, egm_common_values::communication::DEFAULT_PORT_NUMBER + 1));
    configureEGM();

    // create threads for EGM communication
    for(size_t i = 0; i < MAX_NUMBER_OF_EGM_CONNECTIONS; i++)
    {
        io_service_threads_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    }

    return true;
}

void YumiEGMInterface::configureEGM()
{
    EGMInterfaceConfiguration configuration;
    configuration.basic.use_conditions = false;
    configuration.basic.axes = EGMInterfaceConfiguration::Seven;
    configuration.basic.execution_mode = EGMInterfaceConfiguration::ExecutionModes::Direct;
    configuration.communication.use_speed = true;
    configuration.simple_interpolation.use_speed = true;
    configuration.simple_interpolation.use_acceleration = true;
    configuration.simple_interpolation.use_interpolation = true;
    left_arm_egm_interface_->setConfiguration(configuration);
    right_arm_egm_interface_->setConfiguration(configuration);
}

bool YumiEGMInterface::startEGM()
{
    bool egm_started = false;

    if(rws_interface_yumi_ && rws_connection_ready_)
    {
      for(int i = 0; i < rws_max_signal_retries_ && !done; ++i)
      {
        egm_started = rws_interface_yumi_->doEGMStartJoint();
        if(!egm_started)
        {
          ROS_ERROR_STREAM(ros::this_node::getName() << ": failed to send EGM start signal! [Attempt " << i+1 << "/" <<
                           rws_max_signal_retries_<< "]");
        }
      }
    }

    return egm_started;
}

bool YumiEGMInterface::stopEGM()
{
    bool egm_stopped = false;

    if(rws_interface_yumi_ && rws_connection_ready_)
    {
      for(int i = 0; i < rws_max_signal_retries_ && !done; ++i)
      {
        egm_stopped = rws_interface_yumi_->doEGMStartJoint();
        if(!egm_stopped)
        {
          ROS_ERROR_STREAM(ros::this_node::getName() << ": failed to send EGM stop signal! [Attempt " << i+1 << "/" <<
                           rws_max_signal_retries_<< "]");
        }
      }
    }

    return egm_stopped;
}
