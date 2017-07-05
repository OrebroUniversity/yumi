/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2017, Yoshua Nava, yoshua.nava.chocron@gmail.com
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


#include <yumi_cameras/yumi_cameras_rws.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <abb_rws_interface/rws_interface_yumi.h>


using namespace abb::rws_interface;


// RWS interface which uses TCP communication for starting the EGM joint mode on YuMi
boost::shared_ptr<RWSInterfaceYuMi> rws_interface_;
// RWS connection parameters
std::string rws_ip_, rws_port_;
double rws_delay_time_;
int rws_max_signal_retries_;
bool rws_connection_ready_;




void setCameraParams(CameraData* camera_data)
{
    camera_data->setExposureTime(1.0);
}



bool sendCameraParams()
{
    DualCameraData dual_camera_data;

    if(!rws_interface_->getData(&dual_camera_data))
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
        return false;
    }

    setCameraParams(&dual_camera_data.left);
    setCameraParams(&dual_camera_data.right);

    // rws_interface_->setData(dual_camera_data);

    // rws_interface_->getData(&dual_camera_data);

    rws_interface_->doCameraSetExposure(dual_camera_data, BOTH_SIDES);

    return true;
}


bool initRWS()
{

    rws_interface_.reset(new RWSInterfaceYuMi(rws_ip_, rws_port_));
    ros::Duration(rws_delay_time_).sleep();

    // Check that RAPID is running on the robot and that robot is in AUTO mode
    if(!rws_interface_->isRAPIDRunning())
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure that the RAPID program is running on the flexpendant.");
        return false;
    }

    ros::Duration(rws_delay_time_).sleep();

    if(!rws_interface_->isModeAuto())
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure to set the robot to AUTO mode on the flexpendant.");
        return false;
    }

    ros::Duration(rws_delay_time_).sleep();


    if(!sendCameraParams()) 
    {
        ROS_ERROR_STREAM(ros::this_node::getName() << ": robot unavailable, make sure that the camera firmware is correctly set-up.");
        return false;
    }


    rws_connection_ready_ = true;
    ros::Duration(rws_delay_time_).sleep();

    return rws_connection_ready_;
}



int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "yumi_cameras_node");
    ros::NodeHandle nh("~");

    nh.param("rws_ip", rws_ip_, std::string("192.168.125.1"));
    nh.param("rws_port", rws_port_, std::string("80"));
    nh.param("rws_delay_time", rws_delay_time_, 1.0);
    nh.param("rws_max_signal_retries", rws_max_signal_retries_, 5);

    initRWS();

    std::cout << "hola" << std::endl;
    rws_interface_->doCameraRequestImage(BOTH_SIDES);
    ros::Duration(rws_delay_time_).sleep();
    std::cout << "chao" << std::endl;

    // ros::AsyncSpinner spinner(6); // Use 4 threads
    // spinner.start();
    // ros::waitForShutdown();


    return EXIT_SUCCESS;
}