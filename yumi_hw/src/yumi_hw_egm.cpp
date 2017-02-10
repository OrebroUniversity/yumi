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
    left_arm_egm_interface_(io_service_, egm_common_values::communication::DEFAULT_PORT_NUMBER),
    right_arm_egm_interface_(io_service_, egm_common_values::communication::DEFAULT_PORT_NUMBER + 1)
{
    configureEGM();
}


void YumiEGMInterface::~YumiEGMInterface()
{

}


bool YumiEGMInterface::init(const std::string &ip, const std::string &port)
{

    rws_interface_yumi_.reset(new RWSInterfaceYuMi(ip, port));

    for(size_t i = 0; i < MAX_NUMBER_OF_EGM_CONNECTIONS; i++)
    {
        io_service_threads_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    }
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
    left_arm_egm_interface_.setConfiguration(configuration);
    right_arm_egm_interface_.setConfiguration(configuration);
}
