/*
 * Copyright (c) 2014, ISR, University of Coimbra
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *
 * Author: Goncalo Cabrita
 * Gazebo plugin for the FSR Husky Mine Detection Arm with th goal of solving
 * the <mimic> joint tag limitations in Gazebo
 */

#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

namespace gazebo
{
  class MimicPlugin : public ModelPlugin
  {
    public:
      MimicPlugin();
      virtual ~MimicPlugin();

      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();

    private:

      /// Parameters
      std::string node_namespace_;
      std::string joint_name_;
      std::string mimic_joint_name_;

      double multiplier_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;

      physics::JointPtr joint_;
      physics::JointPtr mimic_joint_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      bool kill_sim;
  };
}
#endif
