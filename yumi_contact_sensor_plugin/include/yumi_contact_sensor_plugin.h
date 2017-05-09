/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef YUMI_CONTACT_SENSOR_PLUGIN_H
#define YUMI_CONTACT_SENSOR_PLUGIN_H

#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <sys/time.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>

#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
// #include <sdf/sdf.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/common/Plugin.hh>

namespace yumi {
  class YumiContactSensorPlugin : public gazebo::SensorPlugin {
  public:
     YumiContactSensorPlugin();
     ~YumiContactSensorPlugin();

     void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  private:
    void OnContact();

    ros::NodeHandle* rosnode_;
    ros::Publisher contact_pub_;
    ros::Publisher yumi_contact_pub_;

    gazebo::sensors::ContactSensorPtr parentSensor;

    std::string bumper_topic_name_;

    std::string frame_name_;

    gazebo_msgs::ContactsState contact_state_msg_;

    std::string robot_namespace_;

    ros::CallbackQueue contact_queue_;
    void ContactQueueThread();
    boost::thread callback_queue_thread_;

    gazebo::event::ConnectionPtr update_connection_;
  };
}

#endif

