#include <iostream>
#include <map>
#include <string>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/sensors/Sensor.hh>
// #include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/math/Vector3.hh>

#include <tf/tf.h>

#include <gazebo_plugins/gazebo_ros_utils.h>

// #include <compliance_controllers_msgs/YumiContactState.h>
#include <contact_force_msgs/YumiContactState.h>

#include "yumi_contact_sensor_plugin.h"

namespace yumi {
GZ_REGISTER_SENSOR_PLUGIN(YumiContactSensorPlugin)

YumiContactSensorPlugin::YumiContactSensorPlugin() : SensorPlugin() {}

YumiContactSensorPlugin::~YumiContactSensorPlugin() {
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

void YumiContactSensorPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parentSensor = dynamic_pointer_cast<gazebo::sensors::ContactSensor>(_parent);
  if (!this->parentSensor) {
    ROS_ERROR("Contact sensor parent is not of type ContactSensor");
    return;
  }

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->bumper_topic_name_ = "bumper_states";
  if (_sdf->HasElement("bumperTopicName"))
    this->bumper_topic_name_ =
      _sdf->GetElement("bumperTopicName")->Get<std::string>();

  if (!_sdf->HasElement("frameName")) {
    ROS_INFO("bumper plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  } else {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  this->yumi_contact_pub_ = this->rosnode_->advertise<contact_force_msgs::YumiContactState>(
    std::string(this->bumper_topic_name_), 1);

  this->callback_queue_thread_ = boost::thread(
    boost::bind(&YumiContactSensorPlugin::ContactQueueThread, this));

  this->update_connection_ = this->parentSensor->ConnectUpdated(
    boost::bind(&YumiContactSensorPlugin::OnContact, this));

  this->parentSensor->SetActive(true);

  std::cout << "sensor name = " << this->parentSensor->Name() << "\n";
  std::cout << "sensor parent name = " << this->parentSensor->ParentName() << "\n";
  std::cout << "sensor scoped name = " << this->parentSensor->ScopedName() << "\n";
  std::cout << "sensor topic = " << this->parentSensor->Topic() << "\n";
  std::cout << "sensor type = " << this->parentSensor->Type() << "\n";
  std::cout << "this->robot_namespace_ = " << this->robot_namespace_ << "\n";
  std::cout << "this->bumper_topic_name_ = " << this->bumper_topic_name_ << "\n";
  std::cout << "this->frame_name_ = " << this->frame_name_ << "\n";
}

void YumiContactSensorPlugin::OnContact() {
  if (this->yumi_contact_pub_.getNumSubscribers() <= 0)
    return;
  
  contact_force_msgs::YumiContactState msg;

  gazebo::msgs::Contacts contacts = this->parentSensor->Contacts();
  msg.header.frame_id = this->frame_name_;
  msg.header.stamp = ros::Time(contacts.time().sec(),
                               contacts.time().nsec());

  msg.wrench.force.x = 0;
  msg.wrench.force.y = 0;
  msg.wrench.force.z = 0;
  msg.wrench.torque.x = 0;
  msg.wrench.torque.y = 0;
  msg.wrench.torque.z = 0;

  gazebo::math::Quaternion frame_rot = gazebo::math::Quaternion(1, 0, 0, 0);

  unsigned int contactsPacketSize = contacts.contact_size();
  for (unsigned int i = 0; i < contactsPacketSize; ++i) {
    gazebo::msgs::Contact contact = contacts.contact(i);

    unsigned int contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j) {
      gazebo::math::Vector3 force = frame_rot.RotateVectorReverse(gazebo::math::Vector3(
        contact.wrench(j).body_1_wrench().force().x(),
        contact.wrench(j).body_1_wrench().force().y(),
        contact.wrench(j).body_1_wrench().force().z()));
      gazebo::math::Vector3 torque = frame_rot.RotateVectorReverse(gazebo::math::Vector3(
        contact.wrench(j).body_1_wrench().torque().x(),
        contact.wrench(j).body_1_wrench().torque().y(),
        contact.wrench(j).body_1_wrench().torque().z()));

      msg.wrench.force.x += force.x;
      msg.wrench.force.y += force.y;
      msg.wrench.force.z += force.z;
      msg.wrench.torque.x += torque.x;
      msg.wrench.torque.y += torque.y;
      msg.wrench.torque.z += torque.z;
    }
  }

  yumi_contact_pub_.publish(msg);
  return;
}
/*
void oldshit() {}
  gazebo::msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  this->contact_state_msg_.header.frame_id = this->frame_name_;
  this->contact_state_msg_.header.stamp = ros::Time(contacts.time().sec(),
                               contacts.time().nsec());

  gazebo::math::Pose pose, frame_pose;
  gazebo::math::Quaternion rot, frame_rot;
  gazebo::math::Vector3 pos, frame_pos;

  frame_pos = gazebo::math::Vector3(0, 0, 0);
  frame_rot = gazebo::math::Quaternion(1, 0, 0, 0);  // gazebo u,x,y,z == identity
  frame_pose = gazebo::math::Pose(frame_pos, frame_rot);

  this->contact_state_msg_.states.clear();

  unsigned int contactsPacketSize = contacts.contact_size();
  for (unsigned int i = 0; i < contactsPacketSize; ++i) {
    gazebo_msgs::ContactState state;
    gazebo::msgs::Contact contact = contacts.contact(i);

    state.collision1_name = contact.collision1();
    state.collision2_name = contact.collision2();
    std::ostringstream stream;
    stream << "Debug:  i:(" << i << "/" << contactsPacketSize
      << ")     my geom:" << state.collision1_name
      << "   other geom:" << state.collision2_name
      << "         time:" << ros::Time(contact.time().sec(), contact.time().nsec())
      << std::endl;
    state.info = stream.str();

    state.wrenches.clear();
    state.contact_positions.clear();
    state.contact_normals.clear();
    state.depths.clear();

    geometry_msgs::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    unsigned int contactGroupSize = contact.position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j) {
      gazebo::math::Vector3 force = frame_rot.RotateVectorReverse(gazebo::math::Vector3(
        contact.wrench(j).body_1_wrench().force().x(),
        contact.wrench(j).body_1_wrench().force().y(),
        contact.wrench(j).body_1_wrench().force().z()));
      gazebo::math::Vector3 torque = frame_rot.RotateVectorReverse(gazebo::math::Vector3(
        contact.wrench(j).body_1_wrench().torque().x(),
        contact.wrench(j).body_1_wrench().torque().y(),
        contact.wrench(j).body_1_wrench().torque().z()));

      geometry_msgs::Wrench wrench;
      wrench.force.x  = force.x;
      wrench.force.y  = force.y;
      wrench.force.z  = force.z;
      wrench.torque.x = torque.x;
      wrench.torque.y = torque.y;
      wrench.torque.z = torque.z;
      state.wrenches.push_back(wrench);

      total_wrench.force.x  += wrench.force.x;
      total_wrench.force.y  += wrench.force.y;
      total_wrench.force.z  += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      gazebo::math::Vector3 position = frame_rot.RotateVectorReverse(
          gazebo::math::Vector3(contact.position(j).x(),
                                contact.position(j).y(),
                                contact.position(j).z()) - frame_pos);
      geometry_msgs::Vector3 contact_position;
      contact_position.x = position.x;
      contact_position.y = position.y;
      contact_position.z = position.z;
      state.contact_positions.push_back(contact_position);

      gazebo::math::Vector3 normal = frame_rot.RotateVectorReverse(
          gazebo::math::Vector3(contact.normal(j).x(),
                                contact.normal(j).y(),
                                contact.normal(j).z()));
      
      geometry_msgs::Vector3 contact_normal;
      contact_normal.x = normal.x;
      contact_normal.y = normal.y;
      contact_normal.z = normal.z;
      state.contact_normals.push_back(contact_normal);

      state.depths.push_back(contact.depth(j));
    }
    state.total_wrench = total_wrench;
    this->contact_state_msg_.states.push_back(state);
  }
  this->contact_pub_.publish(this->contact_state_msg_);
}
*/
void YumiContactSensorPlugin::ContactQueueThread() {
  static const double timeout = 0.01;
  while (this->rosnode_->ok()){
    this->contact_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

} // namespace irb4400