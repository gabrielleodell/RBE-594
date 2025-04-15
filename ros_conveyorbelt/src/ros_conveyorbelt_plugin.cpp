/*

# ===================================== COPYRIGHT
===================================== # # # #  IFRA (Intelligent Flexible
Robotics and Assembly) Group, CRANFIELD UNIVERSITY        # #  Created on behalf
of the IFRA Group at Cranfield University, United Kingdom          # #  E-mail:
IFRA@cranfield.ac.uk                                                         #
# # #  Licensed under the Apache-2.0 License. # #  You may not use this file
except in compliance with the License.                     # #  You may obtain a
copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  # # # #
Unless required by applicable law or agreed to in writing, software distributed
# #  under the License is distributed on an "as-is" basis, without warranties or
# #  conditions of any kind, either express or implied. See the License for the
specific  # #  language governing permissions and limitations under the License.
# # # #  IFRA Group - Cranfield University # #  AUTHORS: Mikel Bueno Viso -
Mikel.Bueno-Viso@cranfield.ac.uk                         # #           Dr.
Seemal Asif  - s.asif@cranfield.ac.uk                                   # #
Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 # #
# #  Date: June, 2023. # # # # ===================================== COPYRIGHT
===================================== #

# ===================================== COPYRIGHT
===================================== # # # #  Information and guidance on how
to implement a ROS-Gazebo ConveyorBelt plugin has   # #  been taken from the
usnistgov/ARIAC repo in GitHub. In this repository, the          # #  simulation
of a ConveyorBelt is already being simulated, and the source code can     # # be
found inside /ariac_plugins. This has been useful for the development of the #
#  IFRA_ConveyorBelt plugin, which has been desinged in order to comply with the
IFRA   # #  ROS-Gazebo Robot Simulation. # # # #  usnistgov/ARIAC repo in
GitHub:                                                      # #     Repository
for ARIAC (Agile Robotics for Industrial Automation Competition),      # #
consisting of kit building and assembly in a simulated warehouse. # # # #
Copyright (C) 2023, usnistgov/ARIAC # # # #
===================================== COPYRIGHT
===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) Gazebo-ROS Conveyor Belt Plugin. URL:
https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt.

*/

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>
#include <string>

#include "ros_conveyorbelt/ros_conveyorbelt_plugin.hpp" // Header file.

#include "conveyorbelt_msgs/ConveyorBeltControl.h" // ROS Service.
#include "conveyorbelt_msgs/ConveyorBeltState.h"   // ROS Message.

#include <memory>

namespace gazebo_ros {

class ROSConveyorBeltPluginPrivate {
public:
  // ROS node for communication, managed by gazebo_ros.
  std::shared_ptr<ros::NodeHandle> ros_node_;

  // The joint that controls the movement of the belt:
  gazebo::physics::JointPtr belt_joint_;

  // Additional parametres:
  double belt_velocity_;
  double max_velocity_;
  double power_;
  double limit_;

  // PUBLISH ConveyorBelt status:
  void PublishStatus();                             // Method to publish status.
  ros::Publisher status_pub_;                       // Publisher.
  conveyorbelt_msgs::ConveyorBeltState status_msg_; // ConveyorBelt status.

  // SET Conveyor Power:
  bool SetConveyorPower(
      conveyorbelt_msgs::ConveyorBeltControlRequest &,
      conveyorbelt_msgs::ConveyorBeltControlResponse &); // Method to execute
                                                         // service.
  ros::ServiceServer enable_service_;                    // ROS Service.

  // WORLD UPDATE event:
  void OnUpdate();
  ros::Time last_publish_time_;
  int update_ns_;
  gazebo::event::ConnectionPtr
      update_connection_; // Connection to world update event. Callback is
                          // called while this is alive.
};

ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
    : impl_(std::make_unique<ROSConveyorBeltPluginPrivate>()) {}

ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin() {}

void ROSConveyorBeltPlugin::Load(gazebo::physics::ModelPtr _model,
                                 sdf::ElementPtr _sdf) {

  std::string robot_namespace_ = "";
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED(
        "planar_move",
        "PlanarMovePlugin (ns = "
            << robot_namespace_
            << "). A ROS node for Gazebo has not been initialized, "
            << "unable to load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  // Create ROS node:
  impl_->ros_node_.reset(new ros::NodeHandle(""));

  // OBTAIN -> BELT JOINT:
  impl_->belt_joint_ = _model->GetJoint("belt_joint");

  if (!impl_->belt_joint_) {
    ROS_WARN("missing belt joint");
    return;
  }

  // Set velocity (m/s)
  impl_->max_velocity_ = _sdf->GetElement("max_velocity")->Get<double>();

  // Set limit (m)
  impl_->limit_ = impl_->belt_joint_->UpperLimit();

  // Create status publisher
  impl_->status_pub_ =
      impl_->ros_node_->advertise<conveyorbelt_msgs::ConveyorBeltState>(
          "CONVEYORSTATE", 10);
  impl_->status_msg_.enabled = false;
  impl_->status_msg_.power = 0;

  // REGISTER ConveyorBelt SERVICE:
  // auto fn = boost::bind(&ROSConveyorBeltPlugin::SetConveyorPower, this,
  // boost::placeholders::_1, boost::placeholders::_2);
  // auto fn = SetConveyorPowerTest;
  impl_->enable_service_ = impl_->ros_node_->advertiseService(
      "CONVEYORPOWER", &ROSConveyorBeltPluginPrivate::SetConveyorPower,
      impl_.get());

  double publish_rate = _sdf->GetElement("publish_rate")->Get<double>();
  impl_->update_ns_ = int((1 / publish_rate) * 1e9);

  impl_->last_publish_time_ = ros::Time::now();

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ROSConveyorBeltPluginPrivate::OnUpdate, impl_.get()));
}

void ROSConveyorBeltPluginPrivate::OnUpdate() {
  belt_joint_->SetVelocity(0, belt_velocity_);

  double belt_position = belt_joint_->Position(0);

  if (belt_position >= limit_) {
    belt_joint_->SetPosition(0, 0);
  }

  // Publish status at rate
  ros::Time now = ros::Time::now();
  if (now - last_publish_time_ >= ros::Duration(0, update_ns_)) {
    PublishStatus();
    last_publish_time_ = now;
  }
}

bool ROSConveyorBeltPluginPrivate::SetConveyorPower(
    conveyorbelt_msgs::ConveyorBeltControlRequest &req,
    conveyorbelt_msgs::ConveyorBeltControlResponse &res) {
  res.success = false;
  if (req.power >= 0 && req.power <= 100) {
    power_ = req.power;
    belt_velocity_ = max_velocity_ * (power_ / 100);
    res.success = true;
  }
  return true;
}

void ROSConveyorBeltPluginPrivate::PublishStatus() {
  status_msg_.power = power_;

  if (power_ > 0)
    status_msg_.enabled = true;
  else {
    status_msg_.enabled = false;
  }

  status_pub_.publish(status_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin)
} // namespace gazebo_ros