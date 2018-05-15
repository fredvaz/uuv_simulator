// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __UUV_HYDROPHONE_ROS_PLUGIN_HH__
#define __UUV_HYDROPHONE_ROS_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <uuv_sensor_ros_plugins/ROSBaseModelPlugin.hh>
#include "SensorHydrophone.pb.h"
#include <uuv_sensor_plugins_ros_msgs/Hydrophone.h>

namespace gazebo
{
  class HydrophoneROSPlugin : public ROSBaseModelPlugin
  {
    /// \brief Class constructor
    public: HydrophoneROSPlugin();

    /// \brief Class destructor
    public: ~HydrophoneROSPlugin();

    /// \brief Load the plugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update sensor measurement
    protected: virtual bool OnUpdate(const common::UpdateInfo& _info);

    /// \brief Gazebo's subscriber for transporting measurement messages.
    protected: transport::SubscriberPtr gazeboSensorInputSub;

    /// \brief Update callback from simulator.
    // protected: virtual void OnTrigger(
    //   const uuv_sensor_plugins_ros_msgs::Beacon::ConstPtr &_msg);

    // TEST: Get the Link of beacon to get the his position

    // \brief Pointer to the Beacon model (Acoustic Source). TESTING
    protected: physics::ModelPtr beacon_model;

    // \brief Pointer to the Beacon link (Acoustic Source). TESTING
    protected: physics::LinkPtr beacon_link;

    // SDF PARAMETERS to C++ VARIABLES

    /// \brief Acoustic Source
    protected: std::string beacon_parent;

    /// \brief Sensor sensitivity (re 1V/uPa)
    protected: double sensitivity;

    /// \brief Pre-amplificator Gain
    protected: double pre_ampGain;

    /// \brief Hydrophone Cut Off Frequency
    protected: double cut_off_frequency;

    /// \brief If flag is set to true, give the truth range
    /// measurement
    protected: bool show_range_gt;

    /// \brief ROS Hydrophone message GLOBAL for all Class's
    // protected: uuv_sensor_plugins_ros_msgs::Hydrophone hydrophoneROSMsg;

    //protected: std::vector<uuv_sensor_plugins_ros_msgs::DVLBeam> dvlBeamMsgs;

  };
}

#endif // __UUV_HYDROPHONE_ROS_PLUGIN_HH__
