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

#include <uuv_sensor_ros_plugins/HydrophoneROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
HydrophoneROSPlugin::HydrophoneROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
HydrophoneROSPlugin::~HydrophoneROSPlugin()
{ }

/////////////////////////////////////////////////
void HydrophoneROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  // SDF PARAMETERS to C++ VARIABLES
  GetSDFParam<std::string>(_sdf, "beacon_parent", this->beacon_parent, "bluerov2");
  GetSDFParam<double>(_sdf, "sensitivity", this->sensitivity, -190.0);
  GetSDFParam<double>(_sdf, "pre_ampGain", this->pre_ampGain, 20.0);
  GetSDFParam<double>(_sdf, "cut_off_frequency", this->cut_off_frequency, 10000);
  GetSDFParam<bool>(_sdf, "show_range_gt", this->show_range_gt, false);

  // In the Future we can create a Class that finds every Beacons Sensors in the World
  // With that will be possible create others types of Acoustic Localization Systems
  // And principal for a Multi-Robot Localization with this system
  // e.g Long Base Line (LBL) -> Beacons on the seabed
  // OR we can subscribe all beacon topics but we just need the position of an acoustic source
  // And a trigger so get that with GetLink can be a better choice ?
  // BUT! For two cases we must make a multi-source model too
  // for the acoustic propagation
  this->beacon_model = this->world->GetModel(this->beacon_parent);
  this->beacon_link = this->beacon_model->GetLink(this->beacon_parent + "/beacon_link");


  // From another way, possible we are a introduct a possible position shift
  // because, when computer reads here the position is not exactly equal when signal
  // as triggered. So we will assume that when comes a topic message from the Beacon
  // we are ready to compute, and publish Hydrophone message when his time is equal
  // to the computed arrival time

  // this->beaconSub = this->rosNode->subscribe<uuv_sensor_plugins_ros_msgs::Beacon>(
  //   this->beacon_parent + "/beacon", 1,
  //   boost::bind(&HydrophoneROSPlugin::OnTrigger,
  //     this, _1));

  // IS BETTER A GAZEBO TOPIC - SUBSCRIBER
  // ROS SUBSCRIBER - ALTERNATIVE

  // ROS Output Topic
  this->rosSensorOutputPub =
    this->rosNode->advertise<uuv_sensor_plugins_ros_msgs::Hydrophone>(
      this->sensorOutputTopic, 1);

  // Gazebo output Msg
  if (this->gazeboMsgEnabled)
  {
    this->gazeboSensorOutputPub =
      this->gazeboNode->Advertise<sensor_msgs::msgs::Hydrophone>(
          this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }
}

/////////////////////////////////////////////////
bool HydrophoneROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  // Get Environmet Variables
  double temperature = 0.0;
  double salinaty = 0.0;
  double pH = 0.0;
  // Sound velocity
  double c = 0.0;
  // Absorption coefficient
  double a = 0.0;


  // Using the world pose wrt Gazebo's ENU reference frame
  ignition::math::Vector3d hydrophone_pos, beacon_pos;

#if GAZEBO_MAJOR_VERSION >= 8
  hydrophone_pos = this->link->WorldPose().Pos();
  beacon_pos = this->beacon_link->WorldPose().Pos();
#else
  hydrophone_pos = this->link->GetWorldPose().Ign().Pos();
  beacon_pos = this->beacon_link->GetWorldPose().Ign().Pos();
#endif

  // TEST the position sended by the beacon and the reached here!

  // Get the Horizontal range (D) a and compute the truth range (R) all in [m]
  // corresponding to the Radius Spheferical of the sound propagation model
  // 1st Case
  double D = std::sqrt(std::pow(hydrophone_pos.X() - beacon_pos.X(), 2) + std::pow(hydrophone_pos.Y() - beacon_pos.Y(), 2));
  double R = std::sqrt(std::pow(D, 2) + std::pow(hydrophone_pos.Z() - beacon_pos.Z(), 2));

  // Transmission Loss (TL) or Propagation Loss in [dB]
  double loss = -(20.0 * std::log(R) + a * R);


  double noise[4] = { 0 , 0, 0, 0 };

  // A Simple aproximation -> TO IMPROVE IT with NOISE
  // loss -> max 200 dB
  // channel_sate -> 10
  double CH_state = 10 - (((-loss) * 10) / 200);


  // Compute
  double arrival_time = 0.0;

  // When comes trigger from Beacon
  // if(){
  //
  //
  // }


  // Publish Gazebo pressure message, if enabled
  if (this->gazeboMsgEnabled)
  {
    sensor_msgs::msgs::Hydrophone gazeboMsg;

    gazeboMsg.set_arrival_time(arrival_time);
    gazeboMsg.set_loss(loss);
    gazeboMsg.set_noise(noise[4]);
    gazeboMsg.set_channel_sate(CH_state);
    // range_gt is optional
    if (this->show_range_gt)
      gazeboMsg.set_range_gt(R);

    this->gazeboSensorOutputPub->Publish(gazeboMsg);
  }

  // Publish ROS pressure message
  uuv_sensor_plugins_ros_msgs::Hydrophone rosMsg;

  // We must test what time is best to use, currentTime like GPS for sicronization ?
  rosMsg.header.stamp.sec  = _info.simTime.sec;
  rosMsg.header.stamp.nsec = _info.simTime.nsec;
  rosMsg.header.frame_id = this->link->GetName();

  // this->hydrophoneROSMsg.arrival_time = pressure; // # Can be the time of flight?
  // It's necessary understand if it's better publish when signal comes, or after with
  // the time of flight beacause of computation time and sicronization...
  // Arrival time != Time of Flight!!!!

  rosMsg.loss = loss;

  rosMsg.noise[0] = 0.0;
  rosMsg.noise[1] = 0.0;
  rosMsg.noise[2] = 0.0;
  rosMsg.noise[3] = 0.0;
  rosMsg.channel_sate = CH_state;
  rosMsg.range_gt = R;

  this->rosSensorOutputPub.publish(rosMsg);

  // Read the current simulation time
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastMeasurementTime = this->world->SimTime();
#else
  this->lastMeasurementTime = this->world->GetSimTime();
#endif
  return true;
}

/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(HydrophoneROSPlugin)
}
