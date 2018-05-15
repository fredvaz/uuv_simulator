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

#include <uuv_sensor_ros_plugins/BeaconROSPlugin.hh>

namespace gazebo
{
/////////////////////////////////////////////////
BeaconROSPlugin::BeaconROSPlugin() : ROSBaseModelPlugin()
{ }

/////////////////////////////////////////////////
BeaconROSPlugin::~BeaconROSPlugin()
{ }

/////////////////////////////////////////////////
void BeaconROSPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  ROSBaseModelPlugin::Load(_model, _sdf);

  // SDF PARAMETERS to C++ VARIABLES
  GetSDFParam<double>(_sdf, "operating_frequency", this->operating_frequency, 10000.0);
  GetSDFParam<double>(_sdf, "spl", this->spl, 183.0);
  GetSDFParam<double>(_sdf, "beam_pattern", this->beam_pattern, 180.0);

  // ROS Input Topic
  this->rosSensorInputSub = this->rosNode->subscribe<uuv_sensor_plugins_ros_msgs::Beacon>(
    "/bluerov2/beacon/trigger", 1,
    boost::bind(&BeaconROSPlugin::OnTrigger,
      this, _1)); // this->robotNamespace +

  // Gazebo output Msg
  if (this->gazeboMsgEnabled) // FALSE
  {
    this->gazeboSensorOutputPub = this->gazeboNode->Advertise<uuv_sensor_gazebo_msgs::msgs::Beacon>("~/" + this->sensorOutputTopic);

      //<sensor_msgs::msgs::Beacon>( "~/collision_map/image"
     // this->robotNamespace + "/" + this->sensorOutputTopic, 1);
  }else{
    // ROS Output Topic -> ALTERNATIVE: FIX GAZEBO TOPICS
    this->rosSensorOutputPub =
      this->rosNode->advertise<uuv_sensor_plugins_ros_msgs::BeaconParameters>(
        this->sensorOutputTopic, 1);
  }
}
/////////////////////////////////////////////////
bool BeaconROSPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Publish sensor state
  this->PublishState();

  if (!this->EnableMeasurement(_info))
    return false;

  // NOTHING TO MEASURE - MAYBE NOISE OR OTHER SOMETHING

  common::Time triggered_time;

  triggered_time.sec = _info.simTime.sec;
  triggered_time.nsec = _info.simTime.nsec;

  // Read the current simulation time
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastMeasurementTime = this->world->SimTime();
  #else
    this->lastMeasurementTime = this->world->GetSimTime();
  #endif
    return true;
}
/////////////////////////////////////////////////
void BeaconROSPlugin::OnTrigger(const uuv_sensor_plugins_ros_msgs::Beacon::ConstPtr &_msg){

  // Using the world pose wrt Gazebo's ENU reference frame
  ignition::math::Vector3d beacon_pos;

  // Get Beacon Pose
#if GAZEBO_MAJOR_VERSION >= 8
  beacon_pos = this->link->WorldPose().Pos();
#else
  beacon_pos = this->link->GetWorldPose().Ign().Pos();
#endif

  // Get Sound Pressure Wave Time TEST IF IS BETTER WITH ROS TRIGGER TIME _msg->operating_frequency
  common::Time triggered_time;

  // Read the current simulation time
  #if GAZEBO_MAJOR_VERSION >= 8
    triggered_time = this->world->SimTime();
  #else
    triggered_time = this->world->GetSimTime();
  #endif

  // Publish Gazebo pressure message
  if (this->gazeboMsgEnabled) // this->gazeboMsgEnabled
  {
    uuv_sensor_gazebo_msgs::msgs::Beacon gazeboMsg;

    // Publish simulated measurement
    gazebo::msgs::Time* _time = new gazebo::msgs::Time();
    _time->set_sec(triggered_time.sec);
    _time->set_nsec(triggered_time.nsec);

    gazebo::msgs::Vector3d* pos = new gazebo::msgs::Vector3d();
    pos->set_x(beacon_pos.X());
    pos->set_y(beacon_pos.Y());
    pos->set_z(beacon_pos.Z());

    gazeboMsg.set_allocated_triggered_time(_time);
    gazeboMsg.set_allocated_triggered_pose(pos);
    gazeboMsg.set_operating_frequency(operating_frequency);
    gazeboMsg.set_spl(spl);
    gazeboMsg.set_beam_pattern(beam_pattern);

    this->gazeboSensorOutputPub->Publish(gazeboMsg);
  }else{

    // Publish ROS pressure message - ALTERNATIVE
    uuv_sensor_plugins_ros_msgs::BeaconParameters rosMsg;

    // We must test what time is best to use, currentTime like GPS for sicronization ?
    rosMsg.header.stamp.sec  = triggered_time.sec; //_info.simTime.sec;
    rosMsg.header.stamp.nsec = triggered_time.nsec; //_info.simTime.nsec;
    rosMsg.header.frame_id = this->link->GetName();

    // Arrival time != Time of Flight!!!!
    //rosMsg.triggered_time.sec = triggered_time.sec;
    //rosMsg.triggered_time.nsec = triggered_time.nsec;
    rosMsg.triggered_pose.x = beacon_pos.X();
    rosMsg.triggered_pose.y = beacon_pos.Y();
    rosMsg.triggered_pose.z = beacon_pos.Z();

    // Send Beacon Parameters
    if (_msg->operating_frequency != this->operating_frequency) {
      rosMsg.operating_frequency = _msg->operating_frequency;
    }else{
      rosMsg.operating_frequency = this->operating_frequency;
    }
    rosMsg.spl = spl;
    rosMsg.beam_pattern = this->beam_pattern;

    this->rosSensorOutputPub.publish(rosMsg);
  }
}
/////////////////////////////////////////////////
GZ_REGISTER_MODEL_PLUGIN(BeaconROSPlugin)
}
