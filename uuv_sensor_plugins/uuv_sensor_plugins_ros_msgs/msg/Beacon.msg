# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This message is the trigger to a Beacon Sensor from ROS.
# Hold some experimental data too.
# The definition as a sensor maybe is not the better, because it's sends
# a pressure wave to the environment surround.
#
# Operating Transmitted Frequency [Hz].
#
# For Beacon 1319A can transport some specific onfig data.
#

Header header
# Operating Transmitted Frequency [Hz] [dB]
float32 operating_frequency
# Triggered Time [s, ns] TEST DELAY between Beacon triggered time
std_msgs/Time triggered_time
