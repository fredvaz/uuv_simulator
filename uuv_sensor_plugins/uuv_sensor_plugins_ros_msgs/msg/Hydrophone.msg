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

# This is a message to hold data from a Hydrophone sensor with a signal processing unit.
# The name of this sensor maybe is not the better, because the output of a real hydrophone
# sensor is just a signal and in this case we are compute the time and other data.
#
# Arrival Time is in [s, ns], Loss and Noise is in [dB].
# Channel State is a scale [0-10] and Range Ground Truth is in [m].
#
# Hydrophone sensor receive an pressure wave in an arrival time per one range (Multi-path propagation).
#
# All Rays/Ranges with all arrival times per measurement maybe in future an implementation
# like as in DVLbeam msgs, that will correspond to Multi-path propagation more condensed
# but maybe not useful...

# Arrival Time [s, ns]
Header header
# Arrival Time [s]
# float64 arrival_time # Can be the time of flight?
# Transmission loss [dB]
float64 loss
# Noise of [Turbulence, Wind, Sipping Activity, Thermal Noise] [dB]
float64[4] noise
# Acoustic Channel State [0-10]
int64 channel_sate
# Truth distance from source/beacon to receiver/hydrophone [m]
float64 range_gt
