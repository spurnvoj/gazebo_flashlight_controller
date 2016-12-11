/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <iostream>


/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
//  gazebo::load(_argc, _argv);
  gazebo::setupClient(_argc, _argv);
  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Start transport
  gazebo::transport::run();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Light>("~/light");

  // Wait for a subscriber to connect
  pub->WaitForConnection();

  // Publisher loop...replace with your own code.
  while (true)
  {
    // Throttle Publication
    gazebo::common::Time::MSleep(100);
    
    gazebo::msgs::Light msg;
    msg.set_name("light_1");

    // Generate a pose
    gazebo::math::Pose pose(1, 2, 3, 4, 5, 6);

    // Convert to a pose message
    gazebo::msgs::Set(msg.mutable_pose(), pose);

    pub->Publish(msg);
  }

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
