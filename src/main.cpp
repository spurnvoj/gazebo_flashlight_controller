/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

gazebo::math::Pose offset_pose;
gazebo::math::Quaternion offset_rot;
gazebo::transport::PublisherPtr pub;

void publish_light_pose(const gazebo::math::Pose &pose, const char* name){
  gazebo::msgs::Light msg;
  msg.set_name(name);

  // Convert to a pose message
  gazebo::msgs::Set(msg.mutable_pose(), pose);
  pub->Publish(msg);
}

// Function is called everytime a message is received.
void cb(ConstPosesStampedPtr &_msg){
  
  for(int i=0; i<_msg->pose_size(); i++){
    if (_msg->pose(i).name()=="uav2" || _msg->pose(i).name()=="uav3"){
      gazebo::math::Vector3 math_pos = gazebo::math::Vector3(_msg->pose(i).position().x(),
                                                       _msg->pose(i).position().y(),
                                                       _msg->pose(i).position().z());
      
      gazebo::math::Quaternion math_quat = gazebo::math::Quaternion(_msg->pose(i).orientation().w(),
                                                       _msg->pose(i).orientation().x(),
                                                       _msg->pose(i).orientation().y(),
                                                       _msg->pose(i).orientation().z());
      gazebo::math::Pose new_pose;
      new_pose.Set(math_pos, math_quat);
      gazebo::math::Pose new_pos = new_pose + offset_pose;                                                
      //gazebo::math::Vector3 new_position = new_pose.CoordPositionAdd(offset_pose);                                                
      //gazebo::math::Quaternion new_orintation = new_pose.CoordRotationAdd(offset_rot);                                                
      //new_pose.Set(new_position, new_orintation);
      std::cout << new_pos << std::endl; 
      if (_msg->pose(i).name()=="uav2"){
        publish_light_pose(new_pos,"light_1");
      }else{
        publish_light_pose(new_pos,"light_2");
      }
    
    }
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv){
  
  // initialize node and create no handle
  ros::init(_argc, _argv, "gazebo_flashlight_controller");
  ros::NodeHandle nh = ros::NodeHandle("~");

  double x,y,z,roll,pitch,yaw;
  nh.param("offset/x", x, double(0));
  nh.param("offset/y", y, double(0));
  nh.param("offset/z", z, double(0));
  nh.param("offset/roll", roll, double(0));
  nh.param("offset/pitch", pitch, double(0));
  nh.param("offset/yaw", yaw, double(0));
      
  offset_pose = gazebo::math::Pose(x, y, z, 0, 0, 0);
  offset_rot= gazebo::math::Quaternion(roll, pitch, yaw);
  
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", cb);
  
  pub = node->Advertise<gazebo::msgs::Light>("~/light");
  
  // Wait for a subscriber to connect
  pub->WaitForConnection();
  
  // Busy wait loop...replace with your own code as needed.
  while (ros::ok())
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
