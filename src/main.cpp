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
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <mutex>


#include <iostream>

gazebo::math::Pose offset_pose_uav2, offset_pose_uav3;
gazebo::transport::PublisherPtr pub;
std::mutex pos_uav2_mutex, pos_uav3_mutex;

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
      gazebo::math::Pose actual_pose;
      actual_pose.Set(math_pos, math_quat);
      gazebo::math::Pose new_pose;

      if (_msg->pose(i).name()=="uav2"){

        pos_uav2_mutex.lock();
        {
          new_pose = offset_pose_uav2 + actual_pose;                                                
        }
        pos_uav2_mutex.unlock();
        publish_light_pose(new_pose,"light_1");

      }else{

        pos_uav3_mutex.lock();
        {
          new_pose = offset_pose_uav3 + actual_pose;                                                
        }
        pos_uav3_mutex.unlock();
        publish_light_pose(new_pose,"light_2");

      }
    }
  }
}

void pose_uav2_offset_cb(const geometry_msgs::PoseConstPtr &_msg){
  pos_uav2_mutex.lock();
  {
    double roll,pitch,yaw;

    tf::Quaternion q(
        _msg->orientation.x,
        _msg->orientation.y,
        _msg->orientation.z,
        _msg->orientation.w
        );

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    if(std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw)){
      offset_pose_uav2 = gazebo::math::Pose(
          _msg->position.x,
          _msg->position.y,
          _msg->position.z,
          roll,
          pitch,
          yaw
          );
    }
  }
  pos_uav2_mutex.unlock();
}

void pose_uav3_offset_cb(const geometry_msgs::PoseConstPtr &_msg){
  pos_uav3_mutex.lock();
  {
    double roll,pitch,yaw;

    tf::Quaternion q(
        _msg->orientation.x,
        _msg->orientation.y,
        _msg->orientation.z,
        _msg->orientation.w
        );

    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    if(std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw)){
      offset_pose_uav3 = gazebo::math::Pose(
          _msg->position.x,
          _msg->position.y,
          _msg->position.z,
          roll,
          pitch,
          yaw
          );
    }
  }
  pos_uav3_mutex.unlock();
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

  pos_uav2_mutex.lock();
  {
    offset_pose_uav2 = gazebo::math::Pose(x, y, z, roll, pitch, yaw);
  }
  pos_uav2_mutex.unlock();

  pos_uav3_mutex.lock();
  {
    offset_pose_uav3 = gazebo::math::Pose(x, y, z, roll, pitch, yaw);
  }
  pos_uav3_mutex.unlock();

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

  ros::Subscriber sub_offset_uav2 = nh.subscribe("pose_offset_uav2", 1, pose_uav2_offset_cb, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_offset_uav3 = nh.subscribe("pose_offset_uav3", 1, pose_uav3_offset_cb, ros::TransportHints().tcpNoDelay());

  // Busy wait loop...replace with your own code as needed.
  while (ros::ok()){
    ros::spinOnce();
    gazebo::common::Time::MSleep(10);
  }
  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
