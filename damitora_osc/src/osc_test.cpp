// Copyright 2020-2021 Dream Drive !!
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

// 
// VMTに対して、OSCでトラッカー情報を渡すテストノード
//   3つのトラッカー情報は、円を描いて動き続ける 
//

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <math.h>
#include <std_msgs/String.h>

// ----

#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#define PI 3.14159265358979
#define ADDRESS "192.168.253.144"
#define PORT 39570
#define OUTPUT_BUFFER_SIZE 1024

int main(int argc, char **argv)
{
  ros::init(argc, argv, "osc_test"); // ノードの初期化
  ros::NodeHandle nh;                // ノードハンドラ

  // rosparamからVMTのOSC送信先を取ってくる
  std::string vmt_ip;
  int vmt_port;
  nh.getParam("/vmt_ip", vmt_ip);
  nh.getParam("/vmt_port", vmt_port);

  // ROS_ERROR("IP : %s", vmt_ip.c_str());
  // ROS_ERROR("port : %d", vmt_port);

  geometry_msgs::PoseStamped poseC;
  geometry_msgs::PoseStamped poseR;
  geometry_msgs::PoseStamped poseL;

  poseC.pose.position.x = 0;
  poseC.pose.position.y = 0;
  poseC.pose.position.z = 0;
  poseC.pose.orientation.x = 0;
  poseC.pose.orientation.y = 0;
  poseC.pose.orientation.z = 0;
  poseC.pose.orientation.w = 1.0;

  poseR.pose.position.x = 0;
  poseR.pose.position.y = 0;
  poseR.pose.position.z = 0;
  poseR.pose.orientation.x = 0;
  poseR.pose.orientation.y = 0;
  poseR.pose.orientation.z = 0;
  poseR.pose.orientation.w = 1.0;

  poseL.pose.position.x = 0;
  poseL.pose.position.y = 0;
  poseL.pose.position.z = 0;
  poseL.pose.orientation.x = 0;
  poseL.pose.orientation.y = 0;
  poseL.pose.orientation.z = 0;
  poseL.pose.orientation.w = 1.0;

  // OSC

  int i = 0;

  float position_x = 0.0;
  float position_y = 0.0;
  float position_z = 0.0;
  float rotation_x = 0.0;
  float rotation_y = 0.0;
  float rotation_z = 0.0;
  float rotation_w = 1.0;

  float position_x1 = 0.0;
  float position_y1 = 0.0;
  float position_z1 = 0.0;

  float position_x2 = 0.0;
  float position_y2 = 0.0;
  float position_z2 = 0.0;


  UdpTransmitSocket transmitSocket(IpEndpointName(vmt_ip.c_str(), vmt_port));
  char buffer[OUTPUT_BUFFER_SIZE];

  // OSC

  ros::Publisher pub_poseC;
  ros::Publisher pub_poseR;
  ros::Publisher pub_poseL;
  pub_poseC = nh.advertise<geometry_msgs::PoseStamped>("/tracker_C", 1);
  pub_poseR = nh.advertise<geometry_msgs::PoseStamped>("/tracker_R", 1);
  pub_poseL = nh.advertise<geometry_msgs::PoseStamped>("/tracker_L", 1);

  ros::Rate loop_rate(60); // 制御周期60Hz

  while (ros::ok())
  {

    i++;
    if (i == 360) i = 0;  // カウンタリセット

    position_x = 0.5 * (float)sin(PI * i / 180);
    position_z = 0.5 * (float)cos(PI * i / 180);
    position_x1 = 0.5 * (float)sin(PI * (i + 120) / 180);
    position_z1 = 0.5 * (float)cos(PI * (i + 120) / 180);
    position_x2 = 0.5 * (float)sin(PI * (i + 240) / 180);
    position_z2 = 0.5 * (float)cos(PI * (i + 240) / 180);

    poseC.pose.position.x = position_x;
    poseC.pose.position.y = position_z;
    poseC.pose.position.z = 0;

    poseR.pose.position.x = position_x1;
    poseR.pose.position.y = position_z1;
    poseR.pose.position.z = 0;

    poseL.pose.position.x = position_x2;
    poseL.pose.position.y = position_z2;
    poseL.pose.position.z = 0;

    ROS_DEBUG("pos0 : %2.5f , %2.5f | ", position_x, position_z);
    ROS_DEBUG("pos1 : %2.5f , %2.5f | ", position_x1, position_z1);
    ROS_DEBUG("pos2 : %2.5f , %2.5f\n", position_x2, position_z2);

    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE); // 毎回初期化

    p << osc::BeginBundleImmediate
      << osc::BeginMessage("/VMT/Room/Unity")
      << (int)0 << (int)1 << (float)0 << position_x << position_y << position_z << rotation_x << rotation_y << rotation_z << rotation_w << osc::EndMessage
      << osc::BeginMessage("/VMT/Room/Unity")
      << (int)1 << (int)1 << (float)0 << position_x1 << position_y1 << position_z1 << rotation_x << rotation_y << rotation_z << rotation_w << osc::EndMessage
      << osc::BeginMessage("/VMT/Room/Unity")
      << (int)2 << (int)1 << (float)0 << position_x2 << position_y2 << position_z2 << rotation_x << rotation_y << rotation_z << rotation_w << osc::EndMessage
      << osc::EndBundle;

    transmitSocket.Send(p.Data(), p.Size());
    // usleep(10 * 1000);

    poseC.header.stamp = ros::Time::now();
    poseR.header.stamp = ros::Time::now();
    poseL.header.stamp = ros::Time::now();

    poseC.header.frame_id = "map";
    poseR.header.frame_id = "map";
    poseL.header.frame_id = "map";

    pub_poseC.publish(poseC);
    pub_poseR.publish(poseR);
    pub_poseL.publish(poseL);

    ros::spinOnce(); // コールバック関数を呼ぶ
    loop_rate.sleep();
  }
  return 0;
}
