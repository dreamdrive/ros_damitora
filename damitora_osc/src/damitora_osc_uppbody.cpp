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
// だみとら物理コントローラーの代わりに別プロジェクトの「上半身ロボット」に接続して、上半身をコントロールするノード 
//   Steam VR側で、２つのトラッカーを左右のコントローラーにオーバーライドする設定が必要
//     POSEを拾ってOSCにパスする
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
#define ADDRESS "192.168.7.31"
#define PORT 39570
#define OUTPUT_BUFFER_SIZE 1024

//geometry_msgs::Pose poseC;
geometry_msgs::Pose poseR;
geometry_msgs::Pose poseL;


// コールバックがあるとグローバルに読み込み
void trackerR_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  poseR.position.x = msg->pose.position.x;
  poseR.position.y = msg->pose.position.y;
  poseR.position.z = msg->pose.position.z;
  poseR.orientation.x = msg->pose.orientation.x;
  poseR.orientation.y = msg->pose.orientation.y;
  poseR.orientation.z = msg->pose.orientation.z;
  poseR.orientation.w = msg->pose.orientation.w;
}

// コールバックがあるとグローバルに読み込み
void trackerL_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  poseL.position.x = msg->pose.position.x;
  poseL.position.y = msg->pose.position.y;
  poseL.position.z = msg->pose.position.z;
  poseL.orientation.x = msg->pose.orientation.x;
  poseL.orientation.y = msg->pose.orientation.y;
  poseL.orientation.z = msg->pose.orientation.z;
  poseL.orientation.w = msg->pose.orientation.w;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "damitora_osc"); // ノードの初期化
  ros::NodeHandle nh;                    // ノードハンドラ

  // OSC -----------------------------

  int i = 0;

  float position_x1 = 0.0;
  float position_y1 = 0.0;
  float position_z1 = 0.0;
  float rotation_x1 = 0.0;
  float rotation_y1 = 0.0;
  float rotation_z1 = 0.0;
  float rotation_w1 = 1.0;

  float position_x2 = 0.0;
  float position_y2 = 0.0;
  float position_z2 = 0.0;
  float rotation_x2 = 0.0;
  float rotation_y2 = 0.0;
  float rotation_z2 = 0.0;
  float rotation_w2 = 1.0;

  UdpTransmitSocket transmitSocket(IpEndpointName(ADDRESS, PORT));
  char buffer[OUTPUT_BUFFER_SIZE];

  // OSC -----------------------------

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerR;
  sub_trackerR = nh.subscribe("/pose_tracker_R/output", 60, trackerR_Callback);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerL;
  sub_trackerL = nh.subscribe("/pose_tracker_L/output", 60, trackerL_Callback);

  ros::Rate loop_rate(60); // 制御周期60Hz

  while (ros::ok())
  {

    position_x1 = -poseR.position.y * 3;
    position_y1 = poseR.position.z * 3 - 0.9;
    position_z1 = poseR.position.x * 3;
    rotation_x1 = -poseR.orientation.y;
    rotation_y1 = poseR.orientation.z;
    rotation_z1 = poseR.orientation.x;
    rotation_w1 = -poseR.orientation.w;

    position_x2 = -poseL.position.y * 3;
    position_y2 = poseL.position.z * 3 - 0.9;
    position_z2 = poseL.position.x * 3;
    rotation_x2 = -poseL.orientation.y;
    rotation_y2 = poseL.orientation.z;
    rotation_z2 = poseL.orientation.x;
    rotation_w2 = -poseL.orientation.w;

    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE); // 毎回初期化

    p << osc::BeginBundleImmediate
      // << osc::BeginMessage("/VMT/Follow/Unity")
      // << (int)0 << (int)1 << (float)0 << position_x << position_y << position_z << rotation_x << rotation_y << rotation_z << rotation_w << "HMD" << osc::EndMessage
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)0 << (int)1 << (float)0 << position_x1 << position_y1 << position_z1 << rotation_x1 << rotation_y1 << rotation_z1 << rotation_w1 << "HMD" << osc::EndMessage
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)1 << (int)1 << (float)0 << position_x2 << position_y2 << position_z2 << rotation_x2 << rotation_y2 << rotation_z2 << rotation_w2 << "HMD" << osc::EndMessage
      << osc::EndBundle;

    transmitSocket.Send(p.Data(), p.Size());

    ros::spinOnce(); // コールバック関数を呼ぶ
    loop_rate.sleep();
  }
  return 0;
}
