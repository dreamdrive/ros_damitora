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
// だみとらの5つのトラッカー座標をOCSに渡すノード
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
#define ADDRESS "192.168.7.106" // VMTのアドレス
#define PORT 39570              // VMTの待受ポート
#define OUTPUT_BUFFER_SIZE 1024

geometry_msgs::Pose poseC;
geometry_msgs::Pose poseR;
geometry_msgs::Pose poseL;
geometry_msgs::Pose poseR2;
geometry_msgs::Pose poseL2;

// コールバックがあるとグローバルに読み込み
void trackerC_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  poseC.position.x = msg->pose.position.x;
  poseC.position.y = msg->pose.position.y;
  poseC.position.z = msg->pose.position.z;
  poseC.orientation.x = msg->pose.orientation.x;
  poseC.orientation.y = msg->pose.orientation.y;
  poseC.orientation.z = msg->pose.orientation.z;
  poseC.orientation.w = msg->pose.orientation.w;
}

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

// コールバックがあるとグローバルに読み込み
void trackerR2_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  poseR2.position.x = msg->pose.position.x;
  poseR2.position.y = msg->pose.position.y;
  poseR2.position.z = msg->pose.position.z;
  poseR2.orientation.x = msg->pose.orientation.x;
  poseR2.orientation.y = msg->pose.orientation.y;
  poseR2.orientation.z = msg->pose.orientation.z;
  poseR2.orientation.w = msg->pose.orientation.w;
}

// コールバックがあるとグローバルに読み込み
void trackerL2_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  poseL2.position.x = msg->pose.position.x;
  poseL2.position.y = msg->pose.position.y;
  poseL2.position.z = msg->pose.position.z;
  poseL2.orientation.x = msg->pose.orientation.x;
  poseL2.orientation.y = msg->pose.orientation.y;
  poseL2.orientation.z = msg->pose.orientation.z;
  poseL2.orientation.w = msg->pose.orientation.w;
}

// クオタニオンからオイラー角に変換する関数
void QuaternionToEulerAngles(double q0, double q1, double q2, double q3, double &roll, double &pitch, double &yaw)
{
  double q0q0 = q0 * q0;
  double q0q1 = q0 * q1;
  double q0q2 = q0 * q2;
  double q0q3 = q0 * q3;
  double q1q1 = q1 * q1;
  double q1q2 = q1 * q2;
  double q1q3 = q1 * q3;
  double q2q2 = q2 * q2;
  double q2q3 = q2 * q3;
  double q3q3 = q3 * q3;
  roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
  // pitch = asin(2.0 * (q0q2 - q1q3));
  // yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
  // ROSからUnityで本来とピッチとヨーが逆になる、かつピッチの符号が逆になる
  yaw = asin(2.0 * (q0q2 - q1q3));
  pitch = atan2(2.0 * (q1q2 + q0q3), -(q0q0 + q1q1 - q2q2 - q3q3));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "damitora_osc"); // ノードの初期化
  ros::NodeHandle nh;                    // ノードハンドラ

  // rosparamからVMTのOSC送信先を取ってくる
  std::string vmt_ip;
  int vmt_port;
  nh.getParam("/vmt_ip", vmt_ip);
  nh.getParam("/vmt_port", vmt_port);

  // ROS_ERROR("IP : %s", vmt_ip.c_str());
  // ROS_ERROR("port : %d", vmt_port);


  // OSC -----------------------------

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

  float position_x3 = 0.0;
  float position_y3 = 0.0;
  float position_z3 = 0.0;
  float rotation_x3 = 0.0;
  float rotation_y3 = 0.0;
  float rotation_z3 = 0.0;
  float rotation_w3 = 1.0;

  float position_x4 = 0.0;
  float position_y4 = 0.0;
  float position_z4 = 0.0;
  float rotation_x4 = 0.0;
  float rotation_y4 = 0.0;
  float rotation_z4 = 0.0;
  float rotation_w4 = 1.0;

//  UdpTransmitSocket transmitSocket(IpEndpointName(ADDRESS, PORT));
  UdpTransmitSocket transmitSocket(IpEndpointName(vmt_ip.c_str(), vmt_port));
  char buffer[OUTPUT_BUFFER_SIZE];

  // OSC -----------------------------

  // だみとら表示用

  double rotation_roll;
  double rotation_pitch;
  double rotation_yaw;

  double rotation_roll1;
  double rotation_pitch1;
  double rotation_yaw1;

  double rotation_roll2;
  double rotation_pitch2;
  double rotation_yaw2;

  double rotation_roll3;
  double rotation_pitch3;
  double rotation_yaw3;

  double rotation_roll4;
  double rotation_pitch4;
  double rotation_yaw4;

  int deg_roll;
  int deg_pitch;
  int deg_yaw;

  int deg_roll1;
  int deg_pitch1;
  int deg_yaw1;

  int deg_roll2;
  int deg_pitch2;
  int deg_yaw2;

  int deg_roll3;
  int deg_pitch3;
  int deg_yaw3;

  int deg_roll4;
  int deg_pitch4;
  int deg_yaw4;

  // だみとら表示用

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerC;
  sub_trackerC = nh.subscribe("/pose_tracker_C/output", 60, trackerC_Callback);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerR;
  sub_trackerR = nh.subscribe("/pose_tracker_R/output", 60, trackerR_Callback);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerL;
  sub_trackerL = nh.subscribe("/pose_tracker_L/output", 60, trackerL_Callback);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerR2;
  sub_trackerR2 = nh.subscribe("/pose_tracker_R2/output", 60, trackerR2_Callback);

  //サブスクライバの作成 (移動先の指示)
  ros::Subscriber sub_trackerL2;
  sub_trackerL2 = nh.subscribe("/pose_tracker_L2/output", 60, trackerL2_Callback);

  ros::Rate loop_rate(60); // 制御周期60Hz

  while (ros::ok())
  {

    // ROSからUnityへの座標変換------------------------------
    // ROS -> Unity
    // Position: ROS(x,y,z) -> Unity(-y,z,x)
    // Quaternion: ROS(x,y,z,w) -> Unity(-y,z,x,-w)

    position_x = -poseC.position.y * 7;
    position_y = poseC.position.z * 7;
    position_z = poseC.position.x * 7;
    rotation_x = -poseC.orientation.y;
    rotation_y = poseC.orientation.z;
    rotation_z = poseC.orientation.x;
    rotation_w = -poseC.orientation.w;

    position_x1 = -poseR.position.y * 7;
    position_y1 = poseR.position.z * 7;
    position_z1 = poseR.position.x * 7;
    rotation_x1 = -poseR.orientation.y;
    rotation_y1 = poseR.orientation.z;
    rotation_z1 = poseR.orientation.x;
    rotation_w1 = -poseR.orientation.w;

    position_x2 = -poseL.position.y * 7;
    position_y2 = poseL.position.z * 7;
    position_z2 = poseL.position.x * 7;
    rotation_x2 = -poseL.orientation.y;
    rotation_y2 = poseL.orientation.z;
    rotation_z2 = poseL.orientation.x;
    rotation_w2 = -poseL.orientation.w;

    position_x3 = -poseR2.position.y * 7;
    position_y3 = poseR2.position.z * 7;
    position_z3 = poseR2.position.x * 7;
    rotation_x3 = -poseR2.orientation.y;
    rotation_y3 = poseR2.orientation.z;
    rotation_z3 = poseR2.orientation.x;
    rotation_w3 = -poseR2.orientation.w;

    position_x4 = -poseL2.position.y * 7;
    position_y4 = poseL2.position.z * 7;
    position_z4 = poseL2.position.x * 7;
    rotation_x4 = -poseL2.orientation.y;
    rotation_y4 = poseL2.orientation.z;
    rotation_z4 = poseL2.orientation.x;
    rotation_w4 = -poseL2.orientation.w;

    // ROSからUnityへの座標変換------------------------------

    // OSCの送信 -----------------------------------------------------

    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE); // 毎回初期化

    p << osc::BeginBundleImmediate
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)1 << (int)1 << (float)0 << position_x << position_y << position_z << rotation_x << rotation_y << rotation_z << rotation_w << "HMD" << osc::EndMessage
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)2 << (int)1 << (float)0 << position_x1 << position_y1 << position_z1 << rotation_x1 << rotation_y1 << rotation_z1 << rotation_w1 << "HMD" << osc::EndMessage
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)0 << (int)1 << (float)0 << position_x2 << position_y2 << position_z2 << rotation_x2 << rotation_y2 << rotation_z2 << rotation_w2 << "HMD" << osc::EndMessage
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)3 << (int)1 << (float)0 << position_x3 << position_y3 << position_z3 << rotation_x3 << rotation_y3 << rotation_z3 << rotation_w3 << "HMD" << osc::EndMessage
      << osc::BeginMessage("/VMT/Follow/Unity")
      << (int)4 << (int)1 << (float)0 << position_x4 << position_y4 << position_z4 << rotation_x4 << rotation_y4 << rotation_z4 << rotation_w4 << "HMD" << osc::EndMessage
      << osc::EndBundle;

    transmitSocket.Send(p.Data(), p.Size());

    // OSCの送信 -----------------------------------------------------

    // だみとら用

    // クオタニオンからオイラー角へ (rad)
    QuaternionToEulerAngles(rotation_x, rotation_y, rotation_z, rotation_w, rotation_roll, rotation_pitch, rotation_yaw);
    QuaternionToEulerAngles(rotation_x1, rotation_y1, rotation_z1, rotation_w1, rotation_roll1, rotation_pitch1, rotation_yaw1);
    QuaternionToEulerAngles(rotation_x2, rotation_y2, rotation_z2, rotation_w2, rotation_roll2, rotation_pitch2, rotation_yaw2);
    QuaternionToEulerAngles(rotation_x3, rotation_y3, rotation_z3, rotation_w3, rotation_roll3, rotation_pitch3, rotation_yaw3);
    QuaternionToEulerAngles(rotation_x4, rotation_y4, rotation_z4, rotation_w4, rotation_roll4, rotation_pitch4, rotation_yaw4);

    // rad to deg
    deg_roll = rotation_roll * (180 / M_PI);
    deg_pitch = rotation_pitch * (180 / M_PI);
    deg_yaw = rotation_yaw * (180 / M_PI);
    deg_roll1 = rotation_roll1 * (180 / M_PI);
    deg_pitch1 = rotation_pitch1 * (180 / M_PI);
    deg_yaw1 = rotation_yaw1 * (180 / M_PI);
    deg_roll2 = rotation_roll2 * (180 / M_PI);
    deg_pitch2 = rotation_pitch2 * (180 / M_PI);
    deg_yaw2 = rotation_yaw2 * (180 / M_PI);

    deg_roll3 = rotation_roll3 * (180 / M_PI);
    deg_pitch3 = rotation_pitch3 * (180 / M_PI);
    deg_yaw3 = rotation_yaw3 * (180 / M_PI);

    deg_roll4 = rotation_roll4 * (180 / M_PI);
    deg_pitch4 = rotation_pitch4 * (180 / M_PI);
    deg_yaw4 = rotation_yaw4 * (180 / M_PI);

    // ROSLAUNCHだと、INFOが表示されないため
    ROS_DEBUG("-------------------------------------");
    ROS_DEBUG("%d , %d , %d", (int)(position_x2 * 1000), (int)(position_y2 * 1000) + 1400, (int)(position_z2 * 1000));
    ROS_DEBUG("%d , %d , %d", deg_roll2, deg_yaw2, deg_pitch2);

    ROS_DEBUG("%d , %d , %d", (int)(position_x * 1000), (int)(position_y * 1000) + 1400, (int)(position_z * 1000));
    ROS_DEBUG("%d , %d , %d", deg_roll, deg_yaw, deg_pitch);

    ROS_DEBUG("%d , %d , %d", (int)(position_x1 * 1000), (int)(position_y1 * 1000) + 1400, (int)(position_z1 * 1000));
    ROS_DEBUG("%d , %d , %d", deg_roll1, deg_yaw1, deg_pitch1);

    ROS_DEBUG("%d , %d , %d", (int)(position_x3 * 1000), (int)(position_y3 * 1000) + 1400, (int)(position_z3 * 1000));
    ROS_DEBUG("%d , %d , %d", deg_roll3, deg_yaw3, deg_pitch3);

    ROS_DEBUG("%d , %d , %d", (int)(position_x4 * 1000), (int)(position_y4 * 1000) + 1400, (int)(position_z4 * 1000));
    ROS_DEBUG("%d , %d , %d", deg_roll4, deg_yaw4, deg_pitch4);

    // だみとら用

    ros::spinOnce(); // コールバック関数を呼ぶ
    loop_rate.sleep();
  }
  return 0;
}
