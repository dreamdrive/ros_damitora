// Copyright 2020-2021 SETOUCHI ROS STUDY GROUP
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

// POSEを拾ってOSCにパスする

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
#define ADDRESS "192.168.7.106"      // VMTのアドレス
#define PORT 39570                  // VMTの待受ポート
#define OUTPUT_BUFFER_SIZE 1024

geometry_msgs::Pose poseC;
geometry_msgs::Pose poseR;
geometry_msgs::Pose poseL;

// コールバックがあるとグローバルに読み込み
void trackerC_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
void trackerR_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
void trackerL_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  poseL.position.x = msg->pose.position.x;
  poseL.position.y = msg->pose.position.y;
  poseL.position.z = msg->pose.position.z;
  poseL.orientation.x = msg->pose.orientation.x;
  poseL.orientation.y = msg->pose.orientation.y;
  poseL.orientation.z = msg->pose.orientation.z;
  poseL.orientation.w = msg->pose.orientation.w;
}

// クオタニオンからオイラー角に変換する関数
void QuaternionToEulerAngles(double q0, double q1, double q2, double q3, double& roll, double& pitch, double& yaw)
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
  ros::NodeHandle nh;                // ノードハンドラ


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

  //(void)argc; // suppress unused parameter warnings
  //(void)argv; // suppress unused parameter warnings

  UdpTransmitSocket transmitSocket(IpEndpointName(ADDRESS, PORT));
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

  int deg_roll;
  int deg_pitch;
  int deg_yaw;

  int deg_roll1;
  int deg_pitch1;
  int deg_yaw1;

  int deg_roll2;
  int deg_pitch2;
  int deg_yaw2;

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


  ros::Rate loop_rate(60); // 制御周期60Hz

  while (ros::ok())
  {

    // ROSからUnityへの座標変換------------------------------
    // ROS -> Unity
    // Position: ROS(x,y,z) -> Unity(-y,z,x)
    // Quaternion: ROS(x,y,z,w) -> Unity(-y,z,x,-w)

    position_x = - poseC.position.y * 7;
    position_y = poseC.position.z * 7;
    position_z = poseC.position.x * 7;
    rotation_x = - poseC.orientation.y;
    rotation_y = poseC.orientation.z;
    rotation_z = poseC.orientation.x;
    rotation_w = - poseC.orientation.w;

    position_x1 = - poseR.position.y * 7;
    position_y1 = poseR.position.z * 7;
    position_z1 = poseR.position.x * 7;
    rotation_x1 = - poseR.orientation.y;
    rotation_y1 = poseR.orientation.z;
    rotation_z1 = poseR.orientation.x;
    rotation_w1 = - poseR.orientation.w;

    position_x2 = - poseL.position.y * 7;
    position_y2 = poseL.position.z * 7;
    position_z2 = poseL.position.x * 7;
    rotation_x2 = - poseL.orientation.y;
    rotation_y2 = poseL.orientation.z;
    rotation_z2 = poseL.orientation.x;
    rotation_w2 = - poseL.orientation.w;

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
      << osc::EndBundle;

    transmitSocket.Send(p.Data(), p.Size());

    // OSCの送信 -----------------------------------------------------

    // だみとら用

    // クオタニオンからオイラー角へ (rad)
    QuaternionToEulerAngles(rotation_x, rotation_y, rotation_z, rotation_w, rotation_roll, rotation_pitch, rotation_yaw);
    QuaternionToEulerAngles(rotation_x1, rotation_y1, rotation_z1, rotation_w1, rotation_roll1, rotation_pitch1, rotation_yaw1);
    QuaternionToEulerAngles(rotation_x2, rotation_y2, rotation_z2, rotation_w2, rotation_roll2, rotation_pitch2, rotation_yaw2);

    // rad to deg
    deg_roll = rotation_roll*(180/M_PI);
    deg_pitch = rotation_pitch*(180/M_PI);
    deg_yaw = rotation_yaw*(180/M_PI);
    deg_roll1 = rotation_roll1*(180/M_PI);
    deg_pitch1 = rotation_pitch1*(180/M_PI);
    deg_yaw1 = rotation_yaw1*(180/M_PI);
    deg_roll2 = rotation_roll2*(180/M_PI);
    deg_pitch2 = rotation_pitch2*(180/M_PI);
    deg_yaw2 = rotation_yaw2*(180/M_PI);


    ROS_ERROR("-------------------------------------");
    ROS_ERROR("%d , %d , %d",(int)(position_x2*1000), (int)(position_y2*1000)+1400,(int)(position_z2*1000));
    ROS_ERROR("%d , %d , %d",deg_roll2, deg_yaw2, deg_pitch2);

    ROS_ERROR("%d , %d , %d",(int)(position_x*1000), (int)(position_y*1000)+1400,(int)(position_z*1000));
    ROS_ERROR("%d , %d , %d",deg_roll, deg_yaw,deg_pitch);

    ROS_ERROR("%d , %d , %d",(int)(position_x1*1000), (int)(position_y1*1000)+1400,(int)(position_z1*1000));
    ROS_ERROR("%d , %d , %d",deg_roll1, deg_yaw1,deg_pitch1);

    // だみとら用


    ros::spinOnce(); // コールバック関数を呼ぶ
    loop_rate.sleep();
  }
  return 0;
}
