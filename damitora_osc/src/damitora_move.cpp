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
// だみとら物理コントローラーが足踏みするノード
//   Dynamixel Workbenchに接続して、joint_trajectoryを送信して、物理コンがひたすら動く
// 

#include "ros/ros.h"
#include "ros/time.h"

#include "trajectory_msgs/JointTrajectory.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <unistd.h> // sleep用


int main(int argc, char **argv)
{

  ros::init(argc, argv, "move_leg_dynamixel"); // ノードの初期化
  ros::NodeHandle nh;                          // ノードハンドラ

  //パブリッシャの作成 (DynamixelWorkBenchへの指示)
  ros::Publisher pub_damitora_trajectory;
  pub_damitora_trajectory = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory", 1);

  ros::Rate loop_rate(100); // 制御周期60Hz

  // ポーズ(JointTrajectory)を生成
  trajectory_msgs::JointTrajectory jtp0;
  jtp0.header.frame_id = "world"; // ポーズ名（モーション名)
  jtp0.joint_names.resize(11);    // 関節名をセット
  jtp0.joint_names[0] = "head_R";
  jtp0.joint_names[1] = "head_P";
  jtp0.joint_names[2] = "head_Y";
  jtp0.joint_names[3] = "R_upper_leg_Y";
  jtp0.joint_names[4] = "R_upper_leg_R";
  jtp0.joint_names[5] = "R_upper_leg_P";
  jtp0.joint_names[6] = "R_lower_leg_P";
  jtp0.joint_names[7] = "L_upper_leg_Y";
  jtp0.joint_names[8] = "L_upper_leg_R";
  jtp0.joint_names[9] = "L_upper_leg_P";
  jtp0.joint_names[10] = "L_lower_leg_P";

  jtp0.points.resize(2);                   // ポーズは2つ
  jtp0.points[0].positions.resize(11);     // ポーズ→positionsを2個設定
  jtp0.points[0].velocities.resize(11);    // ポーズ→velocitiesを2個設定
  jtp0.points[0].accelerations.resize(11); // ポーズ→accelerationsを2個設定
  jtp0.points[0].effort.resize(11);        // ポーズ→effortを2個設定
  jtp0.points[1].positions.resize(11);     // ポーズ→positionsを2個設定
  jtp0.points[1].velocities.resize(11);    // ポーズ→velocitiesを2個設定
  jtp0.points[1].accelerations.resize(11); // ポーズ→accelerationsを2個設定
  jtp0.points[1].effort.resize(11);        // ポーズ→effortを2個設定

  // 原点ポーズをセット
  jtp0.points[0].positions[0] = 0.0;
  jtp0.points[0].positions[1] = 0.0;
  jtp0.points[0].positions[2] = 0.0; // 0.8は取り付けオフセット
  jtp0.points[0].positions[3] = 0.0;
  jtp0.points[0].positions[4] = 0.0;
  jtp0.points[0].positions[5] = 0.0;
  jtp0.points[0].positions[6] = 0.0; // 0.8は取り付けオフセット
  jtp0.points[0].positions[7] = 0.0;
  jtp0.points[0].positions[8] = 0.0;
  jtp0.points[0].positions[9] = 0.0;
  jtp0.points[0].positions[10] = 0.0;                  // 0.8は取り付けオフセット
  jtp0.points[0].time_from_start = ros::Duration(0.0); //実行時間0.0sec

  jtp0.points[1].positions[0] = 0.0;
  jtp0.points[1].positions[1] = 0.0;
  jtp0.points[1].positions[2] = 0.0;
  jtp0.points[1].positions[3] = 0.0;
  jtp0.points[1].positions[4] = 0.0;
  jtp0.points[1].positions[5] = 0.0;
  jtp0.points[1].positions[6] = 0.0;
  jtp0.points[1].positions[7] = 0.0;
  jtp0.points[1].positions[8] = 0.0;
  jtp0.points[1].positions[9] = 0.0;
  jtp0.points[1].positions[10] = 0.0;
  jtp0.points[1].time_from_start = ros::Duration(2.0); //実行時間1.0sec

  ROS_INFO("damitora move : start!");

  while (ros::ok())
  {
    
    jtp0.header.stamp = ros::Time::now();

    // 原点ポーズをセット
    jtp0.points[0].positions[0] = 0.0;
    jtp0.points[0].positions[1] = 0.0;
    jtp0.points[0].positions[2] = 0.0; // 0.8は取り付けオフセット
    jtp0.points[0].positions[3] = 0.0;
    jtp0.points[0].positions[4] = 0.0;
    jtp0.points[0].positions[5] = 0.0;
    jtp0.points[0].positions[6] = 0.0; // 0.8は取り付けオフセット
    jtp0.points[0].positions[7] = 0.0;
    jtp0.points[0].positions[8] = 0.0;
    jtp0.points[0].positions[9] = 0.0;
    jtp0.points[0].positions[10] = 0.0;                  // 0.8は取り付けオフセット
    jtp0.points[0].time_from_start = ros::Duration(0.0); //実行時間0.0sec

    jtp0.points[1].positions[0] = 0.0;
    jtp0.points[1].positions[1] = 0.0;
    jtp0.points[1].positions[2] = 0.0;
    jtp0.points[1].positions[3] = 0.0;
    jtp0.points[1].positions[4] = 0.0;
    jtp0.points[1].positions[5] = 1.0;
    jtp0.points[1].positions[6] = 1.0;
    jtp0.points[1].positions[7] = 0.0;
    jtp0.points[1].positions[8] = 0.0;
    jtp0.points[1].positions[9] = 1.0;
    jtp0.points[1].positions[10] = -1.0;
    jtp0.points[1].time_from_start = ros::Duration(2.0); //実行時間1.0sec

    //パブリッシュ (joint_trajectry)
    pub_damitora_trajectory.publish(jtp0);

    usleep(1000 * 1000);

    // 原点ポーズをセット
    jtp0.points[0].positions[0] = 0.0;
    jtp0.points[0].positions[1] = 0.0;
    jtp0.points[0].positions[2] = 0.0; // 0.8は取り付けオフセット
    jtp0.points[0].positions[3] = 0.0;
    jtp0.points[0].positions[4] = 0.0;
    jtp0.points[0].positions[5] = 0.0;
    jtp0.points[0].positions[6] = 0.0; // 0.8は取り付けオフセット
    jtp0.points[0].positions[7] = 0.0;
    jtp0.points[0].positions[8] = 0.0;
    jtp0.points[0].positions[9] = 0.0;
    jtp0.points[0].positions[10] = 0.0;                  // 0.8は取り付けオフセット
    jtp0.points[0].time_from_start = ros::Duration(0.0); //実行時間0.0sec

    jtp0.points[1].positions[0] = 0.0;
    jtp0.points[1].positions[1] = 0.0;
    jtp0.points[1].positions[2] = 0.0;
    jtp0.points[1].positions[3] = 0.0;
    jtp0.points[1].positions[4] = 0.0;
    jtp0.points[1].positions[5] = -1.0;
    jtp0.points[1].positions[6] = 1.0;
    jtp0.points[1].positions[7] = 0.0;
    jtp0.points[1].positions[8] = 0.0;
    jtp0.points[1].positions[9] = -1.0;
    jtp0.points[1].positions[10] = -1.0;
    jtp0.points[1].time_from_start = ros::Duration(2.0); //実行時間1.0sec

    //パブリッシュ (joint_trajectry)
    pub_damitora_trajectory.publish(jtp0);

    usleep(1000 * 1000);

    ros::spinOnce(); // コールバック関数を呼ぶ

    loop_rate.sleep();
  }
  return 0;
}
