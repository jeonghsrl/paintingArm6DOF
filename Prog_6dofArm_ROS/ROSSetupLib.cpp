#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <math.h>
#include "array_trajectry_msgs/ArrayTrajectry.h"
using array_trajectry_msgs::ArrayTrajectry; 
#include "setup_para.h"

extern uint8_t modeNum;
int srv_flag;
int req_trajectry_size;
int req_trajectry_points_size;

void RosInit();
void joint_states_pub();

//#include "ROSSetup.h"
#include "dynamixelHsrlLib.h"

//各関節軌道データに振り分けて格納するfloat型動的配列を定義
std::vector<float> joint1_traj;
std::vector<float> joint2_traj;
std::vector<float> joint3_traj;
std::vector<float> joint4_traj;
std::vector<float> joint5_traj;
std::vector<float> joint6_traj;

void srv_callback(const ArrayTrajectry::Request & req, ArrayTrajectry::Response & res);


#define Joint_Sum   6 //アームの関節数の定義
extern DYNAMIXEL_JOINT DJ[Joint_Sum+1];
/**************グローバル****************/

ros::NodeHandle nh; //rosのノードハンドルをhnとして使用
std_msgs::Float32MultiArray arm_joint_states; //受け取ったjoint

/*************************************/
//パブリッシャーの設定（トピック名，コールバック関数）
//このパブリッシャーは常にアームの関節状態を配信
ros::Publisher chatter("arm_control/arm_joint_states", &arm_joint_states);

//サービスサーバの設定（サーバ名，コールバック関数）
ros::ServiceServer<ArrayTrajectry::Request, ArrayTrajectry::Response> server("arm_control/joint_trajectry", &srv_callback);


/*****************ROS定義用関数*******************/
void RosInit() {
  nh.initNode();//初期化
  nh.advertiseService(server); //サービスサーバ有効化
  nh.advertise(chatter); //パブリッシャー有効化
  nh.getHardware()->setBaud(9600); //ros serial baudrate setup
}

/**サービスコールバック関数**/
void srv_callback(const ArrayTrajectry::Request & req, ArrayTrajectry::Response & res) {
  /**軌道データを取得**/
  tone(BDPIN_BUZZER, 100, 500);
    //delay(1000);
  req_trajectry_size = req.array_trajectry.data_length;
  req_trajectry_points_size = req.array_trajectry.data_length / Joint_Sum;

   //臨時軌道格納配列
  float trpointsJ1[req_trajectry_points_size]={0,};  
  float trpointsJ2[req_trajectry_points_size]={0,};   
  float trpointsJ3[req_trajectry_points_size]={0,};  
  float trpointsJ4[req_trajectry_points_size]={0,};   
  float trpointsJ5[req_trajectry_points_size]={0,};   
  float trpointsJ6[req_trajectry_points_size]={0,};   
   
//ここでリクエストした配列を6関節ごとに分けた軌道データ配列にする
  for (int i = 0; i < req_trajectry_points_size; i++) {
    trpointsJ1[i] = (float)req.array_trajectry.data[0 + i * Joint_Sum];
    trpointsJ2[i] = (float)req.array_trajectry.data[1 + i * Joint_Sum];
    trpointsJ3[i] = (float)req.array_trajectry.data[2 + i * Joint_Sum];
    trpointsJ4[i] = (float)req.array_trajectry.data[3 + i * Joint_Sum];
    trpointsJ5[i] = (float)req.array_trajectry.data[4 + i * Joint_Sum];
    trpointsJ6[i] = (float)req.array_trajectry.data[5 + i * Joint_Sum];
  }

  uint8_t spNum = 4;//補間数   (uint)(waypoints周期時間(ms)/(TIMER_RATE/1000))を入れること
  uint16_t tSize = req_trajectry_points_size;
  float new_trpointsJ1[(tSize-1)*spNum+1]={0,};   //新しい軌道配列
  float new_trpointsJ2[(tSize-1)*spNum+1]={0,};  
  float new_trpointsJ3[(tSize-1)*spNum+1]={0,};   
  float new_trpointsJ4[(tSize-1)*spNum+1]={0,};  
  float new_trpointsJ5[(tSize-1)*spNum+1]={0,};  
  float new_trpointsJ6[(tSize-1)*spNum+1]={0,};   
  float trajStep[Jnum]={0,};                      //補間軌道のステップ値

  for(uint16_t i=0; i< tSize-1; i++){ 
    //(二つのwaypoints間の差分を補間数で割って補間ステップ値を計算）
    trajStep[0] = (trpointsJ1[i+1] - trpointsJ1[i])/(float)spNum;  
    trajStep[1] = (trpointsJ2[i+1] - trpointsJ2[i])/(float)spNum; 
    trajStep[2] = (trpointsJ3[i+1] - trpointsJ3[i])/(float)spNum; 
    trajStep[3] = (trpointsJ4[i+1] - trpointsJ4[i])/(float)spNum; 
    trajStep[4] = (trpointsJ5[i+1] - trpointsJ5[i])/(float)spNum; 
    trajStep[5] = (trpointsJ6[i+1] - trpointsJ6[i])/(float)spNum; 
                        
    for(uint16_t j=0; j <= spNum; j++){
       //補間した軌道を格納
       new_trpointsJ1[i*spNum+j]=trpointsJ1[i]+trajStep[0]*j;
       new_trpointsJ2[i*spNum+j]=trpointsJ2[i]+trajStep[1]*j;
       new_trpointsJ3[i*spNum+j]=trpointsJ3[i]+trajStep[2]*j;
       new_trpointsJ4[i*spNum+j]=trpointsJ4[i]+trajStep[3]*j;
       new_trpointsJ5[i*spNum+j]=trpointsJ5[i]+trajStep[4]*j;
       new_trpointsJ6[i*spNum+j]=trpointsJ6[i]+trajStep[5]*j;               
    }
  }

 for(uint16_t i=0; i< (tSize-1)*spNum+1; i++){
      //各関節の目標軌道配列に格納
        DJ[J1].trajPoints[i]=new_trpointsJ1[i]; 
        DJ[J2].trajPoints[i]=new_trpointsJ2[i]; 
        DJ[J3].trajPoints[i]=new_trpointsJ3[i]; 
        DJ[J4].trajPoints[i]=new_trpointsJ4[i]; 
        DJ[J5].trajPoints[i]=new_trpointsJ5[i]; 
        DJ[J6].trajPoints[i]=new_trpointsJ6[i]; 
  }

   srv_flag = 1;
   modeNum = 2;
  /*　joint◯_trajの配列から取り出し方法　*/
  
  arm_joint_states.data_length = Joint_Sum; //配信する入れる
  
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++) DJ[i].trIdxMax = (uint16_t)req_trajectry_points_size;
  /*
  for (int j = 0; j < req_trajectry_points_size+1; j++) {
    arm_joint_states.data[0] = DJ[J1].trajPoints[j];
    arm_joint_states.data[1] = DJ[J2].trajPoints[j];
    arm_joint_states.data[2] = DJ[J3].trajPoints[j];
    arm_joint_states.data[3] = DJ[J4].trajPoints[j];
    arm_joint_states.data[4] = DJ[J5].trajPoints[j];
    arm_joint_states.data[5] = DJ[J6].trajPoints[j];
    */
    /****************** Debag用パブリッシュ ********************/
    //topicで確認する用
    //chatter.publish(&arm_joint_states);
    //nh.spinOnce();
    //delay(500);

  //}

  /************クライアントに結果を送信*************/
  res.result = true;
}

void update_joint_states() {
  arm_joint_states.data_length = 6;
  /*
  arm_joint_states.data[0] = float(int(DJ[J1].ang.n * (3.14/180) * 100))/100;
  arm_joint_states.data[1] = float(int(DJ[J2].ang.n * (3.14/180) * 100))/100;
  arm_joint_states.data[2] = float(int(DJ[J3].ang.n * (3.14/180) * 100))/100;
  arm_joint_states.data[3] = float(int(DJ[J4].ang.n * (3.14/180) * 100))/100;
  arm_joint_states.data[4] = float(int(DJ[J5].ang.n * (3.14/180) * 100))/100;
  arm_joint_states.data[5] = float(int(DJ[J6].ang.n * (3.14/180) * 100))/100;
  */
  arm_joint_states.data[0] = (float)DJ[J1].ang.n * (3.14/180);
  arm_joint_states.data[1] = (float)DJ[J2].ang.n * (3.14/180);
  arm_joint_states.data[2] = (float)DJ[J3].ang.n * (3.14/180);
  arm_joint_states.data[3] = (float)DJ[J4].ang.n * (3.14/180);
  arm_joint_states.data[4] = (float)DJ[J5].ang.n * (3.14/180);
  arm_joint_states.data[5] = (float)DJ[J6].ang.n * (3.14/180);
}
/*******************関節状態を常にTopic配信する*******************/
void joint_states_pub() {
  update_joint_states();
  chatter.publish(&arm_joint_states);
  nh.spinOnce();
  delay(20);
}
