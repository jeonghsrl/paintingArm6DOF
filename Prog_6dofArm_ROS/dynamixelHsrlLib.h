//-------- v20220113:
//-------- v20220202: 角速度リード関数追加
//-------- v20220203: RecognitionDynamixel()修正、exeTrajectoryPoints関連追加
//-------- v20220202: EPCM()修正

#ifndef __DYNAMIXELHSRLLIB_H
#define __DYNAMIXELHSRLLIB_H

#define __TONE_ON

//Convert Pulse to Degree
#define Pulse2Deg_XM270 0.0879 
#define Pulse2Deg_XM430 0.0879 
#define Pulse2Deg_XH540 0.0879  
#define Pulse2Deg_XM540 0.0879     //2022.01.22 
#define Pulse2Deg_MX64  0.0879  
#define Pulse2Deg_MX28  0.0879 
#define Pulse2Deg_XL430 0.0879 
#define Pulse2Deg_AX12  0.2930 

#define TIMER_RATE 40000
#define InitDelayTime 1500
#define BAUDRATE  1000000
#define RADtoDEG  57.32
//#define DEGtoRAD  0.0174
#define MAX_TRAJECTORY_NUM 4000 /////////////////Trajectry Point


/* state valuable struct */
typedef struct __state{
  double statetime;           //time

  double offset;               //オフセット値
  double lim_max,lim_min;      //limitValue value
  double ori;
  double init;
  double i,i_buf;              //initial value
  double n,n_buf;              //now value
  double d,d_buf;              //desired value       
  
  double s,s_buf;              // start value
  double f,f_buf;              // final value
  double g,g_buf;              // goal value
  
}STATE;


/* Dynamixel Joint Struct */
typedef struct __dynaJoint_type{

  uint8_t jnum;
  uint8_t id;           
  int8_t  dir_rot;


  STATE pulse,velpulse;                        // pulse 
  STATE dxlAng;                       // dynamixel angle
  STATE current;                      // current
  double dynPul2Deg, dynDeg2Pul;      // dynamixel pulse angle 変換係数 
   
  STATE th,dth,ddth;                  // system coord. angle [rad]        
  STATE ang,dang,ddang;               // system coord. angle [deg] 
  STATE x,dx,ddx;                     // system coord. dist  [mm]                                                    

  
  double pul2deg, deg2pul;           // system pulse-角度変換係数
  double gearRatio;                  // dxl to system 状態変換係数（ギア比など）    
  double deg2dist, dist2deg;         // system 角度-距離変換係数    
  double pul2deg_coff;               // pulse to deg 変換係数 
  double deg2pul_coff;               // deg to pulse 変換係数
  uint8_t revmode;                   // reverse mode 状態変数

  char   trFlag;                       // trajectory flag
  uint16_t trIdx,trIdxMax;             //trajectory index, maxIndex
  float trajPoints[MAX_TRAJECTORY_NUM];   //trajectoryPoints buffer
  
  //pid 
  double kp_j, kv_j, ki_j;           /* PID gain for joint control */ 
   
}DYNAMIXEL_JOINT;


//パラメータの初期化
void InitParameter();

//dynamixel認識
void RecognitionDynamixel(uint8_t dxl_id);    //2022.01.12 修正
void RecognitionJoint(uint8_t jnum);

//dynamixel初期化
void InitDynaMixelJoint();
int32_t CBPCM_Set_Init( uint8_t jnum, int32_t vel_back, float originDetect_current,int32_t originPos_pulse,int32_t initMove_pulse,float goal_current, int8_t dir);
int32_t CBPCM_Set_Init_2ID( uint8_t dxl_id, uint8_t dxl_id1, int32_t vel_back, float originDetect_current,int32_t originPos_pulse,int32_t initMove_pulse,float goal_current, int8_t dir);   
int32_t EPCM_Set_Init(uint8_t jnum, float dynAngAtJointHomeAng, float jointHomeAng, int32_t delayTime, int8_t dir);

//データ処理
void setGoalAngle2Pulse(uint8_t jnum);
//void setGoalDist2Pulse(uint8_t jnum);
int32_t getPresentPulse(uint8_t jnum);      
int32_t getPresentVelPulse(uint8_t jnum);    // add 20220202
float getPresentCurrent(uint8_t jnum); 
void calPulse2Angle(uint8_t jnum);           // modi 20220202      

//動作
void goInitJointConfig(uint8_t jnum, float ang_g, float ang_thres);
uint8_t moveJoint(uint8_t jnum, float ang_g, float ang_thres );   //関節を目標角度まで駆動  
uint8_t exeTrajectoryPoints(uint8_t jnum); //配列のtrajectorypointsを目標角度に順次セット add20220203

//Joystic入力
double joyControlVel(int JoyMax, int JoyMin, int PlusMin, int MinusMin, double JoyToVel, int ch);
void RC100_setup();         //Joystick  
void RC100_update();        

//シリアルコマンド入力
uint8_t readCommandFromSerial();           //uart serialからのデータリード  
uint8_t readCommandFromSerial2();          //uart serial2からのデータリード 
int split(String data, char delimiter, String *dst);


//距離センサ関連
void VL53L0X_setup();       //距離センサ
int16_t distance_vl5310x(uint8_t ch);
int16_t distance_vl5310x(uint8_t ch, int16_t distMin, int16_t distMax);

//システム関数
uint8_t rtimeDelay(uint32_t delayTime);    //realtime制御時に使うdelay  ms

#endif
