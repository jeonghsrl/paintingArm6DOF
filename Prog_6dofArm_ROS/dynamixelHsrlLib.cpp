//-------- v20220113
//-------- v20220202: 角速度リード関数追加
//-------- v20220203: RecognitionDynamixel()修正、exeTrajectoryPoints関連追加
//-------- v20220202: EPCM()修正

#include <DynamixelWorkbench.h>
#include "dynamixelHsrlLib.h"
#include "setup_para.h"

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

extern DynamixelWorkbench dxl_wb;
extern DYNAMIXEL_JOINT DJ[];
extern uint8_t st;


///////////////////---Dynamixelの認識 
void RecognitionDynamixel(uint8_t dxl_id){                   //2022.01.12 修正
  
  const char *log;                  
  bool result = false;        
  uint16_t model_number = 0; 

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false){ Serial.println(log); Serial.println("Failed to init");  }
  else{Serial.print("Succeeded to init : "); Serial.println(BAUDRATE);  }
      
  //-- Dynamixelの認識 (1個でも認識失敗すると、モータ駆動ができない
  // ping all dynamixel
  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  { Serial.println(log);  Serial.println("Failed to ping"); 
       #ifdef __TONE_ON
        tone(BDPIN_BUZZER, 100, 500);
       #endif
  }
  else {
    #ifdef __TONE_ON
    tone(BDPIN_BUZZER, 700, 150);
    delay(150);
    tone(BDPIN_BUZZER, 300, 150);
    #endif
    Serial.println("Succeeded to ping");
    result = dxl_wb.torqueOff(dxl_id, &log); 
    result = dxl_wb.itemWrite(dxl_id, "Homing_Offset",0, &log);
    result = dxl_wb.torqueOn(dxl_id, &log);   
    Serial.print("id : ");  Serial.print(dxl_id);
    Serial.print(" model_number : ");  Serial.println(model_number);
  }
  delay(100);
  Serial.println(); 
}  


///////////////////---Dynamixelの認識 
void RecognitionJoint(uint8_t jnum){                      //2022.01.13 修正
  
  const char *log;                  
  bool result = false;        
  uint16_t model_number = 0; 
  uint8_t dxl_id = 0;
  int32_t pulse = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false){ Serial.println(log); Serial.println("Failed to init");  }
  else{Serial.print("Succeeded to init : "); Serial.println(BAUDRATE);  }
   
   
  //-- Dynamixelの認識 (1個でも認識失敗すると、モータ駆動ができない
  // ping all dynamixel
  result = dxl_wb.ping(DJ[jnum].id, &model_number, &log);
  if (result == false)
  { Serial.println(log);  Serial.println("Failed to ping"); 
     #ifdef __TONE_ON
      tone(BDPIN_BUZZER, 100, 500);
     #endif
  }
  else {
    #ifdef __TONE_ON
    tone(BDPIN_BUZZER, 700, 150);
    delay(150);
    tone(BDPIN_BUZZER, 300, 150);
    #endif
    Serial.println("Succeeded to ping");
    result = dxl_wb.torqueOff(DJ[jnum].id, &log); 
    result = dxl_wb.itemWrite(DJ[jnum].id, "Homing_Offset",0, &log);
    result = dxl_wb.torqueOn(DJ[jnum].id, &log);       
    Serial.print("Joint : ");  Serial.print(DJ[jnum].jnum);
    Serial.print("ID : ");  Serial.print(DJ[jnum].id);
    Serial.print(" model_number : ");  Serial.println(model_number);
    dxl_wb.getPresentPositionData(DJ[jnum].id, &pulse, &log);
    Serial.print(" Present pulse: ");  Serial.println(pulse);  
    Serial.print(" Present angle: ");  Serial.println(pulse*DJ[jnum].pul2deg);
  }
  delay(100);
  Serial.println(); 
}  
  

//////////////////////////////////////////////////////////////////////////
// 電流閾値で原点設定後、初期位置まで移動、Current Based Position Control Modeに設定
//  int32_t vel_back ;         //リミット位置への移動速度  
//  float originDetect_current; //リミット確認用電流閾値
//  int32_t originPos_pulse;  //検出位置として設定したいpulse
//  int32_t initMove_pulse;        //初期位置までの移動距離pulse
//  float goal_current = 300;      //Goal current(これ以上のcurrentは流れない)
//  int8_t dir = 1 or -1;          //reverse mode On: -1, reverse mode Off: 1     
//  return: int32_t present postion     
///////////////////////////////////////////////////////////////////////////// 
int32_t CBPCM_Set_Init( uint8_t dxl_id, int32_t vel_back, float originDetect_current,int32_t originPos_pulse,int32_t initMove_pulse,float goal_current, int8_t dir){
 
  const char *log;
  bool result = false;
  int32_t get_pulse = 0;
  int32_t get_current = 0;
  float current =0;
  float SF_current=2.69;    //Scale Factor  X430=2.69 //  
  uint32_t delayTime=1000;
  
 
 //////////// Homing_Offsetを0に設定(torque offの状態で)
  result = dxl_wb.torqueOff(dxl_id, &log); 
  result = dxl_wb.itemWrite(dxl_id, "Homing_Offset", 0, &log);

  
  ///////////連続運転のためにwheelmodeを設定 
  result = dxl_wb.wheelMode(dxl_id, 0, &log);
  if (result == false) { Serial.println(log);  Serial.println("Failed to change wheel mode");  }
  else {  Serial.println("Succeed to change wheelMode");  }


  ////////// 初期位置まで定速で移動
    Serial.println("Move to Limit Point...");
    delay(delayTime);
    dxl_wb.goalVelocity(dxl_id, vel_back);

  /////////  端で接触を検出した後（閾値電流以上)、初期位置に移動     
   while(fabs(current) < originDetect_current){  
     result = dxl_wb.itemRead(dxl_id, "Present_Current", &get_current, &log); // read current value
     current = dxl_wb.convertValue2Current(get_current);                      // convert real current 
    // Serial.println(current);
     delay(10);
  }
 
  #ifdef __TONE_ON 
  tone(BDPIN_BUZZER, 700, 150);
 #endif 
 
   Serial.println("Detect Limit");
   //初期位置に移動
   dxl_wb.goalVelocity(dxl_id, -1*vel_back);       
   delay(100);
   //初期位置で停止
   dxl_wb.goalVelocity(dxl_id, (int32_t)0);        
 
 
/////////// Current Extended Position Control Modeの設定
   Serial.println("Chage to CEPC Mode");
   delay(delayTime);
  //torqueをoffにする
  result = dxl_wb.torqueOff(dxl_id, &log);                
  
  //modeをCBPCに設定
  result = dxl_wb.setCurrentBasedPositionControlMode(dxl_id, &log); 
  if (result == false) { Serial.println(log); Serial.println("Failed to change CBPC mode"); }
  else {  Serial.print("Succeed to change CBPC Mode Joint"); Serial.println(dxl_id); }
  //速度制限を50*0.229 rpmに設定
  result = dxl_wb.itemWrite(dxl_id, "Profile_Velocity", 100, &log);
  
  //現在位置を表示
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log);  
  Serial.print("Limit Postion(before Homing): ");  Serial.println(get_pulse);

  // Homing offsetを -(現在位置)+originPos_pulseに設定
  result = dxl_wb.itemWrite(dxl_id, "Homing_Offset", (int32_t)(dir*(-get_pulse+originPos_pulse)), &log);   

  //再度現在位置を表示
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log); 
 //初期位置になっているか確認
  Serial.print("Limit Position(after Homing): ");  Serial.println(get_pulse);                                 

//////////// 確認用移動(運用初期位置までの移動量設定）
  //torque on
  result = dxl_wb.torqueOn(dxl_id, &log);
  //Goal Currentの設定 mA/scalingFactor 2.69はXH430
  dxl_wb.itemWrite(dxl_id, "Goal_Current", goal_current/SF_current, &log);    
  //運用初期位置のゴール位置設定         
    dxl_wb.goalPosition(dxl_id, (int32_t)initMove_pulse);
/*  
  //運用初期位置まで移動
 while(fabs(initMove_pulse-get_pulse) > 5 ){
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log);
  
  }*/
  
  delay(100);
  
  Serial.print("Operating Initial pulse:");
  Serial.println(get_pulse); 
  Serial.print("Operating Initial angle:");
  Serial.println(get_pulse*Pulse2Deg_XM430);

  return (get_pulse);
  
/*
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log);
  result = dxl_wb.itemRead(dxl_id, "Present_Current", &get_current, &log); // read current value
  current = dxl_wb.convertValue2Current(get_current);                      // convert real current 
  Serial.print(current); Serial.print("\t");
  Serial.println(get_pulse);
  */
}


//////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////// 
int32_t CBPCM_Set_Init_2ID( uint8_t dxl_id, uint8_t dxl_id1, int32_t vel_back, float originDetect_current,int32_t originPos_pulse,int32_t initMove_pulse,float goal_current, int8_t dir){
 
  const char *log;
  bool result = false;
  int32_t get_pulse = 0;
  int32_t get_current = 0;
  float current =0;
  float SF_current=2.69;    //Scale Factor  X430=2.69 //  
  uint32_t delayTime=1000;
  
 
 //////////// Homing_Offsetを0に設定(torque offの状態で)
  result = dxl_wb.torqueOff(dxl_id, &log); 
  result = dxl_wb.itemWrite(dxl_id, "Homing_Offset", 0, &log);
  
  ///////////連続運転のためにwheelmodeを設定 
  result = dxl_wb.wheelMode(dxl_id, 0, &log);
  if (result == false) { Serial.println(log);  Serial.println("Failed to change wheel mode");  }
  else {  Serial.println("Succeed to change wheelMode");  }
  result = dxl_wb.wheelMode(dxl_id1, 0, &log);
  if (result == false) { Serial.println(log);  Serial.println("Failed to change wheel mode");  }
  else {  Serial.println("Succeed to change wheelMode");  }

  ////////// 初期位置まで定速で移動
    Serial.println("Move to Limit Point...");
    delay(delayTime);
    dxl_wb.goalVelocity(dxl_id, vel_back);
    dxl_wb.goalVelocity(dxl_id1, vel_back);

  /////////  端で接触を検出した後（閾値電流以上)、初期位置に移動     
   while(fabs(current) < originDetect_current){  
     result = dxl_wb.itemRead(dxl_id, "Present_Current", &get_current, &log); // read current value
     current = dxl_wb.convertValue2Current(get_current);                      // convert real current 
    // Serial.println(current);
     delay(10);
  }
   Serial.println("Detect Limit");
   //初期位置に移動
   //dxl_wb.goalVelocity(dxl_id, -1*vel_back);       
   //delay(100);
   //初期位置で停止
   dxl_wb.goalVelocity(dxl_id, (int32_t)0);   
   dxl_wb.goalVelocity(dxl_id1, (int32_t)0);       
 
 
/////////// Current Extended Position Control Modeの設定
   Serial.println("Chage to CEPC Mode");
   delay(delayTime);
  //torqueをoffにする
  result = dxl_wb.torqueOff(dxl_id, &log);        
  result = dxl_wb.torqueOff(dxl_id1, &log);          
  
  //modeをCBPCに設定
  result = dxl_wb.setCurrentBasedPositionControlMode(dxl_id, &log); 
  if (result == false) { Serial.println(log); Serial.println("Failed to change CBPC mode"); }
  else {  Serial.print("Succeed to change CBPC Mode Joint"); Serial.println(dxl_id); }
  result = dxl_wb.setCurrentBasedPositionControlMode(dxl_id1, &log); 
  if (result == false) { Serial.println(log); Serial.println("Failed to change CBPC mode"); }
  else {  Serial.print("Succeed to change CBPC Mode Joint"); Serial.println(dxl_id1); }
  //速度制限を50*0.229 rpmに設定
  result = dxl_wb.itemWrite(dxl_id, "Profile_Velocity", 100, &log);
  result = dxl_wb.itemWrite(dxl_id1, "Profile_Velocity", 100, &log);
  
  //現在位置を表示
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log);  
  Serial.print("Limit Postion(before Homing): ");  Serial.println(get_pulse);
  // Homing offsetを -(現在位置)+originPos_pulseに設定
  result = dxl_wb.itemWrite(dxl_id, "Homing_Offset", (int32_t)(dir*(-get_pulse+originPos_pulse)), &log);   
  //再度現在位置を表示
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log); 
 //初期位置になっているか確認
  Serial.print("Limit Position(after Homing): ");  Serial.println(get_pulse);                                 

 //現在位置を表示
  result = dxl_wb.itemRead(dxl_id1, "Present_Position", &get_pulse, &log);  
  Serial.print("Limit Postion(before Homing): ");  Serial.println(get_pulse);
  // Homing offsetを -(現在位置)+originPos_pulseに設定
  result = dxl_wb.itemWrite(dxl_id1, "Homing_Offset", (int32_t)(dir*(-get_pulse+originPos_pulse)), &log);   
  //再度現在位置を表示
  result = dxl_wb.itemRead(dxl_id1, "Present_Position", &get_pulse, &log); 
 //初期位置になっているか確認
  Serial.print("Limit Position(after Homing): ");  Serial.println(get_pulse);             
  

//////////// 確認用移動(運用初期位置までの移動量設定）
  //torque on
  result = dxl_wb.torqueOn(dxl_id, &log);
  result = dxl_wb.torqueOn(dxl_id1, &log);
  //Goal Currentの設定 mA/scalingFactor 2.69はXH430
  dxl_wb.itemWrite(dxl_id, "Goal_Current", goal_current/SF_current, &log);    
  dxl_wb.itemWrite(dxl_id1, "Goal_Current", goal_current/SF_current, &log);    
  //運用初期位置のゴール位置設定         
    dxl_wb.goalPosition(dxl_id, (int32_t)initMove_pulse);
//    dxl_wb.goalPosition(dxl_id1, (int32_t)initMove_pulse);
/*  
  //運用初期位置まで移動
 while(fabs(initMove_pulse-get_pulse) > 5 ){
  result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_pulse, &log);
  
  }*/
  
  delay(100);
  
  Serial.print("Operating Initial pulse:");
  Serial.println(get_pulse); 
  Serial.print("Operating Initial angle:");
  Serial.println(get_pulse*Pulse2Deg_XM430);

  return (get_pulse);

}

///////////////////////////////////////////////////////////
// Dynの角度を補正し、関節角度でのExtended Position Control Modeに設定
//  uint8_t jnum                     //Joint番号
//  float dynAngAtJointHomeAng       //関節Homeポジション角度でのdynamixel角度  
//  float jointHomeAng               //関節Homeポジション角度
//  int32_t delayTime                //関節運動時間 
///////////////////////////////////////////////////////////
int32_t EPCM_Set_Init(uint8_t jnum, float dynAngAtJointHomeAng, float jointHomeAng, int32_t delayTime, int8_t dir){ 

  const char *log;
  bool result = false;
  int32_t get_pulse = 0;

  Serial.println("Try to change EPC mode");
    
 //modeをECPMに設定
  result = dxl_wb.torqueOff(DJ[jnum].id, &log);                
  result = dxl_wb.setExtendedPositionControlMode(DJ[jnum].id, &log); 
  if (result == false) { Serial.println(log); Serial.println("Failed to change EPC mode"); }
  else {  Serial.print("Succeed to change EPC Mode"); Serial.println(DJ[jnum].id); }
   
 //---速度・加速度プロファイル設定
 dxl_wb.itemWrite(DJ[jnum].id, "Profile_Velocity",2000);  //リフトの速度制限を200*0.229 rpmに設定
 dxl_wb.itemWrite(DJ[jnum].id, "Profile_Acceleration",50);   
 result = dxl_wb.torqueOn(DJ[jnum].id, &log); 
 
 //---関節座標系での初期姿勢角度まで移動 
 DJ[jnum].pulse.g =  dynAngAtJointHomeAng/DJ[jnum].pul2deg;   //90度
 dxl_wb.goalPosition(DJ[jnum].id, (int32_t)DJ[jnum].pulse.g);  
 delay(delayTime);

 result = dxl_wb.itemRead(DJ[jnum].id, "Present_Position", &get_pulse, &log); 
// Serial.println(get_pulse); 
 result = dxl_wb.torqueOff(DJ[jnum].id, &log); 
 result = dxl_wb.itemWrite(DJ[jnum].id, "Homing_Offset",-1*dir*get_pulse+jointHomeAng/DJ[jnum].pul2deg, &log); //関節座標系での角度を設定
// result = dxl_wb.itemRead(DJ[J1].id, "Homing_Offset", &get_pulse, &log);  
// Serial.println(get_pulse); 
 result = dxl_wb.torqueOn(DJ[jnum].id, &log);   
}

/////////////////////////////////////////////////////////////


 /////////////////////-- Go to initial pose      
void goInitJointConfig(uint8_t jnum, float ang_g, float ang_thres ){

    DJ[jnum].ang.g=ang_g;
    setGoalAngle2Pulse(DJ[jnum].jnum);   //目標DXLパルス計算   
    dxl_wb.goalPosition(DJ[jnum].id, (int32_t)DJ[jnum].pulse.g);

    while(fabs(DJ[jnum].ang.g-DJ[jnum].ang.n) > ang_thres){
      calPulse2Angle(DJ[jnum].jnum);     //DXLパルスから現在角度取得      
    }
    #ifdef __TONE_ON 
    tone(BDPIN_BUZZER, 700, 150);
   #endif 
}


int32_t getPresentPulse(uint8_t jnum){
    const char *log;   
    bool result = false;
    int32_t get_pulse = 0;
    result = dxl_wb.itemRead(DJ[jnum].id, "Present_Position", &get_pulse, &log);
    return get_pulse;
}

int32_t getPresentVelPulse(uint8_t jnum){
    const char *log;   
    bool result = false;
    int32_t get_velpulse = 0;
    
    result = dxl_wb.itemRead(DJ[jnum].id, "Present_Velocity", &get_velpulse, &log); // read velocity value
    return get_velpulse;
}

////////////getPresentCurrent ///////////////////////////
float getPresentCurrent(uint8_t jnum){
   const char *log;   
   bool result = false;
   int32_t get_current = 0;
   result = dxl_wb.itemRead(DJ[jnum].id, "Present_Current", &get_current, &log); // read current value
   float currentVal = dxl_wb.convertValue2Current(get_current);                      // convert real curre

   return currentVal;
}


/////////////////////Read pulse from DXL and convert to angle[deg]/////////////////////
void calPulse2Angle(uint8_t jnum){
   //回転関節は、deg2distが1。直動関節のang.nは、距離[mm]  
      DJ[jnum].pulse.n = getPresentPulse(DJ[jnum].jnum);
      DJ[jnum].dxlAng.n = DJ[jnum].dir_rot*(DJ[jnum].pulse.n*DJ[jnum].pul2deg);
      DJ[jnum].ang.n = DJ[jnum].dir_rot*(DJ[jnum].pulse.n-DJ[jnum].pulse.offset)*DJ[jnum].pul2deg*DJ[jnum].gearRatio*DJ[jnum].deg2dist;   

      DJ[jnum].velpulse.n = getPresentVelPulse(DJ[jnum].jnum);
      DJ[jnum].dang.n = dxl_wb.convertValue2Velocity(DJ[jnum].id,DJ[jnum].velpulse.n)*RADtoDEG;     // convert to rad/s
}

////////////Set goal angle[deg] to DXL pulse///////////////////////
void setGoalAngle2Pulse(uint8_t jnum){

     DJ[jnum].pulse.g=DJ[jnum].dir_rot*DJ[jnum].ang.g/(DJ[jnum].pul2deg*DJ[jnum].gearRatio*DJ[jnum].deg2dist)+DJ[jnum].pulse.offset;    
 
}

/*

//////////////////////Calculation DXL goal pulse from goal distance (mm) for linear actuation/////////////
void setGoalDist2Pulse(uint8_t jnum){
     
    DJ[jnum].ang.g = DJ[jnum].x.g * DJ[jnum].dist2deg;
    DJ[jnum].pulse.g=((DJ[jnum].dir_rot*DJ[jnum].ang.g+DJ[jnum].ang.offset)*DJ[jnum].deg2pul);
      
}
*/
/////////////moveJoint/////////////////////////////2021.0827
uint8_t moveJoint(uint8_t jnum, float ang_g, float ang_thres ){
    
    DJ[jnum].ang.g=ang_g;
    setGoalAngle2Pulse(DJ[jnum].jnum);   //目標DXLパルス計算   
    //dxl_wb.goalPosition(DJ[jnum].id, (int32_t)DJ[jnum].pulse.g); 

   if(fabs(DJ[jnum].ang.g-DJ[jnum].ang.n) > ang_thres) return 0;
   else{
    #ifdef __TONE_ON 
     tone(BDPIN_BUZZER, 700, 150);
    #endif  
    return 1;
   }
}


////////////配列のtrajectorypointsを目標角度に順次セット//////////2022.0203
uint8_t exeTrajectoryPoints(uint8_t jnum){
     if(DJ[jnum].trIdx < DJ[jnum].trIdxMax){ 
        DJ[jnum].ang.g = DJ[jnum].trajPoints[DJ[jnum].trIdx++]; 
        DJ[J1].trFlag=0; return 0;}
     else{ DJ[jnum].trIdx=0;DJ[jnum].trFlag=1; //DJ[jnum].trIdxMax;  
           return 1;
           }
     }

/////////rtimeDelay///////////////////////2021.0827
uint8_t rtimeDelay(uint32_t delayTime){
  static int32_t passedTime=0;
  passedTime += (uint32_t)(TIMER_RATE/1000);
  if(delayTime > passedTime) return 0;
  else{ passedTime=0;  return 1;   } 
}


////////readCommandFromSerial ///////////////////2021.0827
uint8_t readCommandFromSerial(){
  uint8_t result; 
 
  if(Serial.available() > 0){
    String cmds[20] = {"\0"};                        // String Command Buffer
    String strCmd = Serial.readStringUntil('\n');   // Read string from serial until '\n' 
    int index = split(strCmd,',',cmds);              // Split command  
   result = (uint8_t)cmds[0].toFloat();             
  }  
 return result;
}// readCommand


////////readCommandFromSerial2///////////////////2021.0827
uint8_t readCommandFromSerial2(){

  uint8_t result;
  if(Serial2.available() > 0){
    String cmds[20] = {"\0"};                        // String Command Buffer
    String strCmd = Serial2.readStringUntil('\n');   // Read string from serial until '\n' 
    int index = split(strCmd,',',cmds);              // Split command  
    result = (uint8_t)cmds[0].toFloat();             
  } 
  return result;
}
  


////////////Copy and Paste //////////////////////////文字列を分割
int split(String data, char delimiter, String *dst){
    int index = 0;
    int arraySize = (sizeof(data)/sizeof((data)[0]));  
    int datalength = data.length();
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter ) {
            index++;
            if ( index > (arraySize - 1)) return -1;
        }
        else dst[index] += tmp;
    }
    return (index + 1);
}
///////////////////////////////////////////

   

//------------- Kinematics_2link
void Kinematics_2link(double th0, double th1, double ph_n[]){

  double ph[]={0,0};
  double DEGtoRAD= 3.14/180.; 

  double l0=0.20;               // length of link 0              
  double l1=0.18;               // length of link 1
  double ph_x;                  //  x position of tip 
  double ph_y;                  //  y position of tip

  double RadTh0 = th0*DEGtoRAD;
  double RadTh1 = th1*DEGtoRAD;

  ph_x=l0*cos(RadTh0)+l1*cos(RadTh0+RadTh1);    // contact point of a hip to a sit surface
  ph_y=l0*sin(RadTh0)+l1*sin(RadTh0+RadTh1);    

  ph_n[0] = ph_x;
  ph_n[1] = ph_y;
      
}


//------ inverse kinematics of a chair robot  
void invKinematics_2link(double xh, double yh, double ang_goal[]){
    
  double l0=0.20;               // length of link 0 
  double l1=0.18;               // length of link 1
    
  double s1,c1;
  double s0,c0;
  double thTemp[2];
  double ks,kc;
       
  double th0, th1;             //rad
  
  double r0 = sqrt(pow(xh,2)+pow(yh,2));
  double dl01 = 2*l0*l1;
 
  s1= sqrt( pow(dl01,2) - pow((pow(r0,2)-(pow(l0,2)+pow(l1,2))),2))/(dl01);
  c1= (pow(r0,2)-(pow(l0,2)+pow(l1,2)))/dl01;
  
  thTemp[1]= atan2(s1,c1);
   
  ks = l1*sin(thTemp[1]);
  kc = l0+l1*cos(thTemp[1]);
  
  s0= (-ks*xh+kc*yh)/(pow(kc,2)+pow(ks,2));
  c0= (kc*xh+ks*yh)/(pow(kc,2)+pow(ks,2));
  
  thTemp[0] = atan2(s0,c0);
  
  th0 = thTemp[0]*RADtoDEG;                // desired joint angle
  th1 = thTemp[1]*RADtoDEG;          

  ang_goal[1]=th0;
  ang_goal[2]=th1;
  
}


//---Joystick
double joyControlVel(int JoyMax, int JoyMin, int PlusMin, int MinusMin, double JoyToVel, int ch){

  double joyVal;            // voltage from Joystick
  double vf;              //戻り値

  joyVal = analogRead(ch);                                                 //joystick値の読み込み

  if(joyVal >= JoyMax)  joyVal = JoyMax;        //上限値設定
  if(joyVal <= JoyMin)  joyVal = JoyMin;        //下限値設定

  if(MinusMin <= joyVal && joyVal < PlusMin)  vf = 0;           //　不感帯設定
  else if(joyVal <= MinusMin) vf = (joyVal - MinusMin)/JoyToVel;    //マイナス方向速度計算
  else if(joyVal >= PlusMin)  vf = (joyVal - PlusMin)/JoyToVel;     //プラス方向速度計算

  return vf;              //速度を戻す
}

/*
//Dynamixelの初期化
void initializingDynamixel(){
  
 DJ[LF].pulse.g=CBPCM_Set_Init_2ID(DJ[LF].id,DJ[LB].id,CVel*4,LDCtLt,LPLt,SPLt,IGCtLt,1);   //CurrentBasedPositionControlModeの初期セットアップ
 DJ[FR].pulse.g=CBPCM_Set_Init(DJ[FR].id,-CVel,LDCtL,LPL,SPL,IGCtL,-1);     //
 DJ[FL].pulse.g=CBPCM_Set_Init(DJ[FL].id,-CVel,LDCtL,LPL,SPL,IGCtL,-1);     // 
 DJ[MF].pulse.g=CBPCM_Set_Init(DJ[MF].id,-CVel,LDCtL,LPL,SPL,IGCtL,-1);     //
 DJ[MB].pulse.g=CBPCM_Set_Init(DJ[MB].id,-CVel,LDCtL,LPL,SPL,IGCtL,-1);     //

 //リフトの速度制限を200*0.229 rpmに設定
  dxl_wb.itemWrite(DJ[LF].id, "Profile_Velocity", 100);
  dxl_wb.itemWrite(DJ[LB].id, "Profile_Velocity", 100);

}
*/
