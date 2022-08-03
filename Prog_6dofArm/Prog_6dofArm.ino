/*******************************************************************************
errorlog:XM430-RとほかのTTLタイプが同時には動かない。TTLタイプに統一?
*******************************************************************************/

#include <DynamixelWorkbench.h>
#include "setup_para.h"
#include "dynamixelHsrlLib.h"
#include "robotControl.h"


#define __SERIAL_WAIT_ON               //実動作試験ではコメントアウト
#define __Dynamixel_ON
//#define __Auto_ON                     //３つのタスクを連続実行
//#define __Debug_ON
//#define __Torque_OFF

DynamixelWorkbench dxl_wb;
HardwareTimer Timer(TIMER_CH1); //Periodic timerloop
DYNAMIXEL_JOINT DJ[Jnum+1];

Robot rb;                       //Robot 構造体
unsigned long time_passed;
unsigned long time_old;
uint8_t servoOutputFlag = 0;

typedef enum { st_OfflineSinTrajectory=2,
               st_Error=99,
               st_Debug=100               
}status_t;

uint8_t modeNum = st_OfflineSinTrajectory;

//-------------------SET UP------------------
void setup() 
{
  const char *log;
  bool result = false;
  
  Serial.begin(115200);
  Serial2.begin(115200); 
  #ifdef __SERIAL_WAIT_ON
   while(!Serial); // Wait for Opening Serial Monitor
  #endif 
  
//  RC100_setup();         //Joystick setup 
//  delay(1000);

 //---パラメータ初期化
  SetupParameter();
  
 //---Dynamixelの認識
  RecognitionJoint(DJ[J1].jnum); 
  RecognitionJoint(DJ[J2].jnum); 
  RecognitionDynamixel(ID_J20);               //J2のdual駆動で、直接Dynamixel IDを設定 
  RecognitionJoint(DJ[J3].jnum); 
  RecognitionJoint(DJ[J4].jnum); 
  RecognitionJoint(DJ[J5].jnum); 
  RecognitionJoint(DJ[J6].jnum); 

  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){  
    dxl_wb.itemWrite(DJ[i].id, "Profile_Velocity",30);  //リフトの速度制限を200*0.229 rpmに設定
    dxl_wb.itemWrite(DJ[i].id, "Profile_Acceleration",0);  
    dxl_wb.itemWrite(DJ[i].id, "Position_P_Gain",1500); 
    dxl_wb.itemWrite(DJ[i].id, "Position_I_Gain",10); 
 }
 

 //---Dynamixelの初期化及びモード設定
 EPCM_Set_Init(DJ[J4].jnum, 180,0,1000,1);  //dyn180degを関節0degに。
 EPCM_Set_Init(DJ[J5].jnum, 180,0,1000,-1);  //dyn180degを関節0degに。180
 EPCM_Set_Init(DJ[J6].jnum, 180,0,1000,1);  //dyn180degを関節0degに。
 EPCM_Set_Init(DJ[J3].jnum, 162,0,2000,1);  //dyn125degを関節30degに。 162
 EPCM_Set_Init(DJ[J2].jnum, 245,90,2000,1);  //dyn105degを関節150degに。245
 EPCM_Set_Init(DJ[J1].jnum, 180,0,1000,1);  //dyn315degを関節0degに。



  //--現在角度の表示
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){
    calPulse2Angle(DJ[i].jnum);     //DXLパルスから現在角度取得 
  }
  
  Serial.println("----------------------------------------------------------------");
  Serial.print("Dxl Pulse: ");
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ Serial.print(DJ[i].pulse.n);    Serial.print('\t');   }
  Serial.println();  
  Serial.print("Dxl angle: ");
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ Serial.print(DJ[i].dxlAng.n);    Serial.print('\t');   }
  Serial.println();  
  Serial.print("Jnt angle: ");
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ Serial.print(DJ[i].ang.n);    Serial.print('\t');   }
  Serial.println();    
  Serial.println("----------------------------------------------------------------");
  Serial.println("Mechanical Origin Position Setup Finished ");
  delay(1000);
  Serial.println("Go to Control Orign Position ");
  delay(1000);

  //作業初期ポーズへ  (任意)
  DJ[J1].ang.g=0;  DJ[J2].ang.g=90;   DJ[J3].ang.g=0;
  DJ[J4].ang.g=0; DJ[J5].ang.g=0;   DJ[J6].ang.g=0; 
  
  goInitJointConfig(DJ[J1].jnum,DJ[J1].ang.g,1);  //関節を目標角度まで駆動  
  goInitJointConfig(DJ[J3].jnum,DJ[J3].ang.g,1);  //関節を目標角度まで駆動  
  goInitJointConfig(DJ[J2].jnum,DJ[J2].ang.g,1);  //関節を目標角度まで駆動
  goInitJointConfig(DJ[J4].jnum,DJ[J4].ang.g,3);  //関節を目標角度まで駆動
  goInitJointConfig(DJ[J5].jnum,DJ[J5].ang.g,5);  //関節を目標角度まで駆動
  goInitJointConfig(DJ[J6].jnum,DJ[J6].ang.g,3);  //関節を目標角度まで駆動

 //--現在角度の表示
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){
    calPulse2Angle(DJ[i].jnum);     //DXLパルスから現在角度取得 
  }
  Serial.println("----------------------------------------------------------------");
  Serial.print("Dxl Pulse: ");
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ Serial.print(DJ[i].pulse.n);    Serial.print('\t');   }
  Serial.println();  
  Serial.print("Dxl angle: ");
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ Serial.print(DJ[i].dxlAng.n);    Serial.print('\t');   }
  Serial.println();  
  Serial.print("Jnt angle: ");
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ Serial.print(DJ[i].ang.n);    Serial.print('\t');   }
  Serial.println();    
  Serial.println("----------------------------------------------------------------");

 // testJointMotion();
 
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){  
    dxl_wb.itemWrite(DJ[i].id, "Profile_Velocity",100);  //リフトの速度制限を200*0.229 rpmに設定
    dxl_wb.itemWrite(DJ[i].id, "Profile_Acceleration",10);  
    dxl_wb.itemWrite(DJ[i].id, "Position_P_Gain",1000); 
    dxl_wb.itemWrite(DJ[i].id, "Position_I_Gain",10); 
 }

 
 testMakeTrajectoryPoints();
 
/*
//=======Stanby(初期化ボタンを待つ)
//
Serial.print("wait to start. Press Left button of m5stack");
while(1){
 if(Serial2.available() > 0){
    String cmds[20] = {"\0"};                        // String Command Buffer
    String strCmd = Serial2.readStringUntil('\n');   // Read string from serial until '\n' 
    int index = split(strCmd,',',cmds);              // Split command    
    uint8_t result = (uint8_t)cmds[0].toFloat();
    if(result==5) { Serial.println("start05"); break;}  
 }
 #ifdef __TONE_ON 
   tone(BDPIN_BUZZER, 700, 150);
 #endif 
 Serial2.println("press Left button to start standy");
 delay(1000);
}
*/

//rb.subst=1;                   //sub状態遷移変数

time_old = millis();
//-TimerLoop----------------
  Timer.stop();
  Timer.setPeriod(TIMER_RATE);           // in microseconds
  Timer.attachInterrupt(timerloop);
  Timer.start(); 
//-----------------------


}



/////////////   loop  ////////////////////////////////
void loop() 
{
   const char *log;  
   bool result = false;   
    
   RC100_update(); 

       
// readCommand();

  if(servoOutputFlag){
    dxl_wb.goalPosition(DJ[J1].id, (int32_t)DJ[J1].pulse.g);  
    dxl_wb.goalPosition(DJ[J2].id, (int32_t)DJ[J2].pulse.g);
    dxl_wb.goalPosition(DJ[J3].id, (int32_t)DJ[J3].pulse.g);
    dxl_wb.goalPosition(DJ[J4].id, (int32_t)DJ[J4].pulse.g);
    dxl_wb.goalPosition(DJ[J5].id, (int32_t)DJ[J5].pulse.g);   
    dxl_wb.goalPosition(DJ[J6].id, (int32_t)DJ[J6].pulse.g);         
    servoOutputFlag = 0;
  }  
}

//////////////////////////TimerLoop  //////////////////
void timerloop(void) {

   const char *log;  
   bool result = false;   
  
    time_passed=millis()-time_old;
  
 ////角度取得 pulse to ang_n
  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++) calPulse2Angle(DJ[i].jnum);   
  
 //// タスク  
  switch(modeNum){

    case 0:                //Joystick
       //joystickTask();           
       if(moveJoint(DJ[1].jnum,50,3)) 
        if(moveJoint(DJ[1].jnum,-50,3))
         if(moveJoint(DJ[1].jnum,0,3)) modeNum=100;
        
       break;
    case st_OfflineSinTrajectory: 
          DJ[J1].trFlag=exeTrajectoryPoints(DJ[J1].jnum);
          DJ[J2].trFlag=exeTrajectoryPoints(DJ[J2].jnum);
          DJ[J3].trFlag=exeTrajectoryPoints(DJ[J3].jnum);
          DJ[J4].trFlag=exeTrajectoryPoints(DJ[J4].jnum);
          DJ[J5].trFlag=exeTrajectoryPoints(DJ[J5].jnum);     
          DJ[J6].trFlag=exeTrajectoryPoints(DJ[J6].jnum);     
//          if(rtimeDelay(3000)){
//            if(DJ[J1].trFlag){modeNum=100; DJ[J1].trIdx=0;DJ[J1].trFlag=0;}
//          }
          
          setGoalAngle2Pulse(DJ[J1].jnum);
          //setGoalAngle2Pulse(DJ[J2].jnum);
          setGoalAngle2Pulse(DJ[J3].jnum);
          setGoalAngle2Pulse(DJ[J4].jnum);
          setGoalAngle2Pulse(DJ[J5].jnum);
          setGoalAngle2Pulse(DJ[J6].jnum);
          
          Serial.print(DJ[J1].ang.n);  Serial.print('\t'); 
          Serial.print(DJ[J2].ang.n);  Serial.print('\t'); 
          Serial.println(DJ[J3].ang.n);    
       break;  
         
     case 100:
  //     Serial.print(distance_vl5310x(0));    Serial.print("\t");
   //    Serial.print(distance_vl5310x(1));    Serial.print("\t"); 
   //    Serial.print(modeNum); Serial.print("\t");
    
     //for(uint8_t i=J1; i <= J6 ;i++){ Serial.print(DJ[i].ang.n);  Serial.print('\t');}
      
       break;
      
   default: break;
  }

  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){ DJ[i].ang.g_buf = DJ[i].ang.g;}
 servoOutputFlag = 1; 
}



void testJointMotion(){

  Serial.print("Start each joint motion testing after 3s: ");
  delay(3000);
  goInitJointConfig(DJ[J1].jnum, -45,1);  //関節を目標角度まで駆動 
  goInitJointConfig(DJ[J1].jnum, 45,1);  //関節を目標角度まで駆動 
  goInitJointConfig(DJ[J1].jnum, 0,1);  //関節を目標角度まで駆動 
  delay(500);
  goInitJointConfig(DJ[J2].jnum, 30,1);  //関節を目標角度まで駆動 
  goInitJointConfig(DJ[J2].jnum, 150,1);  //関節を目標角度まで駆動 
  delay(500);
  goInitJointConfig(DJ[J3].jnum, 10,3);  //関節を目標角度まで駆動 
   goInitJointConfig(DJ[J3].jnum, 90,3);  //関節を目標角度まで駆動 
  delay(500);
  goInitJointConfig(DJ[J4].jnum, 90,3);  //関節を目標角度まで駆動 
  goInitJointConfig(DJ[J4].jnum, 0,3);  //関節を目標角度まで駆動 
  delay(500);
  goInitJointConfig(DJ[J5].jnum, -90,5);  //関節を目標角度まで駆動 
  goInitJointConfig(DJ[J5].jnum, 0,5);  //関節を目標角度まで駆動 
  delay(500);
  goInitJointConfig(DJ[J6].jnum, 90,3);  //関節を目標角度まで駆動 
  goInitJointConfig(DJ[J6].jnum, 0,3);  //関節を目標角度まで駆動 
 }
 
void testMakeTrajectoryPoints(){

  /////////////Trajectoryの生成///////////////////////////////////
 float startTime=0.0;         //軌道スタート時間s
 float finishTime=5.0;        //軌道終了時間s
 float invTime = (TIMER_RATE/1000)*0.001;                               //インターバル時間
 uint16_t timeStampIndex =0;          //時間index
 uint16_t trajIndex=0;
 float tt;                            //時間

  for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){
  
 DJ[i].trIdxMax=(uint16_t)((finishTime-startTime)/invTime);    //最大index値

 for(tt=startTime; tt<=finishTime; tt+=invTime){
//   trajTT[timeStampIndex++] = tt;  // 2*3.14*f(hz) 
   DJ[i].trajPoints[trajIndex] = 30*sin(2*3.14*0.2*tt);  // 2*3.14*f(hz) 
   trajIndex++;
 }
 trajIndex=0;
}

 for(uint8_t i=Jnum_S; i <= Jnum_F ;i++){
  for(int idx=0; idx<=DJ[i].trIdxMax; idx++){
  //Serial.print(trajTT[i]); Serial.print('\t'); 
   Serial.println(DJ[i].trajPoints[idx]);
  }
   Serial.print(DJ[i].jnum); Serial.println("finished"); delay(3000);
 }
/////////////////////////////////////////////////////////////// 
}


  
