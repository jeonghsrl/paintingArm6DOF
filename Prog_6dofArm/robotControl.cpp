#include <DynamixelWorkbench.h>
#include "dynamixelHsrlLib.h"
#include "setup_para.h"
#include "robotControl.h"


extern DYNAMIXEL_JOINT DJ[];
extern Robot rb;

////////////////////////////////////////
void joystickTask(){

   switch(rb.subst){
    case js_start:
        // 位置更新
        DJ[J1].ang.g +=DJ[J1].dang.g*0.05;
        DJ[J2].ang.g +=DJ[J2].dang.g*0.05;
        DJ[J3].ang.g +=DJ[J3].dang.g*0.05;
        DJ[J4].ang.g +=DJ[J4].dang.g*0.05;
        DJ[J5].ang.g +=DJ[J5].dang.g*0.05;

//        if(rb.distVL[0] < 100 ) rb.subst=js_fixedLegGrip;
//        if(rb.distVL[1] < 100 ) rb.subst=js_movableLegGrip;        
     
        //自動昇降へ(最初は、自動で各phaseへ移動
//        if(updownFlag==ClimbUp||updownFlag==ClimbDown) modeNum=7;   
          break;         
          
    case js_fixedLegGrip:
//        Serial.print(rb.subst); Serial.println(" grip");
         DJ[J2].ang.g +=DJ[J2].dang.g*0.05;
         DJ[J3].ang.g +=DJ[J3].dang.g*0.05;
          break;

     case js_movableLegGrip:
//        Serial.print(rb.subst); Serial.println(" grip");
         DJ[J3].ang.g +=DJ[J3].dang.g*0.05;
         DJ[J4].ang.g +=DJ[J4].dang.g*0.05;
         break;

    default: break;

   }
   
    //可動限界設定
   for (uint8_t i=J1; i <= J5 ;i++){  
        if(DJ[i].ang.g >= DJ[i].ang.lim_max ) DJ[i].ang.g =DJ[i].ang.lim_max;
        if(DJ[i].ang.g <= DJ[i].ang.lim_min ) DJ[i].ang.g = DJ[i].ang.lim_min;
    } 
    
    //目標角度をdynamixel用の目標pulseに変換
    for(uint8_t i=Jnum_S; i <= Jnum_F ;i++) setGoalAngle2Pulse(DJ[i].id);  

   Serial.print(DJ[J1].ang.g); Serial.print('\t');  
   Serial.print(DJ[J2].ang.g); Serial.print('\t');  
   Serial.print(DJ[J3].ang.g); Serial.print('\t');    
   Serial.print(DJ[J4].ang.g); Serial.print('\t');
   Serial.print(DJ[J5].ang.g); Serial.print('\t');
 //  Serial.print(rb.distVL[0]);    Serial.print("\t");
   Serial.print(getPresentCurrent(DJ[J1].jnum));    Serial.print("\t"); 
   Serial.println();  
}

/*

////////////////////////////////////////
void floorCleaning1Task(){             

   switch(rb.subst){

    case fc1_start:
        if(rtimeDelay(3000)) rb.subst=fc1_moveBaeStartPoint;
        break;
             
    case fc1_moveBaeStartPoint:
          Serial.print(rb.subst); Serial.println(" fc1_moveBaeStartPoint");         
          if(moveJoint(DJ[J1].jnum, 1140, 5 )) rb.subst=fc1_moveFloorBaseStartPoint;   
      break;

    case fc1_moveFloorBaseStartPoint:
          Serial.print(rb.subst); Serial.println(" fc1_moveFloorBaseStartPoint");
         if(moveJoint(DJ[J2].jnum, 125, 5 )) rb.subst=fc1_alignFloorTip;   
      break;

    case  fc1_alignFloorTip:
         Serial.print(rb.subst); Serial.println(" fc1_alignFloorTip");
         if(moveJoint(DJ[J4].jnum, 180, 5 )) rb.subst=fc1_downFloorArmOnFloor;   
      break;

    case fc1_downFloorArmOnFloor:
         Serial.print(rb.subst); Serial.println(" fc1_downFloorArmOnFloor");
         if(moveJoint(DJ[J3].jnum, -42, 5 )) rb.subst=fc1_moveBaseToMidPoint;   
      break;

    case fc1_moveBaseToMidPoint:
        Serial.print(rb.subst); Serial.println(" fc1_moveBaseToMidPoint");
         if(moveJoint(DJ[J1].jnum, 630, 5 )) rb.subst=fc1_bendFoorTip;  
      break;

   case fc1_bendFoorTip:
        Serial.print(rb.subst); Serial.println(" fc1_bendFoorTip");
         if(moveJoint(DJ[J4].jnum, 90, 5 )) rb.subst=fc1_backBaseToOriginPoint; 
      break;
      
   case fc1_backBaseToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc1_backBaseToOriginPoint");
        if(moveJoint(DJ[J1].jnum, 10, 5 )) rb.subst=fc1_upFloorArmToOriginPoint; 
      break;
      
   case fc1_upFloorArmToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc1_upFloorArmToOriginPoint");
        if(moveJoint(DJ[J3].jnum, 100, 5 )) rb.subst=fc1_backFloorBaseToOriginPoint; 
      break;

   case fc1_backFloorBaseToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc1_backFloorBaseToOriginPoint");
        if(moveJoint(DJ[J2].jnum, 10, 5 ))  rb.subst=fc1_backFloorTipToOriginPoint; 
      break;    
      
   case fc1_backFloorTipToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc1_backFloorTipToOriginPoint");
        if(moveJoint(DJ[J4].jnum, 90, 5 )) rb.subst=fc1_finish;
      break;      

   

    default: break;
   }
   
}


////////////////////////////////////////
void floorCleaning2Task(){             

   switch(rb.subst){

    case fc2_start:
        if(rtimeDelay(3000)) rb.subst=fc2_moveBaeStartPoint;
        break;
             
    case fc2_moveBaeStartPoint:
          Serial.print(rb.subst); Serial.println(" fc2_moveBaeStartPoint");         
          if(moveJoint(DJ[J1].jnum, 1140, 5 )) rb.subst=fc2_moveFloorBaseStartPoint;   
      break;

    case fc2_moveFloorBaseStartPoint:
          Serial.print(rb.subst); Serial.println(" fc2_moveFloorBaseStartPoint");
         if(moveJoint(DJ[J2].jnum, 310, 5 )) rb.subst=fc2_alignFloorTip;   
      break;

    case  fc2_alignFloorTip:
         Serial.print(rb.subst); Serial.println(" fc2_alignFloorTip");
         if(moveJoint(DJ[J4].jnum, 180, 5 )) rb.subst=fc2_downFloorArmOnFloor;   
      break;

    case fc2_downFloorArmOnFloor:
         Serial.print(rb.subst); Serial.println(" fc2_downFloorArmOnFloor");
         if(moveJoint(DJ[J3].jnum, -42, 5 )) rb.subst=fc2_moveBaseToMidPoint;   
      break;

    case fc2_moveBaseToMidPoint:
        Serial.print(rb.subst); Serial.println(" fc2_moveBaseToMidPoint");
         if(moveJoint(DJ[J1].jnum, 624, 5 )) rb.subst=fc2_bendFoorTip;  
      break;

   case fc2_bendFoorTip:
        Serial.print(rb.subst); Serial.println(" fc2_bendFoorTip");
         if(moveJoint(DJ[J4].jnum, 90, 5 )) rb.subst=fc2_backFloorBaseToMidPoint;  
      break;

   case fc2_backFloorBaseToMidPoint:
        Serial.print(rb.subst); Serial.println(" fc2_backFloorBaseToMidPoint");
         if(moveJoint(DJ[J2].jnum, 125, 5 )) rb.subst=fc2_backBaseToOriginPoint; 
      break;      

   case fc1_backBaseToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc2_backBaseToOriginPoint");
        if(moveJoint(DJ[J1].jnum, 10, 5 )) rb.subst=fc2_upFloorArmToOriginPoint; 
      break;
      
   case fc2_upFloorArmToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc2_upFloorArmToOriginPoint");
        if(moveJoint(DJ[J3].jnum, 100, 5 )) rb.subst=fc2_backFloorBaseToOriginPoint; 
      break;  

   case fc2_backFloorBaseToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc2_backFloorBaseToOriginPoint");
        if(moveJoint(DJ[J2].jnum, 10, 5 ))  rb.subst=fc2_backFloorTipToOriginPoint; 
      break;    

    case fc2_backFloorTipToOriginPoint:
        Serial.print(rb.subst); Serial.println(" fc2_backFloorTipToOriginPoint");
        if(moveJoint(DJ[J4].jnum, 90, 5 )) rb.subst=fc2_finish;
      break;     

    default: break;
   }
   
}
*/


//=============固定足開放
uint8_t releaseFixLeg(){
  /*
       // goal distanceを設定
       DJ[J2].x.g=LPPos_FixLeg;     
       
       // goal distをgoal pulseに変換
       DJ[J2].pulse.g=calDist2Pulse(DJ[J2].id,DJ[J2].x.g);
  
       //現在位置が1mm以下であれば、足上げ動作へ
       if(fabs(DJ[FR].x.g-DJ[FR].x.n) < 1 && fabs(DJ[FL].x.g-DJ[FL].x.n) < 1) return 1;
       else return 0;
       */
       }

//=============リフター下降       
uint8_t downLifter(){
  /*
       //リフトのgoal distanceを設定とgoal pulseへの変換
       DJ[LF].x.g=DownPos_Lifter;   //胴体を上げて可動アームを保持フェーズへ移動させるための位置
       DJ[LF].pulse.g=calDist2Pulse(DJ[LF].id,DJ[LF].x.g);

      //現在位置が1mm以下であれば、可動足把持動作モードへ
       if(fabs(DJ[LF].x.g-DJ[LF].x.n) < 1) return 1;
       else return 0;*/
       }

//=============固定足把持  
uint8_t holdFixLeg(){       /*
       // goal distanceを設定
       DJ[FR].x.g=HPPos_FixLeg;       DJ[FL].x.g=HPPos_FixLeg;
       //--- 目標電流値設定----//
       // goal distをgoal pulseに変換
       DJ[FR].pulse.g=calDist2Pulse(DJ[FR].id,DJ[FR].x.g);
       DJ[FL].pulse.g=calDist2Pulse(DJ[FL].id,DJ[FL].x.g);

       DJ[FR].current.n = double(getPresentCurrent(DJ[FR].id));
       DJ[FL].current.n = double(getPresentCurrent(DJ[FL].id));

       if( fabs(HCt_Legs) < fabs(DJ[FR].current.n) && fabs(HCt_Legs) < fabs(DJ[FL].current.n)) return 1;
       else return 0;*/
}

//=============可動足開放  
uint8_t releaseMovLeg(){/*
       // goal distanceを設定
       DJ[MF].x.g=LPPos_MovLeg;       DJ[MB].x.g=LPPos_MovLeg;
       
       // goal distをgoal pulseに変換
       DJ[MF].pulse.g=calDist2Pulse(DJ[MF].id,DJ[MF].x.g);
       DJ[MB].pulse.g=calDist2Pulse(DJ[MB].id,DJ[MB].x.g);

       //現在位置が1mm以下であれば、足上げ動作へ
       if(fabs(DJ[MF].x.g-DJ[MF].x.n) < 1 && fabs(DJ[MB].x.g-DJ[MB].x.n) < 1) return 1;
       else return 0;*/
}

//=============リフター上昇  
uint8_t upLifter(){       /*
       //リフトのgoal distanceを設定とgoal pulseへの変換
       DJ[LF].x.g=UpPos_Lifter;
       DJ[LF].pulse.g=calDist2Pulse(DJ[LF].id,DJ[LF].x.g);

      //現在位置が1mm以下であれば、可動足把持動作モードへ
       if(fabs(DJ[LF].x.g-DJ[LF].x.n) < 1)  return 1;
       else return 0;*/
}

//=============可動足把持 
uint8_t holdMovLeg(){   /*
       // 保持用の目標距離を設定
       DJ[MF].x.g=HPPos_MovLeg;       DJ[MB].x.g=HPPos_MovLeg;
       //--- 目標電流値設定----//
       // goal distをgoal pulseに変換
       DJ[MF].pulse.g=calDist2Pulse(DJ[MF].id,DJ[MF].x.g);
       DJ[MB].pulse.g=calDist2Pulse(DJ[MB].id,DJ[MB].x.g);

       DJ[MF].current.n = double(getPresentCurrent(DJ[MF].id));
       DJ[MB].current.n = double(getPresentCurrent(DJ[MB].id));

     //HoldingCurrent以上であれば、次の上り動作へ
       if( fabs(HCt_Legs) < fabs(DJ[MF].current.n) && fabs(HCt_Legs) < fabs(DJ[MB].current.n)) return 1;       
       else return 0;*/
}


//////Go init pose
uint8_t goInitialPose(){
  
    uint8_t result=0;
    static uint8_t caseNum=1;
/*    
    switch(caseNum){
      case 1:  if(moveJoint(DJ[J3].jnum, 20, 5 )&& moveJoint(DJ[J5].jnum, 90, 5 )) caseNum=2;   break;
      case 2:  if(moveJoint(DJ[J2].jnum, 10, 5 )) caseNum=3;   break;
      case 3:  if(moveJoint(DJ[J1].jnum, 10, 5 )) caseNum=4;   break;
      case 4:  if(moveJoint(DJ[J4].jnum, 90, 5 )) caseNum=5;   break;
      case 5:  if(moveJoint(DJ[J3].jnum, 100, 5 )) {caseNum=1; result=1;  break;}
      default: break;   
    } 
  */ 
    return result;
  
  }
