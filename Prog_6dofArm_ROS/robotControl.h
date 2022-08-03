#define ClimbUp 1
#define ClimbDown 0


/* Robot Struct */
typedef struct __robot_type{

  uint8_t st,subst;
//  uint8_t pause_st;              //一時停止時の状態 
//  uint8_t pause_subst;            //一時停止時のサブ状態  
//  uint8_t pause_flag;             //一時停止フラグ

  uint8_t js_finish_flag;
 // uint8_t fc1_finish_flag;
 // uint8_t fc2_finish_flag;

  int32_t distVL[2];
  
}Robot;

//動作関数
uint8_t releaseFixLeg();
uint8_t downLifter();
uint8_t holdFixLeg();
uint8_t releaseMovLeg();
uint8_t upLifter();
uint8_t holdMovLeg();

typedef enum{ js_start=1,            
              js_fixedLegGrip,
              js_movableLegGrip,
              js_finish
}subst_Joystick;

void joystickTask();


/*
typedef enum{ fc1_start=1,
              fc1_moveBaeStartPoint, 
              fc1_moveFloorBaseStartPoint, 
              fc1_alignFloorTip,
              fc1_downFloorArmOnFloor,
              fc1_moveBaseToMidPoint,
              fc1_bendFoorTip,
              fc1_backFloorBaseToMidPoint,
              fc1_backBaseToOriginPoint,
              fc1_upFloorArmToOriginPoint,
              fc1_backFloorTipToOriginPoint,
              fc1_backFloorBaseToOriginPoint,
              fc1_finish
}subst_FloorClean1;

typedef enum{ fc2_start=1,
              fc2_moveBaeStartPoint, 
              fc2_moveFloorBaseStartPoint, 
              fc2_alignFloorTip,
              fc2_downFloorArmOnFloor,
              fc2_moveBaseToMidPoint,
              fc2_bendFoorTip,
              fc2_backFloorBaseToMidPoint,
              fc2_backBaseToOriginPoint,
              fc2_upFloorArmToOriginPoint,
              fc2_backFloorTipToOriginPoint,
              fc2_backFloorBaseToOriginPoint,
              fc2_finish
}subst_FloorClean2;

uint8_t goInitialPose();
void floorCleaning1Task();
void floorCleaning2Task();
void seatCleaningTask();

*/
