
#include <RC100.h>
#include <DynamixelSDK.h>

#define ON  1
#define OFF 0

RC100 Controller;
int RcvData = 0;
extern DYNAMIXEL_JOINT DJ[];
extern uint8_t modeNum;
extern uint8_t updownFlag;

void RC100_setup() {

  Controller.begin(2);

}

void RC100_update() {
  
  if (Controller.available()) {

    RcvData = Controller.readData();
    if      ((RcvData & RC100_BTN_U)) {  DJ[J1].dang.g = 20; }
    else if ((RcvData & RC100_BTN_D)) {  DJ[J1].dang.g = -20; }
    else if ((RcvData & RC100_BTN_R)) {  DJ[J2].dang.g = 10; DJ[J3].dang.g=10; }
    else if ((RcvData & RC100_BTN_L)) {  DJ[J2].dang.g = -10; DJ[J3].dang.g=-10; }
    else if ((RcvData & RC100_BTN_4)) {  DJ[J4].dang.g = 10; DJ[J5].dang.g = 10; }
    else if ((RcvData & RC100_BTN_2)) {  DJ[J4].dang.g = -10; DJ[J5].dang.g = -10; }
    else if ((RcvData & RC100_BTN_5)) {  updownFlag = ClimbUp;  }   
   else if ((RcvData & RC100_BTN_6)) {  updownFlag = ClimbDown;  }    
    else {  
      DJ[J1].dang.g = 0;
      DJ[J2].dang.g = 0; 
      DJ[J3].dang.g = 0;
      DJ[J4].dang.g = 0; 
      DJ[J5].dang.g = 0;
     
      }
 
     return;
  } 
  /*
 else{
  DJ[MF].dx.g = 0;
  DJ[MB].dx.g = 0; 
 }*/
     
 
}
