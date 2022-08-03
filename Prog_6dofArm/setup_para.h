//#define __SERIAL_WAIT_ON               //実動作試験ではコメントアウト


#define Jnum 6  
#define Jnum_S  1     //実行したい関節の開始番号
#define Jnum_F  6     //実行したい関節の終了番号

// Joint Number
#define J1   1     //Joint 1 (XM540)
#define J2   2     //Joint 2 (XM540)
#define J20  20    //Joing 2-1 (XM540)
#define J3   3     //Joint 3 (XM540)  
#define J4   4     //Joint 4 (XM540)
#define J5   5     //Joint 5 (XM430)  
#define J6   6     //Joing 6 (XM430)


// Dynamixel Joint ID
#define ID_J1     1        //XM540-W270-R ( rpm, Nm) 
#define ID_J2     2        //XM540-W270-R ( rpm, Nm) 
#define ID_J20    20       //XM540-W270-R ( rpm, Nm) 
#define ID_J3     3        //XM540-W270-R ( rpm, Nm) 
#define ID_J4     4        //XM430-W210-R ( rpm, Nm) 
#define ID_J5     5        //XM430-W210-R ( rpm, Nm) 
#define ID_J6     6        //XM430-W210-R ( rpm, Nm) 

 //直動の距離変換係数：
 //ラックピニオン：ピニオン半径r*deg*(pi/180), ボールねじ：リード,  回転ジョイント：１
#define Deg2Dist_J1     1   
#define Deg2Dist_J2     1    
#define Deg2Dist_J3     1
#define Deg2Dist_J4     1    
#define Deg2Dist_J5     1    
#define Deg2Dist_J6     1  

//初期姿勢角度でのdynamixelの内部角度(dynamixel offset)
#define OffsetPul_J1 0
#define OffsetPul_J2 0
#define OffsetPul_J3 0
#define OffsetPul_J4 0
#define OffsetPul_J5 0
#define OffsetPul_J6 0

//----初期姿勢角度 (read dynangle - dynoffsetangle)
#define InitAng_J1 0       
#define InitAng_J2 0 
#define InitAng_J3 0 
#define InitAng_J4 0 
#define InitAng_J5 0
#define InitAng_J6 0

// 各ジョイントの最小ー最大角度
#define LimMinAng_J1 -70 
#define LimMaxAng_J1  290    
#define LimMinAng_J2 -10
#define LimMaxAng_J2 150     
#define LimMinAng_J3 -100
#define LimMaxAng_J3 90     
#define LimMinAng_J4 0
#define LimMaxAng_J4 359     
#define LimMinAng_J5 0
#define LimMaxAng_J5 180     
#define LimMinAng_J6 0
#define LimMaxAng_J6 359    


void SetupParameter();
