//#define PiPaArm_3dPrinter
#define PiPaArm_fieldProto

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


//初期姿勢角度でのdynamixelの内部角度(dynamixel offset) homingoffsetが使えないposition modeでoffset角度を与えて角度を計算する場合
#define OffsetPul_J1 0
#define OffsetPul_J2 0
#define OffsetPul_J3 0
#define OffsetPul_J4 0
#define OffsetPul_J5 0
#define OffsetPul_J6 0


/*----homingoffsetを使ってoffset角度を任意の角度に設定する場合----------------*/  
#ifdef PiPaArm_fieldProto
//extendded position mode      
#define OffsetAng_J1 130 //180 
#define OffsetAng_J2 307 //257 
#define OffsetAng_J3 117 //187 
#define OffsetAng_J4 180 ////180 
#define OffsetAng_J5 182 // 120.42 ////182 
#define OffsetAng_J6 180 //180 

//初期角度（homingoffsetを使ってoffset角度をこの角度に設定する）（extendded position mode）
#define InitAng_J1 -50   //0        //0.09  170.35  -90.89  -0.53 -59.86  0.00 
#define InitAng_J2 140  //90       //169.47
#define InitAng_J3 -70  //0        //-90.80
#define InitAng_J4 0    //0        //-0.97
#define InitAng_J5 0    //         //-59.42
#define InitAng_J6 0        //0

//Reverse mode設定:-1  設定してない：1
#define ReverseMode_J1 1       
#define ReverseMode_J2 1 
#define ReverseMode_J3 1 
#define ReverseMode_J4 1 
#define ReverseMode_J5 -1   //1
#define ReverseMode_J6 1

#endif


/*----homingoffsetを使ってoffset角度を任意の角度に設定する場合----------------*/
#ifdef PiPaArm_3dPrinter
//extendded position mode
#define OffsetAng_J1 180
#define OffsetAng_J2 48
#define OffsetAng_J3 224
#define OffsetAng_J4 180
#define OffsetAng_J5 134
#define OffsetAng_J6 180

//初期角度（homingoffsetを使ってoffset角度をこの角度に設定する）（extendded position mode）
#define InitAng_J1 0       
#define InitAng_J2 90 
#define InitAng_J3 0 
#define InitAng_J4 0 
#define InitAng_J5 0
#define InitAng_J6 0

//Reverse mode設定:-1  設定してない：1
#define ReverseMode_J1 1       
#define ReverseMode_J2 1 
#define ReverseMode_J3 1 
#define ReverseMode_J4 1 
#define ReverseMode_J5 1   //1
#define ReverseMode_J6 1

#endif





/*--------------------*/
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
