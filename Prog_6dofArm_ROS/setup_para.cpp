#include <DynamixelWorkbench.h>
#include "dynamixelHsrlLib.h"
#include "setup_para.h"
extern DYNAMIXEL_JOINT DJ[];


void SetupParameter(){

//Lift  
 DJ[J1].jnum = J1;
 DJ[J1].id   = ID_J1;
 DJ[J1].pul2deg = Pulse2Deg_XM540;
 DJ[J1].gearRatio = 1;                 //ギア比
 DJ[J1].deg2dist =  Deg2Dist_J1;       
 DJ[J1].pul2deg_coff = DJ[J1].pul2deg*DJ[J1].gearRatio*DJ[J1].deg2dist;   
 DJ[J1].deg2pul_coff = (1/DJ[J1].pul2deg)*(1/DJ[J1].gearRatio)*(1/DJ[J1].deg2dist);
 DJ[J1].dir_rot = 1; 
 DJ[J1].revmode = ReverseMode_J1 ;
 DJ[J1].pulse.offset = OffsetPul_J1;
 DJ[J1].ang.offset = OffsetAng_J1;
 DJ[J1].ang.init = InitAng_J1;
 DJ[J1].ang.lim_min = LimMinAng_J1;
 DJ[J1].ang.lim_max = LimMaxAng_J1;
  

 //Fixed Leg Left
 DJ[J2].jnum = J2;
 DJ[J2].id   = ID_J2;
 DJ[J2].pul2deg = Pulse2Deg_XM540;  
 DJ[J2].gearRatio = 1;                               //ギア比
 DJ[J2].deg2dist =  Deg2Dist_J2;                     //直動の距離変換係数：ピニオン半径r(20mm)*deg*(pi/180)
 DJ[J2].pul2deg_coff = DJ[J2].pul2deg*DJ[J2].gearRatio*DJ[J2].deg2dist;   
 DJ[J2].deg2pul_coff = (1/DJ[J2].pul2deg)*(1/DJ[J2].gearRatio)*(1/DJ[J2].deg2dist);
 DJ[J2].dir_rot = 1; 
 DJ[J2].revmode = ReverseMode_J2 ;
 DJ[J2].pulse.offset = OffsetPul_J2;
 DJ[J2].ang.offset = OffsetAng_J2;
 DJ[J2].ang.init = InitAng_J2;
 DJ[J2].ang.lim_min = LimMinAng_J2;
 DJ[J2].ang.lim_max = LimMaxAng_J2;


 //Fixed Leg Right
 DJ[J3].jnum = J3;
 DJ[J3].id   = ID_J3;
 DJ[J3].pul2deg = Pulse2Deg_XM540;  
 DJ[J3].gearRatio = 1;                               //ギア比
 DJ[J3].deg2dist =  Deg2Dist_J3;                     //直動の距離変換係数：ピニオン半径r(20mm)*deg*(pi/180)
 DJ[J3].pul2deg_coff = DJ[J3].pul2deg*DJ[J3].gearRatio*DJ[J3].deg2dist;   
 DJ[J3].deg2pul_coff = (1/DJ[J3].pul2deg)*(1/DJ[J3].gearRatio)*(1/DJ[J3].deg2dist);
 DJ[J3].dir_rot = 1; 
 DJ[J3].revmode = ReverseMode_J3 ;
 DJ[J3].pulse.offset = OffsetPul_J3;
 DJ[J3].ang.offset = OffsetAng_J3;
 DJ[J3].ang.init = InitAng_J3;
 DJ[J3].ang.lim_min = LimMinAng_J3;
 DJ[J3].ang.lim_max = LimMaxAng_J3;


 
 //Movable Leg Front
 DJ[J4].jnum = J4;
 DJ[J4].id   = ID_J4;
 DJ[J4].pul2deg = Pulse2Deg_XM430;  
 DJ[J4].gearRatio = 1;                               //ギア比
 DJ[J4].deg2dist =  Deg2Dist_J4;                     //直動の距離変換係数：ピニオン半径r(20mm)*deg*(pi/180)
 DJ[J4].pul2deg_coff = DJ[J4].pul2deg*DJ[J4].gearRatio*DJ[J4].deg2dist;   
 DJ[J4].deg2pul_coff = (1/DJ[J4].pul2deg)*(1/DJ[J4].gearRatio)*(1/DJ[J4].deg2dist);
 DJ[J4].dir_rot = 1; 
 DJ[J4].revmode = ReverseMode_J4 ;
 DJ[J4].pulse.offset = OffsetPul_J4;
 DJ[J4].ang.offset = OffsetAng_J4;
 DJ[J4].ang.init = InitAng_J4;
 DJ[J4].ang.lim_min = LimMinAng_J4;
 DJ[J4].ang.lim_max = LimMaxAng_J4;



 //Movable Leg Back
 DJ[J5].jnum = J5;
 DJ[J5].id   = ID_J5;
 DJ[J5].pul2deg = Pulse2Deg_XM430;  
 DJ[J5].gearRatio = 1;                               //ギア比
 DJ[J5].deg2dist =  Deg2Dist_J5;                     //直動の距離変換係数：ピニオン半径r(20mm)*deg*(pi/180)
 DJ[J5].pul2deg_coff = DJ[J5].pul2deg*DJ[J5].gearRatio*DJ[J5].deg2dist;   
 DJ[J5].deg2pul_coff = (1/DJ[J5].pul2deg)*(1/DJ[J5].gearRatio)*(1/DJ[J5].deg2dist);
 DJ[J5].dir_rot = 1; 
 DJ[J5].revmode = ReverseMode_J5 ;
 DJ[J5].pulse.offset = OffsetPul_J5;
 DJ[J5].ang.offset = OffsetAng_J5; 
 DJ[J5].ang.init = InitAng_J5;
 DJ[J5].ang.lim_min = LimMinAng_J5;
 DJ[J5].ang.lim_max = LimMaxAng_J5;

  //Movable Leg Back
 DJ[J6].jnum = J6;
 DJ[J6].id   = ID_J6;
 DJ[J6].pul2deg = Pulse2Deg_XM430;  
 DJ[J6].gearRatio = 1;                               //ギア比
 DJ[J6].deg2dist =  Deg2Dist_J6;                     //直動の距離変換係数：ピニオン半径r(20mm)*deg*(pi/180)
 DJ[J6].pul2deg_coff = DJ[J6].pul2deg*DJ[J6].gearRatio*DJ[J6].deg2dist;   
 DJ[J6].deg2pul_coff = (1/DJ[J6].pul2deg)*(1/DJ[J6].gearRatio)*(1/DJ[J6].deg2dist);
 DJ[J6].dir_rot = 1; 
 DJ[J6].revmode = ReverseMode_J6; 
 DJ[J6].pulse.offset = OffsetPul_J6;
 DJ[J6].ang.offset = OffsetAng_J6; 
 DJ[J6].ang.init = InitAng_J6;
 DJ[J6].ang.lim_min = LimMinAng_J6;
 DJ[J6].ang.lim_max = LimMaxAng_J6;
   
}
