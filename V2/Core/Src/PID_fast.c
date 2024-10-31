
#include "main.h"
#include "PID_fast.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
extern uint8_t init_state;


/* USER CODE END EV */


void PID_REG(Motor_Sruct *Motor){
  uint8_t dead_zone1=0, dead_zone2=0,dead_zone3=0,direct_pid1=0,direct_pid2=0,direct_pid3=0; 
  int64_t  Error1=0;
  int32_t regD1=0,regP1=0,PID1;
  int16_t  Error2;
  int32_t regD2=0,regP2=0,PID2;
  int16_t  Error3;
  int32_t regD3=0,regP3=0,PID3;
  int32_t temp;
  //-------------------pid1------------------------------------------//   
#ifndef debug_PID_Current  
#ifndef debug_PID_Velocity
  Error1=Motor->position_sp-Motor->position;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  constrain_(Error1,10000,-10000);
  regP1=Error1*Motor->P_position/10;               // position difference will determine rotation
  if (Motor->I_position){ 
    Motor->regI1+=(Error1*Motor->I_position)/100; //becouse time is 0.1  error*Ki*dt
  }else{
    Motor->regI1=0;
  }
  regD1=(Error1-Motor->Error_old1)*Motor->D_position*1; //becouse dt is 0.1
  regD1=constrain_(regD1,10000,-10000);
  Motor->regI1=constrain_(Motor->regI1,10000,-10000);
  Motor->Error_old1=Error1;
  if ((Error1>=dead_zone1)||(Error1<=(dead_zone1*(-1))) )  
  {
    
    if (direct_pid1){
      PID1=(regP1+Motor->regI1+regD1)*(-1);
    }else{
      PID1=regP1+Motor->regI1+regD1;
    }
    PID1=constrain_(PID1,10000,-10000);
    
    if (init_state!=2){
      temp=PID1*Motor->velocity_max/10000; //0-velocity_max
      Motor->velocity_sp=(int16_t)temp;
    }
  }
#endif
  //-------------------pid2------------------------------------------//   
  Error2=Motor->velocity_sp-Motor->velocity_average;    
  
  regP2=(Error2*Motor->P_velocity)/10;  
  
  if (Motor->I_velocity){
    Motor->regI2+=(Error2*Motor->I_velocity)/100; //becouse time is 0.1  error*Ki*dt
  }else{
    Motor->regI2 =0; 
  }
  regD2=(Error2-Motor->Error_old2)*Motor->D_velocity*1; //becouse dt is 0.1
  regD2=constrain_(regD2,10000,-10000);
  Motor->regI2=constrain_(Motor->regI2,10000,-10000);
  Motor->Error_old2=Error2;
  if ((Error2>=dead_zone2)||(Error2<=(dead_zone2*(-1))) )  
  {
    
    if (direct_pid2){
      PID2=(regP2+Motor->regI2+regD2)*(-1);
    }else{
      PID2=regP2+Motor->regI2+regD2;
    }
    PID2=constrain_(PID2,10000,0);
    temp=Motor->I_M_max*PID2/10000;
    Motor->I_M_sp=(int16_t)temp;
  }
#endif    
  if (Error1<0){ //CW or CCW according to position error
    Motor->curr_direction=-1;
  }else{
    Motor->curr_direction=1;
  }
  //-------------------pid3------------------------------------------//   
  Error3=Motor->I_M_sp-(Motor->I_M); 
  regP3=(Error3*Motor->P_current)/10;  //and Kp 0-25.6
  if (Motor->I_current){
    Motor->regI3+=(Error3*Motor->I_current)/100; //becouse time is 0.1  error*Ki*dt and Ki 0-25.6
  }else{
    Motor->regI3 =0; 
  }
  regD3=(Error3-Motor->Error_old3)*Motor->D_current*1; //becouse dt is 0.1 and Kd 0-25.6
  regD3=constrain_(regD3,10000,-10000);
  Motor->regI3=constrain_(Motor->regI3,10000,-10000);
  Motor->Error_old3=Error3;
  if ((Error3>=dead_zone3)||(Error3<=(dead_zone3*(-1))) )  
  {
    if (direct_pid3){
      PID3=(regP3+Motor->regI3+regD3)*(-1);
    }else{
      PID3=regP3+Motor->regI3+regD3;
    }
    PID3=constrain_(PID3,10000,0);
    Motor->PWM_out=(PID3/10)*Motor->curr_direction;  
  }
}



void PID_REG_V_only(Motor_Sruct *Motor){
  uint8_t dead_zone1=0, dead_zone2=0,direct_pid1=0,direct_pid2=0; 
  int64_t  Error1=0;
  int32_t regD1=0,regP1=0,PID1;
  int16_t  Error2;
  int32_t regD2=0,regP2=0,PID2;
  int32_t temp;
  //-------------------pid1------------------------------------------//   
#ifndef debug_PID_Velocity
  Error1=Motor->position_sp-Motor->position;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  constrain_(Error1,10000,-10000);
  regP1=Error1*Motor->P_position/10;               // position difference will determine rotation
  if (Motor->I_position){ 
    Motor->regI1+=(Error1*Motor->I_position)/100; //becouse time is 0.1  error*Ki*dt
  }else{
    Motor->regI1=0;
  }
  regD1=(Error1-Motor->Error_old1)*Motor->D_position*1; //becouse dt is 0.1
  regD1=constrain_(regD1,10000,-10000);
  Motor->regI1=constrain_(Motor->regI1,10000,-10000);
  Motor->Error_old1=Error1;
  if ((Error1>=dead_zone1)||(Error1<=(dead_zone1*(-1))) )  
  {
    
    if (direct_pid1){
      PID1=(regP1+Motor->regI1+regD1)*(-1);
    }else{
      PID1=regP1+Motor->regI1+regD1;
    }
    PID1=constrain_(PID1,10000,-10000);
    
    if (init_state!=2){
      temp=PID1*Motor->velocity_max/10000; //0-velocity_max
      Motor->velocity_sp=(int16_t)temp;
    }
  }
#endif
  if (Error1<0){ //CW or CCW according to position error
    Motor->curr_direction=-1;
  }else{
    Motor->curr_direction=1;
  }
  //-------------------pid2------------------------------------------//   
  Error2=Motor->velocity_sp-Motor->velocity_average;    
  
  regP2=(Error2*Motor->P_velocity)/10;  
  
  if (Motor->I_velocity){
    Motor->regI2+=(Error2*Motor->I_velocity)/100; //becouse time is 0.1  error*Ki*dt
  }else{
    Motor->regI2 =0; 
  }
  regD2=(Error2-Motor->Error_old2)*Motor->D_velocity*1; //becouse dt is 0.1
  regD2=constrain_(regD2,10000,-10000);
  Motor->regI2=constrain_(Motor->regI2,10000,-10000);
  Motor->Error_old2=Error2;
  if ((Error2>=dead_zone2)||(Error2<=(dead_zone2*(-1))) )  
  {
    
    if (direct_pid2){
      PID2=(regP2+Motor->regI2+regD2)*(-1);
    }else{
      PID2=regP2+Motor->regI2+regD2;
    }
    PID2=constrain_(PID2,10000,0);
    Motor->PWM_out=(PID2/10)*Motor->curr_direction;  
  }
  
}





void PID_REG_position_only(Motor_Sruct *Motor){
  uint8_t dead_zone1=0,direct_pid1=0; 
  int64_t  Error1=0;
  int32_t regD1=0,regP1=0,PID1;
  int32_t temp;
  //-------------------pid1------------------------------------------//   
  Error1=Motor->position_sp-Motor->position;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  constrain_(Error1,10000,-10000);
  regP1=Error1*Motor->P_position/10;               // position difference will determine rotation
  if (Motor->I_position){ 
    Motor->regI1+=(Error1*Motor->I_position)/100; //becouse time is 0.1  error*Ki*dt
  }else{
    Motor->regI1=0;
  }
  regD1=(Error1-Motor->Error_old1)*Motor->D_position*1; //becouse dt is 0.1
  regD1=constrain_(regD1,10000,-10000);
  Motor->regI1=constrain_(Motor->regI1,10000,-10000);
  Motor->Error_old1=Error1;
  if ((Error1>=dead_zone1)||(Error1<=(dead_zone1*(-1))) )  
  {
    
    if (direct_pid1){
      PID1=(regP1+Motor->regI1+regD1)*(-1);
    }else{
      PID1=regP1+Motor->regI1+regD1;
    }
    PID1=constrain_(PID1,10000,-10000);
    
    if (init_state!=2){
      temp=PID1*Motor->velocity_max/10000; //0-velocity_max
      Motor->velocity_sp=(int16_t)temp;
    }
  }
  if (Error1<0){ //CW or CCW according to position error
    Motor->curr_direction=-1;
  }else{
    Motor->curr_direction=1;
  }
  
  
}