
#include "main.h"
#include "PID_fast.h"
#include "stm32f1xx_hal.h"
extern uint8_t init_state;


/* USER CODE END EV */


void PID_REG(Motor_Sruct *Motor){
  static uint8_t dead_zone1=0, dead_zone2=0,dead_zone3=0,direct_pid1=0,direct_pid2=0,direct_pid3=0; 
  static int16_t  Error1,Error_old1=0;
  static int32_t regD1=0,regP1=0,regI1=0,PID1;
  static int16_t  Error2,Error_old2=0;
  static int32_t regD2=0,regP2=0,regI2=0,PID2;
  static int16_t  Error3,Error_old3=0;
  static int32_t regD3=0,regP3=0,regI3=0,PID3;
  int32_t temp;
  //-------------------pid1------------------------------------------//   
  Error1=Motor->position_sp-Motor->position;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  regP1=Error1*Motor->P_position;
  if (Motor->I_position){ 
    regI1+=(Error1*Motor->I_position)/100; //becouse time is 0.01  error*Ki*dt
  }else{
    regI1=0;
  }
  regD1=(Error1-Error_old1)*Motor->D_position*100; //becouse dt is 0.01
  if (regD1<(-10000))
  {
    regD1=(-10000);
  }
  if (regD1>(10000))
  {
    regD1=(10000);
  }
  if (regI1<(-10000))
  {
    regI1=(-10000);
  }
  if (regI1>(10000))
  {
    regI1=(10000);
  }
  Error_old1=Error1;
  if ((Error1>=dead_zone1)||(Error1<=(dead_zone1*(-1))) )  
  {
    
    if (direct_pid1){
      PID1=(regP1+regI1+regD1)*(-1);
    }else{
      PID1=regP1+regI1+regD1;
    }
    if (PID1<=-10000)
    {
      PID1=-10000;
    }
    if (PID1>=10000)
    {
      PID1=10000;
    }
    
    if ((init_state!=2)&&(init_state!=0)){
      temp=PID1*Motor->velocity_max/10000; //0-velocity_max
      Motor->velocity_sp=(int16_t)temp;
    }
  }
  //-------------------pid2------------------------------------------//   
  Error2=Motor->velocity_sp-Motor->velocity;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  
  regP2=Error2*Motor->P_velocity;  
  
  if (Motor->I_velocity){
    regI2+=(Error2*Motor->I_velocity)/100; //becouse time is 0.01  error*Ki*dt
  }else{
    regI2 =0; 
  }
  regD2=(Error2-Error_old2)*Motor->D_velocity*100; //becouse dt is 0.01
  if (regD2<(-10000))
  {
    regD2=(-10000);
  }
  if (regD2>(10000))
  {
    regD2=(10000);
  }
  if (regI2<(-10000))
  {
    regI2=(-10000);
  }
  if (regI2>(10000))
  {
    regI2=(10000);
  }
  Error_old2=Error2;
  if ((Error2>=dead_zone2)||(Error2<=(dead_zone2*(-1))) )  
  {
    
    if (direct_pid2){
      PID2=(regP2+regI2+regD2)*(-1);
    }else{
      PID2=regP2+regI2+regD2;
    }
    if (PID2<=-10000)
    {
      PID2=-10000;
    }
    if (PID2>=10000)
    {
      PID2=10000;
    }
    temp=Motor->I_M_max*PID2/10000;
    Motor->I_M_sp=(int16_t)temp;
  }
    
  //-------------------pid3------------------------------------------//   
  Error3=Motor->I_M_sp-Motor->I_M*Motor->curr_direction;    //+-MAX_INT32 zadanie +-MAX_UINT16 feedback
  regP3=Error3*Motor->P_current; 
  if (Motor->I_current){
    regI3+=(Error3*Motor->I_current)/100; //becouse time is 0.01  error*Ki*dt
  }else{
    regI3 =0; 
  }
  regD3=(Error3-Error_old3)*Motor->D_current*100; //becouse dt is 0.01
  if (regD3<(-10000))
  {
    regD3=(-10000);
  }
  if (regD3>(10000))
  {
    regD3=(10000);
  }
  if (regI3<(-10000))
  {
    regI3=(-10000);
  }
  if (regI3>(10000))
  {
    regI3=(10000);
  }
  Error_old3=Error3;
  if ((Error3>=dead_zone3)||(Error3<=(dead_zone3*(-1))) )  
  {
    if (direct_pid3){
      PID3=(regP3+regI3+regD3)*(-1);
    }else{
      PID3=regP3+regI3+regD3;
    }
    if (PID3<=-10000)
    {
      PID3=-10000;
    }
    if (PID3>=10000)
    {
      PID3=10000;
    }
    Motor->PWM_out=PID3/10;  
  }
}