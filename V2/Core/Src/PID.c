
#include "main.h"
#include "PID.h"
#include "stm32f1xx_hal.h"
#include "string.h"
extern uint8_t init_state;


/* USER CODE END EV */


void PID_REG(Motor_Sruct *Motor){
  static uint8_t dead_zone1=0, dead_zone2=0,dead_zone3=0,direct_pid1=0,direct_pid2=0,direct_pid3=0; 
  static int16_t errorsold1[200]={0};
  static int16_t  Error1;
  static int32_t regD1=0,regP1=0,regI1=0,PID1;
  static int16_t errorsold2[200]={0};
  static int16_t  Error2;
  static int32_t regD2=0,regP2=0,regI2=0,PID2;
  static int16_t errorsold3[200]={0};
  static int16_t  Error3;
  static int32_t regD3=0,regP3=0,regI3=0,PID3;
  uint8_t iter;
  //-------------------pid1------------------------------------------//   
  Error1=Motor->position_sp-Motor->position;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  
  regP1=Error1*Motor->P_position;
  if (Motor->I_position){
    regI1=0;
    for (iter=0; iter<199; iter++)  
    {
      regI1=regI1+errorsold1[iter];
    } 
    regI1=(Error1+regI1)/100; //becouse time is 0.01 and buff 200 that means integral is for last 2 secound
    regI1 =regI1*Motor->I_position; 
//    for (iter=198; iter>0; iter--)
//    {
//      errorsold1[iter+1]=errorsold1[iter];
//    } 
    memcpy(errorsold1,errorsold1+2,sizeof(errorsold1)-2);
  }else{
    regI1=0;
  }
  regD1=(Error1-errorsold1[0])*Motor->D_position*100; //becouse dt is 0.01
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
  
  errorsold1[0]=Error1;
  if ((Error1>=dead_zone1)||(dead_zone1==0)||(Error1<=(dead_zone1*(-1))) )  
  {
    
    if (direct_pid1==0){
      PID1=regP1+regI1+regD1;
    }else{
      PID1=(regP1+regI1+regD1)*(-1);
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
      Motor->velocity_sp=PID1*Motor->velocity_max/10000; //0-velocity_max
    }
  }
  //-------------------pid2------------------------------------------//   
  Error2=Motor->velocity_sp-Motor->velocity;    //+-MAX_INT32 zadanie +-MAX_INT64 feedback
  
  regP2=Error2*Motor->P_velocity;  
  
  if (Motor->I_velocity){
    regI2=0;
    for (iter=0; iter<199; iter++)  
    {
      regI2=regI2+errorsold2[iter];
    } 
    regI2=(Error2+regI2)/100; //becouse time is 0.01 and buff 200 that means integral is for last 2 secound
    regI2 =regI2*Motor->I_velocity; 
    
//    for (iter=198; iter>0; iter--)
//    {
//      errorsold2[iter+1]=errorsold2[iter];
//    } 
    memcpy(errorsold2,errorsold2+2,sizeof(errorsold2)-2);
  }else{
    regI2 =0; 
  }
  regD2=(Error2-errorsold2[0])*Motor->D_velocity*100; //becouse dt is 0.01
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
  
  errorsold2[0]=Error2;
  if ((Error2>=dead_zone2)||(dead_zone2==0)||(Error2<=(dead_zone2*(-1))) )  
  {
    
    if (direct_pid2==0){
      PID2=regP2+regI2+regD2;
    }else{
      PID2=(regP2+regI2+regD2)*(-1);
    }
    if (PID2<=-10000)
    {
      PID2=-10000;
    }
    if (PID2>=10000)
    {
      PID2=10000;
    }
    Motor->I_M_sp=Motor->I_M_max*PID2/10000;
    //-------------------pid3------------------------------------------//   
    Error3=Motor->I_M_sp-Motor->I_M*Motor->curr_direction;    //+-MAX_INT32 zadanie +-MAX_UINT16 feedback
    regP3=Error3*Motor->P_current; 
    if (Motor->I_current){
      regI3=0;
      for (iter=0; iter<199; iter++)  
      {
        regI3=regI3+errorsold3[iter];
      } 
      
      regI3=(Error3+regI3)/100; //becouse time is 0.01 and buff 200 that means integral is for last 2 secound
      regI3 =regI3*Motor->I_current; 
//      for (iter=198; iter>0; iter--)
//      {
//        errorsold3[iter+1]=errorsold3[iter];
//      } 
      memcpy(errorsold3,errorsold3+2,sizeof(errorsold3)-2);
    }else{
      regI3 =0; 
    }
    regD3=(Error3-errorsold3[0])*Motor->D_current*100; //becouse dt is 0.01
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
    
    errorsold3[0]=Error3;
    if ((Error3>=dead_zone3)||(dead_zone3==0)||(Error3<=(dead_zone3*(-1))) )  
    {
      
      if (direct_pid3==0){
        PID3=regP3+regI3+regD3;
      }else{
        PID3=(regP3+regI3+regD3)*(-1);
      }
      if (PID3<=-10000)
      {
        PID3=-10000;
      }
      if (PID3>=10000)
      {
        PID3=10000;
      }
      Motor->PWM_out=PID3/10;  //temporary
      
    }
    
    
  }
}