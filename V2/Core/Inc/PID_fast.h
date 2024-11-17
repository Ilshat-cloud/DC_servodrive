
#ifdef __cplusplus
 extern "C" {
#endif
   
   
   void PID_REG(Motor_Sruct *Motor);
   void PID_REG_V_only(Motor_Sruct *Motor);
   void PID_REG_position_only(Motor_Sruct *Motor);
   
#define constrain_(amt,high,low) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))  
     
#ifdef __cplusplus
}
#endif

