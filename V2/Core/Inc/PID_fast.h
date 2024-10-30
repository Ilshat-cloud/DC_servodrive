
#ifdef __cplusplus
 extern "C" {
#endif

   
   void PID_REG(Motor_Sruct *Motor);
#define constrain_(a, max, min) a >= max ? max : (a <= min ?min:a)  
   
#ifdef __cplusplus
}
#endif

