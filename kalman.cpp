#include <kalman.hpp>


void Kalman::KalmanUpdate(float data)
{
  pc = p + var_process;
  gain = pc/(pc + var_current);
  xp = xe;
  zp = xp;
  xe = gain*(data - zp) + xp;
}
 
float Kalman::getResult(float data)
{
   KalmanUpdate(data);
   return  xe;
}
