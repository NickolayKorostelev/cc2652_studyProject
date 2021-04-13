#pragma once
extern "C"
{
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
}

class Kalman
{
private:
  
  float var_current = 2.0;
  float var_process = 1;
  float pc = 0.0;
  float gain = 0.0;
  float p =  25.0;
  float xp = 0.0;
  float zp = 0.0;
  float xe = 0.0;
  
  void KalmanUpdate(float value);
  
public:
  Kalman(){};
  ~Kalman(){};
  
  float getResult(float data);
  
};
