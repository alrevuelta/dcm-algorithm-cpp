#ifndef DcmAlgorithm_h
#define DcmAlgorithm_h
#include <math.h>
#include <stdlib.h>

class DcmAlgorithm{
public:
  DcmAlgorithm(void);
  void update(float, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  float getRoll();
  float getPitch();
  float getYaw();
  
private:
  float _roll;
  float _pitch;
  float _yaw;
  
  float _dcm[3][3];
  float _omegaProp[3];
  float _omegaInt[3];
  
  void MatrixVectorMultiply(const float a[3][3], const float b[3], float out[3]);
  void MatrixMultiply(const float a[3][3], const float b[3][3], float out[3][3]);
  void VectorAdd(float out[3], const float v1[3], const float v2[3]);
  void VectorScale(float out[3], const float v[3], float scale);
  void VectorCrossProduct(float out[3], const float v1[3], const float v2[3]);
  float VectorDotProduct(const float v1[3], const float v2[3]);
};
#endif

