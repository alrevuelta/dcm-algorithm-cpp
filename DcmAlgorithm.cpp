// Includes
#include "DcmAlgorithm.h"
#include <math.h>

// Some macros
#define TO_RAD(x) (x * 0.01745329252)
#define TO_DEG(x) (x * 57.2957795131)
#define CONSTRAIN(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// Propirtional (P) and Integrator (I) terms for the PID controller.
#define ROLL_PITCH_KP 0.02f
#define ROLL_PITCH_KI 0.00002f
#define YAW_KP        1.2f
#define YAW_KI        0.00002f

DcmAlgorithm::DcmAlgorithm() {
  // Init the DCM matrix.The rotation is performed in ZYX order. Note that in this section
  // all Euler angles are assumed to be zero, so it will takes a few hundreds of ms to converge.
  // If you want quicker convergence, try to initiate the dcm matrix with the first read Euler angles.
  _dcm[0][0] = 1;  // cos(pitch) * cos(yaw)
  _dcm[0][1] = 0;  // cos(yaw)   * sin(roll) * sin(pitch) - cos(roll) * sin(yaw)
  _dcm[0][2] = 0;  // sin(roll)  * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)
  
  _dcm[1][0] = 0;  // cos(pitch) * sin(yaw)
  _dcm[1][1] = 1;  // cos(roll)  * cos(yaw) + sin(roll)*sin(pitch)*sin(yaw)
  _dcm[1][2] = 0;  // cos(roll)  * sin(pitch)*sin(yaw) - cos(yaw)*sin(roll)
  
  _dcm[2][0] = 0;  // -sin(pitch)
  _dcm[2][1] = 0;  //  cos(pitch) * sin(roll)
  _dcm[2][2] = 1;  //  cos(roll)  * cos(pitch)
  
  // Initiate the proportional and integrative term of the PID controller to zero
  _omegaProp[0] = 0;
  _omegaProp[1] = 0;
  _omegaProp[2] = 0;
  _omegaInt[0] = 0;
  _omegaInt[1] = 0;
  _omegaInt[2] = 0;
}

void DcmAlgorithm::update(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  
  float UpdateMatrix[3][3];
  float auxiliarMatrix[3][3];
  float accelVector[3];
  float gyrosVector[3];
  float errorRollPitch[3];
  float errorYaw[3];
  float scaledOmegaP[3];
  float scaledOmegaI[3];
  float omega[3];
  float heading;
  float errorCourse;
  float accelModule;
  float accelWeight;
  float error = 0;
  float renorm = 0;
  
  // Apply tilt compensation for the magnetometer and calculate the heading
  heading = atan2((-my * cos(_roll) + mz * sin(_roll)),
                    mx * cos(_pitch) + my * sin(_roll) * sin(_pitch) + mz * cos(_roll) * sin(_pitch));
  
  // Create a vector with the gyroscope measurements in radians
  gyrosVector[0]=TO_RAD(gx);
  gyrosVector[1]=TO_RAD(gy);
  gyrosVector[2]=TO_RAD(gz);
  
  // Create a vector with the accelerometer measurements
  accelVector[0]=ax;
  accelVector[1]=ay;
  accelVector[2]=az;
  
  // Add proportional and integrator term
  VectorAdd(omega, gyrosVector, _omegaInt);
  VectorAdd(omega, omega, _omegaProp);
  
  // Update the matrix with the delta t
  UpdateMatrix[0][0] = 0;
  UpdateMatrix[0][1] = -dt * omega[2]; //-z
  UpdateMatrix[0][2] =  dt * omega[1]; // y
  UpdateMatrix[1][0] =  dt * omega[2]; // z
  UpdateMatrix[1][1] = 0;
  UpdateMatrix[1][2] = -dt * omega[0]; //-x
  UpdateMatrix[2][0] = -dt * omega[1]; //-y
  UpdateMatrix[2][1] =  dt * omega[0]; // x
  UpdateMatrix[2][2] = 0;
  
  // Update the DCM matrix
  MatrixMultiply(_dcm, UpdateMatrix, auxiliarMatrix);
  for(int x = 0; x < 3; x++)
  {
    for(int y = 0; y < 3; y++)
    {
      _dcm[x][y] += auxiliarMatrix[x][y];
    } 
  }
  
  // Normalize and keep orthogonality
  error= -VectorDotProduct(&_dcm[0][0],&_dcm[1][0]) * .5;
  
  VectorScale(&auxiliarMatrix[0][0], &_dcm[1][0], error);
  VectorScale(&auxiliarMatrix[1][0], &_dcm[0][0], error);
  VectorAdd(&auxiliarMatrix[0][0], &auxiliarMatrix[0][0], &_dcm[0][0]);
  VectorAdd(&auxiliarMatrix[1][0], &auxiliarMatrix[1][0], &_dcm[1][0]);
  VectorCrossProduct(&auxiliarMatrix[2][0], &auxiliarMatrix[0][0], &auxiliarMatrix[1][0]);
  
  renorm = .5 * (3 - VectorDotProduct(&auxiliarMatrix[0][0], &auxiliarMatrix[0][0]));
  VectorScale(&_dcm[0][0], &auxiliarMatrix[0][0], renorm);
  
  renorm = .5 * (3 - VectorDotProduct(&auxiliarMatrix[1][0], &auxiliarMatrix[1][0]));
  VectorScale(&_dcm[1][0], &auxiliarMatrix[1][0], renorm);
  
  renorm = .5 * (3 - VectorDotProduct(&auxiliarMatrix[2][0], &auxiliarMatrix[2][0]));
  VectorScale(&_dcm[2][0], &auxiliarMatrix[2][0], renorm);
  
  // Compensate the drift for pitch and roll
  accelModule = sqrt(ax * ax + ay * ay + az * az);
  accelWeight = CONSTRAIN(1 - 2 * abs(1 - accelModule), 0, 1);
  
  // Update the proportional term
  VectorCrossProduct(errorRollPitch, accelVector, &_dcm[2][0]);
  VectorScale(_omegaProp, errorRollPitch, ROLL_PITCH_KP * accelWeight);
  
  // Update the integral term
  VectorScale(scaledOmegaI, errorRollPitch, ROLL_PITCH_KI * accelWeight);
  VectorAdd(_omegaInt, _omegaInt, scaledOmegaI);     
  
  // Compensate the drift for yaw
  errorCourse = (_dcm[0][0] * sin(heading)) - (_dcm[1][0] * cos(heading));
  VectorScale(errorYaw, &_dcm[2][0], errorCourse);
  
  // Update the proportional term
  VectorScale(scaledOmegaP, errorYaw, YAW_KP);
  VectorAdd(_omegaProp, _omegaProp, scaledOmegaP);
  
  // Update the integral term
  VectorScale(scaledOmegaI, errorYaw, YAW_KI);
  VectorAdd(_omegaInt, _omegaInt, scaledOmegaI);
  
  // Calculate Euler angles in radians
  _pitch = -asin(_dcm[2][0]);
  _roll  = atan2(_dcm[2][1], _dcm[2][2]);
  _yaw   = atan2(_dcm[1][0], _dcm[0][0]);
}

float DcmAlgorithm::VectorDotProduct(const float v1[3], const float v2[3])
{
  float result = 0;
  
  for(int c = 0; c < 3; c++)
  {
    result += v1[c] * v2[c];
  }
  
  return result; 
}

void DcmAlgorithm::VectorCrossProduct(float out[3], const float v1[3], const float v2[3])
{
  out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

void DcmAlgorithm::VectorScale(float out[3], const float v[3], float scale)
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
}

void DcmAlgorithm::VectorAdd(float out[3], const float v1[3], const float v2[3])
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

void DcmAlgorithm::MatrixMultiply(const float a[3][3], const float b[3][3], float out[3][3])
{
  for(int x = 0; x < 3; x++)
  {
    for(int y = 0; y < 3; y++)
    {
      out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
    }
  }
}

void DcmAlgorithm::MatrixVectorMultiply(const float a[3][3], const float b[3], float out[3])
{
  for(int x = 0; x < 3; x++)
  {
    out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
  }
}

float DcmAlgorithm::getRoll() {
  return TO_DEG(_roll);
}

float DcmAlgorithm::getPitch() {
  return TO_DEG(_pitch);
}

float DcmAlgorithm::getYaw() {
  return TO_DEG(_yaw) + 180.0f;
}
