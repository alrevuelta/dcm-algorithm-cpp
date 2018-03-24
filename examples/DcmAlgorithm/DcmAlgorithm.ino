// Includes
#include "DcmAlgorithm.h"

// Global variable definition
DcmAlgorithm dcmAlgorithm;

// Time control
unsigned long timestamp;
unsigned long timestamp_old;
float deltat;

void setup()
{
  /*
   * Note that this library only provides the algorithm to perform the Direction
   * Cosine Matrix calculations. This algorithm estimates with drift correction
   * the pitch, roll and yaw (also known as Euler Angles).
   * It has to be feed with data read from a gyroscope, magnetometer and accelerometer,
   * with the following units:
   *    -Gyroscopes:    Deg/s
   *    -Magnetometer:  Gauss
   *    -Accelerometer: G
   * This library aims to be generic and not sensor dependant, so just use the sensor that you
   * which, read the specified data from it, and feed the algorithm with it.
   */
}

void loop()
{
  // Calcualte time since last update
  timestamp_old = timestamp;
  timestamp = micros();
  if (timestamp > timestamp_old)
    deltat = (float) (timestamp - timestamp_old) / 1000000.0f;
  else deltat = 0;

  dcmAlgorithm.update(deltat, gyrCalib[0], gyrCalib[1], gyrCalib[2],
                              accCalib[0], accCalib[1], accCalib[2],
                              magCalib[0], magCalib[1], magCalib[2]);

  sendYPR();
}

//todo:
void sendYPR() {
  String deltat = String(deltat, 5);
  String yaw_s = String(dcmAlgorithm.getYaw(), 2);
  String pitch_s = String(dcmAlgorithm.getPitch(), 2);
  String roll_s = String(dcmAlgorithm.getRoll(), 2);

  String message = String("#TYPR=" + deltat + "," + yaw_s + "," + pitch_s + "," + roll_s);

  Serial.println(message);
}
