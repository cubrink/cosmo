// Libraries for Accelerometer
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//mpu.getEvent(&a, &g, &temp);e
// Accelerometer Initialization and Constants

Adafruit_MPU6050 mpu;
const float J2_X_THRESH = 40; // Squared jerk, Up and Down movement
const float J2_Y_THRESH = 25; // Squared jerk, Back and forth movement
const int JERK_COOLDOWN = 3;

// acceleration values for jerk calculation
float x_curr, y_curr, z_curr;
float x_prev, y_prev, z_prev;


double x = 0;

float roll, pitch;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Arduino Started");

  /*
   * Accelerometer Setup
   */
  if (!mpu.begin()) 
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("mpu initialized");
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  roll = atan2(a.acceleration.y, a.acceleration.z);
  Serial.println(roll); 
//  x_curr = a.acceleration.x / 256;
//  y_curr = a.acceleration.y / 256;
//  z_curr = a.acceleration.z / 256;
//  Serial.print("Acceleration X:");
//  Serial.print(x_curr);
//  Serial.print("Acceleration y:");
//  Serial.print(y_curr);
//  Serial.print("Acceleration Z:");
//  Serial.print(z_curr);
//  Serial.println("");



//  pitch = atan(y_curr / sqrt(pow(x_curr, 2) + pow(z_curr, 2))) * 180 / PI;
//  roll = atan(-1 * x_curr / sqrt(pow(y_curr, 2) + pow(z_curr, 2))) * 180 / PI;
//  Serial.print(roll);
//  Serial.print("\t");
//  Serial.println(pitch);
  delay(100);
}
