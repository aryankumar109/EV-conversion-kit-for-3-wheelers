#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
// #include <imuFilter.h>
// #include <accIntegral.h>

#define THROTTLE A1
#define OUTPUT_P 9
float nu = 0.015;
float m = 450;
float gravity = 9.81;
float r = 0.16;
float pho = 1.2;
float Cd = 0.4792;
float A = 2.09;
float a_max ;
float t_max;
float dp = 0.1;
float dt = 40;

float v_x  = 0;
float v_y  = 0; 
float v_z = 0;

float theta_x = 0; 
float theta_y = 0;
float theta_z = 0;

// Filter coefficients                       //  Unit           
// constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
// constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
// constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
// constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.

// accIntegral fusion;
Adafruit_MPU6050 mpu;

double tilt_angle(double yaw, double pitch, double roll) {
  // """
  // Calculates the tilt angle of a robot based on yaw, pitch, and roll angles (in degrees).

  // Args:
  //     yaw: Yaw angle (in degrees).
  //     pitch: Pitch angle (in degrees).
  //     roll: Roll angle (in degrees).

  // Returns:
  //     The tilt angle of the robot (in degrees).
  // """

  // Convert angles to radians
  double yaw_rad = M_PI * yaw / 180.0;
  double pitch_rad = M_PI * pitch / 180.0;
  double roll_rad = M_PI * roll / 180.0;

  // Define rotation matrices for yaw, pitch, and roll
  double C_yaw[3][3] = {{cos(yaw_rad), -sin(yaw_rad), 0},
                        {sin(yaw_rad),  cos(yaw_rad), 0},
                        {0,                 0,                 1}};

  double C_pitch[3][3] = {{cos(pitch_rad), 0, sin(pitch_rad)},
                          {0,                 1,                 0},
                          {-sin(pitch_rad), 0, cos(pitch_rad)}};

  double C_roll[3][3] = {{1,                0, -sin(roll_rad)},
                         {0,                 cos(roll_rad), 0},
                         {0,                 sin(roll_rad),  cos(roll_rad)}};

  // Combine rotations (ZYX convention - Yaw, then Pitch, then Roll)
  double R[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R[i][j] = 0.0;
      for (int k = 0; k < 3; k++) {
        R[i][j] += C_yaw[i][k] * C_pitch[k][j] * C_roll[j][k];
      }
    }
  }

  // Extract element corresponding to Z-axis in ground frame (assuming ZYX)
  double z_axis = R[2][2];

  // Calculate tilt angle using arctangent
  double tilt_angle_rad = atan2(sqrt(1 - z_axis * z_axis), z_axis);

  // Convert back to degrees
  double tilt_angle_deg = tilt_angle_rad * 180.0 / M_PI;

  return tilt_angle_deg;
}

void setup(void) {
  Serial.begin(115200);
  pinMode(THROTTLE, INPUT);
  pinMode(OUTPUT_P, OUTPUT);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // fusion.setup();
  // imuFilter filter = imuFilter(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
  
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float a_req = a_max * ((float)analogRead(THROTTLE) / 1023.0) * (5.0 / 4.0);
  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
  Serial.println("");

  // vec3_t accel = { a.acceleration.x, a.acceleration.y, a.acceleration.z };    // g-unit
  // vec3_t gyro = { g.gyro.x, g.gyro.y, g.gyro.z };     // radians/second
 
  // // Update heading and velocity estimate:
  
  //     // known measured velocity (target state). Estimate will be forced towards this vector
  // // vec3_t vel_t = {0,0,0};

  // // vel_t /= GRAVITY;                         // must have unit: g-force * second
  
  //     /* note: all coefficients are optional and have default values */
  // fusion.update( gyro, accel);//, vel_t, SD_ACC, SD_VEL, ALPHA ); 

      // obtain velocity estimate
  // vec3_t vel = fusion.getVel() * GRAVITY;   // scale by gravity for desired units
  // // Get roll, pitch, and yaw angles (in degrees) from the filter
  // Serial.print( vel.x, 2 );
  // Serial.print( " " );
  // Serial.print( vel.y, 2 );
  // Serial.print( " " );
  // Serial.print( vel.z, 2 );  
  // Serial.println();

  // filter.update();

  float theta = tilt_angle(theta_z, theta_y, theta_x);
  float linear_acceleration_x = a.acceleration.x - gravity * sin(theta) * sin(g.gyro.z);
  float linear_acceleration_y = a.acceleration.y - gravity * sin(theta) * cos(g.gyro.z);
  float linear_acceleration_z = a.acceleration.z - gravity * cos(theta);

  float f_tractive = (nu * m * gravity * cos(theta)) + (0.5 * pho * A * Cd * (pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2))) + (m * a_req) + (m * gravity * sin(theta) * cos(theta_z)); // F_trac = F_rolling + F_air + F_grad + F_inertial
  float torque = f_tractive / r;
  Serial.println();
  Serial.println(torque);
  // float power = map(torque, 0, t_max, 0, 255);
  // analogWrite(OUTPUT_P, power);

  theta_x += g.gyro.x * dt;
  theta_y += g.gyro.y * dt;
  theta_z += g.gyro.z * dt;

  v_x += linear_acceleration_x * dt;
  v_y += linear_acceleration_y * dt;
  v_z += linear_acceleration_z * dt;

  
  delay(dt);

}