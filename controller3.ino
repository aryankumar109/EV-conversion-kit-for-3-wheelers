#include <basicMPU6050.h>   // Library for IMU sensor. See this link: https://github.com/RCmags/basicMPU6050
#include <accIntegral.h>

// ========== IMU sensor ==========

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET = 0;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = 0;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = 0;

//-- Set template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET
            >imu;

// =========== Settings ===========
accIntegral fusion;

// Filter coefficients                       //  Unit           
constexpr float GRAVITY = 9.81e3;            //  mm/s^2             Magnitude of gravity at rest. Determines units of velocity. [UNITS MUST MATCH ACCELERATION]
constexpr float SD_ACC  = 1000 / GRAVITY;    //  mm/s^2 / g-force   Standard deviation of acceleration. Deviations from zero are suppressed.
constexpr float SD_VEL  = 200  / GRAVITY;    //  mm/s   / g-force   Standard deviation of velocity. Deviations from target value are suppressed.
constexpr float ALPHA   = 0.5;               //                     Gain of heading update - See example "output" for more information.

#define OUTPUT_P 9
float nu = 0.015;
float m = 450;
float gravity = 9.81;
float r = 0.16;
float pho = 1.2;
float Cd = 0.4792;
float A = 2.09;
float a_max = 10;
float t_max = 1300; //1000
float dp = 0.1;
float dt = 40;
float a_threshold = 6;
float min_power = 150;

double tilt_angle(double yaw_rad, double pitch_rad, double roll_rad) {
  // """
  // Calculates the tilt angle of a robot based on yaw, pitch, and roll angles (in degrees).

  // Args:
  //     yaw: Yaw angle (in degrees).
  //     pitch: Pitch angle (in degrees).
  //     roll: Roll angle (in degrees).

  // Returns:
  //     The tilt angle of the robot (in degrees).
  // """

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

  return tilt_angle_rad;
}


void setup() {
  Serial.begin(38400);
  delay(1000);

  pinMode(OUTPUT_P, OUTPUT);

  // calibrate IMU sensor
  imu.setup();
  imu.setBias();

  // initialize sensor fusion
  //fusion.setup( imu.ax(), imu.ay(), imu.az() );   // ALWAYS set initial heading with acceleration 
  fusion.setup();
  
  //fusion.reset();  /* Use this function to zero velocity estimate */                               
}

void loop() {
  
  imu.updateBias(); 
  float a_req = a_max; // Set the desired acceleration to the maximum

  // Measure state:  
  vec3_t accel = { imu.ax(), imu.ay(), imu.az() };    // g-unit
  vec3_t gyro = { imu.gx(), imu.gy(), imu.gz() };     // radians/second
 
  // Update heading and velocity estimate:
  
  // known measured velocity (target state). Estimate will be forced towards this vector
  vec3_t vel_t = {0,0,0};

  vel_t /= GRAVITY;                         // must have unit: g-force * second
  
  /* note: all coefficients are optional and have default values */
  fusion.update( gyro, accel, vel_t, SD_ACC, SD_VEL, ALPHA ); 

  // obtain velocity estimate
  vec3_t vel = fusion.getVel() * GRAVITY/float(1e3);   // scale by gravity for desired units
  float theta_z = fusion.yaw();
  float theta_y = fusion.pitch();
  float theta_x = fusion.roll();

  float theta = tilt_angle(theta_z, theta_y, theta_x);
  if (theta_x < 0) theta *= -1; 
  float f_tractive = (nu * m * gravity * cos(theta)) + (0.5 * pho * A * Cd * (pow(vel.x, 2) + pow(vel.y, 2) + pow(vel.z, 2))) + (m * a_req) + (m * gravity * sin(theta) * cos(theta_z)); // F_trac = F_rolling + F_air + F_grad + F_inertial
  float torque = f_tractive * r;
  float power = map(torque, 0, t_max, 0, 255);

  if (power < 255) {
    analogWrite(OUTPUT_P, power);
    if (power < min_power && a_req > a_threshold) {
      analogWrite(OUTPUT_P, min_power);
      power = min_power;
    }
  } else {
    analogWrite(OUTPUT_P, 255);
    power = 255;
  }
  // Serial.println(torque);
  // Serial.println(a_req);
  Serial.print("Theta:");
  Serial.print(theta * 180 / PI);
  Serial.print(",");
  Serial.print("A_req:");
  Serial.println(a_req);
  Serial.print(",");
  Serial.print("torque:");
  Serial.println(torque);
  Serial.print(",");
  Serial.print("Power:");
  Serial.println(power);//fg

  delay(dt);

}
