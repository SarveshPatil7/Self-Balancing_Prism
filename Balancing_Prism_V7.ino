#include <SimpleFOC.h>
#include <Wire.h>

#define MPU6050_ADDRESS 0x68
#define MT6701_ADDRESS  0x06
#define I2C_SDA 5  // SDA Pin for MPU6050
#define I2C_SCL 1  // SCL Pin for MPU6050
//#define ENC_SDA 5  // SDA Pin for MT6701
//#define ENC_SCL 1  // SCL Pin for MT6701 

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

volatile int16_t ax = 0, ay = 0, gz = 0;
int16_t Ax, Ay, Gz;

// BLDC motor & driver instance
MagneticSensorPWM sensor = MagneticSensorPWM(4, 2, 997);
void doPWM(){sensor.handlePWM();}
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 11, 12, 13);

float target_velocity = 0;
float acc_angle = 0;
float filt_acc_angle = 0;
float gyro_angle = 0;
float dt;
float tri_angle = 0;  //actual orientation of the robot
float angle_convert = 180.0 / PI;
float alpha = 0.15;  //filter for accelerometer
float beta = 0.02;  //filter for final angle
float omega;
unsigned long current_time;
unsigned long last_time = 0;

float base_12 = -57;
float base_23 = -179;
float base_31 = 57.2;
float bal_1 = 0;
float bal_2 = -118.2;
float bal_3 = 120.8;

// Variables for PID
float P_gain = 3, I_gain = 0, D_gain = 0;
float balancing_point = 0;
float last_error = 0;
float error = 0;
float integral = 0;
float derivative = 0;

// Mutex to protect shared access to target_velocity
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void read_MPU(void* pvParameters) {

  for (;;) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);  // Start at register 0x3B (Accel X)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);  // Read 14 bytes

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  // Protect shared variable
  portENTER_CRITICAL(&mux);
  ax = AcX;
  ay = AcY;
  gz = GyZ;  
  portEXIT_CRITICAL(&mux);
  vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void focLoop(void* pvParameters) {
  for (;;) {
    // Safely read shared velocity
    portENTER_CRITICAL(&mux);
    Ax = ax;
    Ay = ay;
    Gz = gz;
    portEXIT_CRITICAL(&mux);

    sensor.update();

    current_time = millis();
    dt = (current_time - last_time) / 1000.0f;   //seconds

    acc_angle = atan2(Ay, Ax) * angle_convert;
    filt_acc_angle = (alpha * acc_angle) + ((1 - alpha) * filt_acc_angle);  //complimentary filter to reduce noise

    //omega = Gz / 131;   // In °/s. Gyro is configured to 250°/s, LSB is 131
    //gyro_angle += gyro_rate * dt;
    //tri_angle = ((1 - beta) * (tri_angle + (omega * dt))) + (beta * acc_angle);   //complimentary filter to use gyroscope for primary angle calculation and use accelerometer to filter out drift

    //error = balancing_point - tri_angle;
    error = balancing_point - filt_acc_angle;

    if (abs(error) < 2) {
      motor.loopFOC();
      motor.move(0);
      last_time = current_time;
      continue;
    }
    else {
      integral += error * dt;
      integral = constrain(integral, -100, 100);

      if (dt > 0) {
        derivative = (error - last_error) / dt;
      }  
      else {
        derivative = 0;
      }
      last_error = error;

      target_velocity = (P_gain * error) + (I_gain * integral) + (D_gain * derivative);
      //target_velocity = constrain(target_velocity, -50, 50);

      motor.loopFOC();
      motor.move(target_velocity);
      last_time = current_time;
    }
  }
}

void setup() {
  //MPU setup
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Set to zero to wake up
  Wire.endTransmission(true);

  // Set accelerometer full-scale range to ±2g
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // 0x00 = ±2g (AFS_SEL = 0)
  Wire.endTransmission();

  //FOC setup
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);

  driver.voltage_power_supply = 12.8;
  driver.voltage_limit = 12.5;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  // motor controller parameters
  motor.PID_velocity.P = 0.15f;
  motor.PID_velocity.I = 0.05;
  motor.PID_velocity.D = 0.0003;

  motor.voltage_limit = 12;
  motor.PID_velocity.output_ramp = 300;
  motor.LPF_velocity.Tf = 2.5f;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;
  motor.useMonitoring(Serial);
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  delay(5000);  // To stabilize MPU

  last_time = millis();

  xTaskCreatePinnedToCore(read_MPU, "MPU6050", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(focLoop, "FOC", 4096, NULL, 2, NULL, 1);
}

void loop() {
  
}
