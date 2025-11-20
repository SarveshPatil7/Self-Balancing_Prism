#include <SimpleFOC.h>
#include <ESP32_SoftWire.h>

SoftWire i2c;

#define MPU6050_ADDRESS 0x68
#define MT6701_ADDRESS  0x06
#define I2C_SDA 5  // SDA Pin for MPU6050
#define I2C_SCL 1  // SCL Pin for MPU6050
//#define ENC_SDA 35  // SDA Pin for MT6701
//#define ENC_SCL 36  // SCL Pin for MT6701 
#define RGB_BUILTIN 48

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;
int16_t t;

volatile int16_t Ax = 0, Ay = 0, Gz = 0;
int16_t ax, ay, gz;

// BLDC motor & driver instance
MagneticSensorPWM sensor = MagneticSensorPWM(4, 2, 997);
void doPWM(){sensor.handlePWM();}
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 11, 12, 13);

float target_velocity = 0;
float filt_ax;
float filt_ay;
float acc_angle = 0;
float gyro_angle = 0;
float tri_angle = 0;  //actual orientation of the robot
float alpha = 0.2;    //filter for accelerometer
float beta = 0.998;   //filter for final angle  
float omega;
float dt;
unsigned long current_time;
unsigned long last_time = 0;
float gyro_offset;
int gz_sum = 0;

// Equillibrium points for the device, not used for now
//float base_12 = -57;
//float base_23 = -179;
//float base_31 = 57.2;
//float bal_1 = 0;
//float bal_2 = -118.2;
//float bal_3 = 120.8;

// Variables for PID
float P_gain = 3.0, I_gain = 0.4, D_gain = 0.001, O_gain = 0.4;   // O_gain is for state observer to avoid saturating the motor speed
float balancing_point = 0;
float last_error = 0;
float error = 0;
float integral = 0;
float derivative = 0;

// Mutex to protect shared access to target_velocity
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Free RTOS task to read the data from MPU6050, 100
void read_MPU(void* pvParameters) {
  TickType_t lastWakeMPU = xTaskGetTickCount();
  for (;;) {
    i2c.beginTransmission(MPU6050_ADDRESS);
    i2c.write(0x3B);  // Accel X high byte
    i2c.endTransmission(false);
    i2c.requestFrom(MPU6050_ADDRESS, 4, true);  // Accel X/Y (4 bytes)

    if (i2c.available() == 4) {
      AcX = (i2c.read() << 8) | i2c.read();  
      AcY = (i2c.read() << 8) | i2c.read();  
    }

    // --- Read only Gyro Z ---
    i2c.beginTransmission(MPU6050_ADDRESS);
    i2c.write(0x47);  // Gyro Z high byte
    i2c.endTransmission(false);
    i2c.requestFrom(MPU6050_ADDRESS, 2, true);  // Gyro Z (2 bytes)

    if (i2c.available() == 2) {
      GyZ = (i2c.read() << 8) | i2c.read(); 
    }

  // Protect shared variable
  portENTER_CRITICAL(&mux);
  Ax = AcX;
  Ay = AcY;
  Gz = GyZ - gyro_offset;  
  portEXIT_CRITICAL(&mux);

  // 200Hz polling rate, to avoid excessive codeblocking from i2c callouts
  vTaskDelayUntil(&lastWakeMPU, pdMS_TO_TICKS(5));
  }
}

void focLoop(void* pvParameters) {
  TickType_t lastWakeFOC = xTaskGetTickCount();
  for (;;) {
    // Safely read shared velocity
    portENTER_CRITICAL(&mux);
    ax = Ax;
    ay = Ay;
    gz = -Gz;     //direction correction
    portEXIT_CRITICAL(&mux);

    sensor.update();

    filt_ax = (alpha * ax) + ((1 - alpha) * filt_ax);
    filt_ay = (alpha * ay) + ((1 - alpha) * filt_ay);
    acc_angle = atan2f(filt_ay, filt_ax) * RAD_TO_DEG;

    omega = gz / 131;   // In °/s. Gyro is configured to 250°/s, LSB is 131

    current_time = millis();
    dt = (current_time - last_time) / 1000.0f;   //seconds
    //gyro_angle += omega * dt;

    tri_angle = (beta * (tri_angle + (omega * dt))) + ((1 - beta) * acc_angle);   // complimentary filter to use gyroscope for primary angle calculation and use accelerometer for drift comepnsation

    float wheel_vel = motor.shaftVelocity();

    error = balancing_point - tri_angle;

    motor.loopFOC();

    if (abs(error) > 10) {
      target_velocity = 0;
      integral = 0;
    }
    else {
      integral += error * dt;
      integral = constrain(integral, -20, 20);

      derivative = omega;

      // Calualating the target voltage (veloctiy) using PID and state observer feedback 
      target_velocity = (P_gain * error) + (I_gain * integral) + (D_gain * derivative) + (O_gain * wheel_vel);

      //target_velocity = constrain(target_velocity, -50, 50);
    }

    motor.move(target_velocity);
    last_time = current_time;

    // 1000Hz for smooth operation of FOC algorithm
    vTaskDelayUntil(&lastWakeFOC, pdMS_TO_TICKS(1));
  }
}

// Function to calibrate the gyro 
void cal() {
  const int samples = 1000;
  float acc_angle_sum = 0;

  neopixelWrite(RGB_BUILTIN, 0, 0, 30);

  delay(60000);   // Letting the gyro settle

  neopixelWrite(RGB_BUILTIN, 30, 30, 30);

  for (int i = 0; i < samples; i++){
    // Read MPU6050 data
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    t = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    // Calculate angle in degrees
    filt_ax = (alpha * AcX) + ((1 - alpha) * filt_ax);
    filt_ay = (alpha * AcY) + ((1 - alpha) * filt_ay);
    acc_angle = atan2f(filt_ay, filt_ax) * RAD_TO_DEG;

    acc_angle_sum += acc_angle;

    gz_sum += GyZ;
      
    delay(2);
  }

  gyro_angle = acc_angle_sum / samples;
  gyro_offset = gz_sum / samples;

  neopixelWrite(RGB_BUILTIN, 0, 40, 0);
  delay(2000);
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);
}

void setup() {
  //MPU setup
  i2c.begin(I2C_SDA, I2C_SCL, 400000);
  //Wire.setClock(400000); 

  // Wake up MPU6050
  i2c.beginTransmission(MPU6050_ADDRESS);
  i2c.write(0x6B);         // Power management register
  i2c.write(0);            // Set to zero to wake up
  i2c.endTransmission(true);

  // Set accelerometer full-scale range to ±2g
  i2c.beginTransmission(MPU6050_ADDRESS);
  i2c.write(0x1C);         // ACCEL_CONFIG register
  i2c.write(0x00);         // 0x00 = ±2g
  i2c.endTransmission();

  // Set gyro full-scale range to ±250 dps
  i2c.beginTransmission(0x68);
  i2c.write(0x1B);          // GYRO_CONFIG register
  i2c.write(0x00);          // FS_SEL=0 (±250 dps)
  i2c.endTransmission();

  //FOC setup
  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  sensor.enableInterrupt(doPWM);

  driver.voltage_power_supply = 12.8;
  driver.voltage_limit = 12.5;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);

  // motor controller parameters
  motor.PID_velocity.P = 0.7f;
  motor.PID_velocity.I = 0.9f;
  motor.PID_velocity.D = 0.0003f;

  motor.voltage_limit = 12;
  motor.PID_velocity.output_ramp = 300;
  motor.LPF_velocity.Tf = 0.1f;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();

  pinMode(RGB_BUILTIN, OUTPUT);

  cal();

  last_time = millis();

  // Pinning the tasks to separate cores using Free RTOS
  xTaskCreatePinnedToCore(read_MPU, "MPU6050", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(focLoop, "FOC", 4096, NULL, 2, NULL, 0);
}

void loop() {
  
}
