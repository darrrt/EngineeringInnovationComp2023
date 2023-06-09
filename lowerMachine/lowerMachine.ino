#include <Stream.h>
// #define OUTPUT_LOG 1
// String inString[128];
// short readinLen=0;
#include <Arduino.h>
#include <TMCStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// our servo # counter
// uint8_t servonum = 0;
char comchar;
bool shaft = false;
// 默认地址 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// #define SERVOMIN 150 // this is the 'minimum' pulse length count (out of 4096)
// // 这是“最小”脉冲长度计数（在4096）中
// #define SERVOMAX 600 // this is the 'maximum' pulse length count (out of 4096)
// 这是“最大”脉冲长度计数（在4096中）

// #define Servo0 30
// #define Servo1 31
// #define Servo2 32
#define EN_PIN 32   //   
#define DIR_PIN 36  // Direction
#define STEP_PIN 38 // Step
// #define CS_PIN           42 // Chip select
// #define SW_MOSI          66 // Software Master Out Slave In (MOSI)
// #define SW_MISO          44 // Software Master In Slave Out (MISO)
// #define SW_SCK           64 // Software Slave Clock (SCK)
// #define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
// #define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
// TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
// TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
// TMC2660Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
// TMC2660Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
// TMC5160Stepper driver(CS_PIN, R_SENSE);
// TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware Serial
// TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software serial
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
// TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
//  Servo Servo0,Servo1,Servo2;
//  void ServoTurn(int pos, Servo myServo){
//    int i=0;
//    for (i=0;i<=pos;i++)
//    {
//      myServo.write(i);
//      delay(10);
//    }
//  }
void StepperControl(unsigned int times, bool shaft = 1)
{
  // shaft = !shaft;
  // driver.shaft(shaft);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware
  digitalWrite(DIR_PIN, shaft);
  
  // Serial.print(shaft);
  for (; times > 0; times--)
  {
    for (uint16_t i = 200; i > 0; i--)
    {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(200);
      // Serial.println("HIGH");
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(200);
      // Serial.println("LOW");
    }
  }
  digitalWrite(EN_PIN, HIGH); // Enable driver in hardware
}

#include <SunConfig.h>
#include <Version.h>

// 走矩形线路测试
// 运行前先校准陀螺仪
//  20210406更新 使用枚举类型 imu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <FlexiTimer2.h> //定时中断
#include "SunConfig.h"

// 串口总线舵机配置参数
// #define BAUDRATE 115200 // 波特率

// 调试串口的配置
#if defined(ARDUINO_AVR_UNO)
#include <SoftwareSerial.h>
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
#define DEBUG_SERIAL softSerial
#define DEBUG_SERIAL_BAUDRATE 4800
#elif defined(ARDUINO_AVR_MEGA2560)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#elif defined(ARDUINO_ARCH_ESP32)
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_BAUDRATE 115200
#endif

uint16_t interval;    // 运行周期 单位ms
uint16_t t_acc = 100; // 加速时间 单位ms
uint16_t t_dec = 100; // 减速时间 单位ms
float velocity = 150; // 目标转速 单位°/s
float power = 10000;
float last2, last3;
MPU6050 mpu;
// 新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
// 新建小车电机实例
A4950MotorShield motors;
// 建立4个编码器脉冲数变量
long newPulses[4] = {0, 0, 0, 0};
// 新建小车编码器实例
Encoder ENC[4] = {
    Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
    Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D)};
float targetPulses[4] = {0};
float Kp = 15, Ki = 0.1, Kd = 0;
double outPWM[4] = {0, 0, 0, 0};
// 新建4个电机速度PID实例
PID VeloPID[4] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;  // 附体坐标系下x方向速度，单位 m/s
float linear_vel_y = 0;  // 附体坐标系下y方向速度，单位 m/s
float angular_vel_z = 0; // 附体坐标系下z方向角速度，单位 rad/s
unsigned long currentMillis = millis();
unsigned long previousMillis = 0; // will store last time run
const long period = 5000;         // period at which to run in ms

// 运行状态标记 c++ enum实际上并没有创建任何变量，只是定义数据类型。
enum CARMOTION
{
  PAUSE,
  LEFTWARD,
  FORWARD,
  RIGHTWARD,
  BACKWARD
};
CARMOTION direction = PAUSE;
char var; // 上下位机通信
void control()
{
  sei(); // 全局中断开启
  // 设置各电机目标脉冲数（速度）=逆运动学解算各电机脉冲数（速度）
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;

  // 获取各电机实测脉冲数（速度），然后置零
#ifdef PINS_REVERSE
  newPulses[0] = -ENC[0].readAndReset(); // A
  newPulses[1] = ENC[1].readAndReset();  // B
  newPulses[2] = -ENC[2].readAndReset(); // C
  newPulses[3] = ENC[3].readAndReset();  // D
#else
  newPulses[0] = ENC[0].readAndReset();  // A
  newPulses[1] = -ENC[1].readAndReset(); // B
  newPulses[2] = ENC[2].readAndReset();  // C
  newPulses[3] = -ENC[3].readAndReset(); // D
#endif
  // 通过pid控制器输出各电机PWM,并复位编码器计数器
  for (int i = 0; i < WHEEL_NUM; i++)
  {
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
  }
  // 设置电机速度
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}
// 陀螺仪变量
//  MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success,
                        // !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container 四元数容器
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 gy; // [x, y, z]            gyro sensor measurements
VectorInt16
    aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;         // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float
    ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
/* 等待并报告当前的角度*/

int move(int number, char sensor)
{
  static int count = 0, flag = 0;
  if (digitalRead(sensor) == 1) // 如果发送高电平，识别到黑线进入
  {
    if (flag == 0)
    {           // 如果上一次检测时，白线没有进入，则视为第一次进入。
      flag = 1; // 先将标志记录为白线已进入。
      count++;  // 计数加一
    }
    if (count == number)
    {
      count = 0;
      flag = 0;
      return 1;
    }
  }
  else
  {           // 如果检测到障白线移出
    flag = 0; // 将标志记录为白线已移出。
  }
  return 0;
}

void allstop()
{
  linear_vel_x = 0;  // m/s
  linear_vel_y = 0;  // m/s
  angular_vel_z = 0; // rad/s
                     // 使用millis函数进行定时控制，代替delay函数
  if (!dmpReady)
    return;
  // read a packet from FIFO

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}

void left()
{
  linear_vel_x = 0;    // m/s
  linear_vel_y = 0.25; // m/s
  angular_vel_z = 0;   // rad/s
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

  Kinematics::velocities vel;
  // This is a simulated feedback from each motor. We'll just pass
  // the calculated rpm above for demo's sake. In a live robot,
  // these should be replaced with real RPM values derived from
  // encoder.
  int motor1_feedback = newPulses[0]; // in rpm
  int motor2_feedback = newPulses[1]; // in rpm
  int motor3_feedback = newPulses[2]; // in rpm
  int motor4_feedback = newPulses[3]; // in rpm
  // sensors_read();
#ifndef DEBUG
  vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                             motor3_feedback, motor4_feedback);
//  Serial.print(" VEL X: ");
//  Serial.print(vel.linear_x, 4);
//
//  Serial.print(" VEL_Y: ");
//  Serial.print(vel.linear_y, 4);
//
//  Serial.print(" ANGULAR_Z: ");
//  Serial.println(vel.angular_z, 4);
//  Serial.println("");
#endif
}

void right()
{
  linear_vel_x = 0;     // m/s
  linear_vel_y = -0.25; // m/s
  angular_vel_z = 0;    // rad/s
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

  Kinematics::velocities vel;
  // This is a simulated feedback from each motor. We'll just pass
  // the calculated rpm above for demo's sake. In a live robot,
  // these should be replaced with real RPM values derived from
  // encoder.
  int motor1_feedback = newPulses[0]; // in rpm
  int motor2_feedback = newPulses[1]; // in rpm
  int motor3_feedback = newPulses[2]; // in rpm
  int motor4_feedback = newPulses[3]; // in rpm
  // sensors_read();

#ifndef DEBUG
  vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                             motor3_feedback, motor4_feedback);
//  Serial.print(" VEL X: ");
//  Serial.print(vel.linear_x, 4);
//
//  Serial.print(" VEL_Y: ");
//  Serial.print(vel.linear_y, 4);
//
//  Serial.print(" ANGULAR_Z: ");
//  Serial.println(vel.angular_z, 4);
//  Serial.println("");
#endif
}

void forward()
{
  linear_vel_x = 0.2; // m/s
  linear_vel_y = 0;   // m/s
  angular_vel_z = 0;  // rad/s
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

  Kinematics::velocities vel;
  // This is a simulated feedback from each motor. We'll just pass
  // the calculated rpm above for demo's sake. In a live robot,
  // these should be replaced with real RPM values derived from
  // encoder.
  int motor1_feedback = newPulses[0]; // in rpm
  int motor2_feedback = newPulses[1]; // in rpm
  int motor3_feedback = newPulses[2]; // in rpm
  int motor4_feedback = newPulses[3]; // in rpm
  // sensors_read();

#ifndef DEBUG
  vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                             motor3_feedback, motor4_feedback);
//  Serial.print(" VEL X: ");
//  Serial.print(vel.linear_x, 4);
//
//  Serial.print(" VEL_Y: ");
//  Serial.print(vel.linear_y, 4);
//
//  Serial.print(" ANGULAR_Z: ");
//  Serial.println(vel.angular_z, 4);
//  Serial.println("");
#endif
}

void backward()
{
  linear_vel_x = -0.2; // m/s
  linear_vel_y = 0;    // m/s
  angular_vel_z = 0;   // rad/s
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

  Kinematics::velocities vel;
  // This is a simulated feedback from each motor. We'll just pass
  // the calculated rpm above for demo's sake. In a live robot,
  // these should be replaced with real RPM values derived from
  // encoder.
  int motor1_feedback = newPulses[0]; // in rpm
  int motor2_feedback = newPulses[1]; // in rpm
  int motor3_feedback = newPulses[2]; // in rpm
  int motor4_feedback = newPulses[3]; // in rpm
  // sensors_read();

#ifndef DEBUG
  vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                             motor3_feedback, motor4_feedback);
//  Serial.print(" VEL X: ");
//  Serial.print(vel.linear_x, 4);
//
//  Serial.print(" VEL_Y: ");
//  Serial.print(vel.linear_y, 4);
//
//  Serial.print(" ANGULAR_Z: ");
//  Serial.println(vel.angular_z, 4);
//  Serial.println("");
#endif
}


void back()
{
  linear_vel_x = -0.2; // m/s
  linear_vel_y = -0.2; // m/s
  angular_vel_z = 0;   // rad/s
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

  Kinematics::velocities vel;
  int motor1_feedback = newPulses[0]; // in rpm
  int motor2_feedback = newPulses[1]; // in rpm
  int motor3_feedback = newPulses[2]; // in rpm
  int motor4_feedback = newPulses[3]; // in rpm
  // sensors_read();

#ifndef DEBUG
  vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                             motor3_feedback, motor4_feedback);
//  Serial.print(" VEL X: ");
//  Serial.print(vel.linear_x, 4);
//
//  Serial.print(" VEL_Y: ");
//  Serial.print(vel.linear_y, 4);
//
//  Serial.print(" ANGULAR_Z: ");
//  Serial.println(vel.angular_z, 4);
//  Serial.println("");
#endif
}

void wait(int t)
{
  previousMillis = millis();
  while (1)
  {
    allstop();
    currentMillis = millis();
    if (currentMillis - previousMillis >= t * 1000)
    {
      previousMillis = currentMillis;
      break;
    }
  }
}
void test()
{
  previousMillis = millis();
  while (1)
  {
    backward();
    currentMillis = millis();
    if (currentMillis - previousMillis >= 5000)
    {
      previousMillis = currentMillis;
      break;
    }
  }
  wait(0.1);
  previousMillis = millis();
  while (1)
  {
    forward();
    currentMillis = millis();
    if (currentMillis - previousMillis >= 5000)
    {
      previousMillis = currentMillis;
      break;
    }
  }
  wait(0.1);
  previousMillis = millis();
  while (1)
  {
    right();
    currentMillis = millis();
    if (currentMillis - previousMillis >= 5000)
    {
      previousMillis = currentMillis;
      break;
    }
  }
  wait(0.1);
  previousMillis = millis();
  while (1)
  {
    left();
    currentMillis = millis();
    if (currentMillis - previousMillis >= 5000)
    {
      previousMillis = currentMillis;
      break;
    }
  }
  wait(0.1);
  while (1)
  {
    allstop();
  }
}
void MotionControl(double vx, double vy, double vz, double t)
{
  float yaw;
  previousMillis = millis();
  while (1)
  {
    linear_vel_x = vx;  // m/s
    linear_vel_y = vy;  // m/s
    angular_vel_z = vz; // rad/s
    if (!dmpReady)
      return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    { // Get the Latest packet

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      
      yaw=mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // Serial.print("yaw:");
      // Serial.print(yaw);
      angular_vel_z = ypr[0] * 5;
    }
    rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
    pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

    Kinematics::velocities vel;
    int motor1_feedback = newPulses[0]; // in rpm
    int motor2_feedback = newPulses[1]; // in rpm
    int motor3_feedback = newPulses[2]; // in rpm
    int motor4_feedback = newPulses[3]; // in rpm
                                        // sensors_read();

#ifndef DEBUG
    vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                               motor3_feedback, motor4_feedback);
#endif
    currentMillis = millis();
    if (currentMillis - previousMillis >= t)
    {
      previousMillis = currentMillis;
      break;
    }
  }
  wait(0.1);
}
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>


// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     43
#define TFT_RST    39   // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     41
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 49   // set these to be whatever pins you like!
#define TFT_MOSI 47   // set these to be whatever pins you like!
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
void displayTask(long int num=213321){
  tft.drawChar(20+48,0,num/100000+'0',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(20+64,0,(num/10000)%10+'0',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(20+80,0,(num/1000)%10+'0',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(20+48,64,(num%1000)/100+'0',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(20+64,64,(num%100)/10+'0',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(20+80,64,(num%10)+'0',ST7735_WHITE,ST7735_BLACK,2,4);
}


unsigned int public_instruction = 0;
#define INFO_NUM 8
struct CommInfo
{
  float last, current;
  char txt[12];
} comminfo[INFO_NUM];
struct CommInfo &vx_delta = comminfo[0], &vy_delta = comminfo[1], &vw_delta = comminfo[2], &t_delta = comminfo[3], &stepper_pos = comminfo[4], &motion_type = comminfo[5], &InsNum = comminfo[6],&FetchOrder = comminfo[7];
// String assistString;
#define SERIAL_RX_BUFFER_SIZE 4096
unsigned char rxBuf[512];                  // 串口接收缓冲
unsigned char *pChar = rxBuf, *pHead2Instruction = rxBuf, *pTail2Instruction = rxBuf; // 字符指针

// char kp1[10], ki1[10], kd1[10], sv1[10], mode[10], out[10], LR[10];
int serialRxFlag = 0; // 串口接收完标志
// char rou1[10], theta1[10];
// float rou, theta;
// void(* resetFunc) (void) = 0;
#define START_PIN 30
void start(){
  char press = digitalRead(START_PIN);
  while(!press){
    delay(100);
    press = digitalRead(START_PIN);
  }
}

void setup()
{
  // resetFunc();
  Serial.begin(115200);
  while (Serial.read() >= 0)
  {
    /* code */
  }
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化软串口的波特率
#ifdef OUTPUT_LOG
  DEBUG_SERIAL.println("SetServoAngle");
#endif
  // 电机初始化
  motors.init();
  // pinMode(49, INPUT);
  // pinMode(47, INPUT);
  // pinMode(43, INPUT);
  // pinMode(41, INPUT);
  // pinMode(39, INPUT);
  // pinMode(37, INPUT);
  // pinMode(35, INPUT);
  // pinMode(31, INPUT);
  // 陀螺仪IIC初始化
  //  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having
                         // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ;
  while (Serial.available() > 0)
  {
    char inByte = Serial.read();
    Serial.write(inByte);
    // Serial.write("wait\n");
  }
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // Enable driver in hardware
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  // Enable one according to your setup
  // SPI.begin();                    // SPI drivers
  //  Serial.begin(115200);
  SERIAL_PORT.begin(115200); // HW UART drivers
                             // driver.beginSerial(115200);     // SW UART drivers
  // driver.begin();            //  SPI: Init CS pins and possible SW SPI pins
  //                            // UART: Init SW UART (if selected) with default 115200 baudrate
  // driver.toff(5);            // Enables driver in software
  // driver.rms_current(600);   // Set motor RMS current
  // driver.microsteps(16);     // Set microsteps to 1/16th

  // // driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  // driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  // driver.pwm_autoscale(true);   // Needed for stealthChop
  // digitalWrite(EN_PIN, LOW);
  mpu.initialize();
#ifdef OUTPUT_LOG
  Serial.println(F("InitializingI2Cdevices..."));

  Serial.println(F("TestingDeviceConnections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050connectionsuccessful")
                                      : F("MPU6050connectionfailed"));
  // load and configure the DMP
  Serial.println(F("InitializingDMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(80);
  mpu.setYAccelOffset(777);
  mpu.setZAccelOffset(1554);
  mpu.setXGyroOffset(21);
  mpu.setYGyroOffset(-50);
  mpu.setZGyroOffset(31);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // mpu.PrintActiveOffsets();
// turn on the DMP, now that it's ready
#ifdef OUTPUT_LOG
    Serial.println();
    Serial.println(F("EnablingDMP..."));
#endif
    mpu.setDMPEnabled(true);

    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
#ifdef OUTPUT_LOG
    Serial.print(F("DMPInitializationFailedCode"));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }

  delay(100);                              // 延时等待初始化完成
  FlexiTimer2::set(TIMER_PERIOD, control); // 10毫秒定时中断函数
  FlexiTimer2::start();                    // 中断使能
  delay(100);                              // 延时等待初始化完成
#ifdef OUTPUT_LOG
  Serial.println("Sunnybot-Basic-Mecanum-Rectangle-Test:");
#endif
  memset(rxBuf, ' ', sizeof(rxBuf)); // 数组清零，不清的话串口会有问题
  // StepperControl(4,1);
  tft.initR(INITR_BLACKTAB); 
  tft.fillScreen(ST7735_BLACK);
  tft.drawChar(10+0,0,'1',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(10+16,0,'s',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(10+32,0,'t',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(10+0,64,'2',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(10+16,64,'s',ST7735_WHITE,ST7735_BLACK,2,4);
  tft.drawChar(10+32,64,'t',ST7735_WHITE,ST7735_BLACK,2,4);

  // displayTask();
}
// void serialEvent(){
//   while (Serial.available() > 0)
//   {
//     // Serial.print("j");
//     char inByte = Serial.read(); // 串口读进来的始终是一个整数，即ASCII码值80（助手以ASCII码发送，如'P'）
//     // Serial.write(inByte);
//     if (('0' <= inByte and '9' >= inByte) or ('A' <= inByte or inByte <= 'Z') or inByte == ',' or inByte == '\n' or inByte == '!')
//     {
//       *pChar++ = inByte; // 保存一个字符
//       // if (rxBuf[0] != 'I')
//       // {
//       //   memset(rxBuf, ' ', sizeof(rxBuf)); // 数组清零，不清的话串口会有问
//       //   pChar = &rxBuf[0];                 // 指针回指，准备下次接收题
//       // }

//       // if (inByte == '\n' or inByte == '!')
//       if (inByte == '!')
//       { // 检查是不是最后一个字符：回车换行符0x0D 0x0A
//         serialRxFlag += 1;
//         // Serial.write(inByte);
//       }

//     }
//   }
//   Serial.print("2\n");
// }
void loop()
{
  // Serial.print("1\n");
  // Serial.print("aaa\n");
  // while (!(Serial.available() > 0))
  // {
  //   // Serial.write("wait\n");
  // }
  while (Serial.available() > 0)
  {
    // Serial.print("j");
    char inByte = Serial.read(); // 串口读进来的始终是一个整数，即ASCII码值80（助手以ASCII码发送，如'P'）
    // Serial.write(inByte);
    if (('0' <= inByte and '9' >= inByte) or ('A' <= inByte or inByte <= 'Z') or inByte == ',' or inByte == '\n' or inByte == '!')
    {
      *pChar++ = inByte; // 保存一个字符
      if (inByte == 'I')
      {
        // Serial.print('a');
      }
      Serial.write(inByte);
      // if (inByte == '\n' or inByte == '!')
      if (inByte == '!')
      { // 检查是不是最后一个字符：回车换行符0x0D 0x0A
        serialRxFlag += 1;
        // Serial.print('b');
        // Serial.print('\n');
      }
    }
  }
  if (serialRxFlag > 0)
  {
    pHead2Instruction = &rxBuf[0];

    while (serialRxFlag > 0)
    { // 处理接收的字符串
      serialRxFlag -= 1;
      // if (serialRxFlag == 0)
      // {
      //   pChar = &rxBuf[0]; // 指针回指，准备下次接收
      // }

// Serial.print("串口发送的命令串:"); // 调试时可以取消注释
#ifdef OUTPUT_LOG
      Serial.write(rxBuf, 50);
      Serial.write('!');
#endif
      // Serial.write(rxBuf, 50);
      // Serial.write('!');
      // Serial.println();
      // 串口发送例子N,P6.0,I2.5,D0.6,S45.5,M1,T50,R1    第一个字母区别是内回路参数还是外回路参数，注意要发送新行
      // 串口发送例子W,P6.0,I2.5,D0.6,S45.5,M1,T50       第一个字母区别是内回路参数还是外回路参数，注意要发送新行
      while (pHead2Instruction[0] != 'I' and pHead2Instruction < pChar)
      {
        pHead2Instruction++;
      }
      // TODO
      // may be not needed currently
      pTail2Instruction = pHead2Instruction;
      while (pTail2Instruction[0] != '!' and pTail2Instruction < pChar)
      {
        pTail2Instruction++;
      }
      // Serial.write(pHead2Instruction[0]);
      // Serial.write(pTail2Instruction[0]);
      // Serial.print('\n');
      if (pHead2Instruction[0] == 'I' and pTail2Instruction[0] == '!')
      // if (pHead2Instruction[0] == 'I' )
      { // 解析命令
        // Serial.println("wok");   //调试时可以取消注释
        sscanf((const char *)pHead2Instruction, "I%[^','],X%[^','],Y%[^','],W%[^','],T%[^','],O%[^','],N%[^','],P[^'\n']\n", InsNum.txt, vx_delta.txt, vy_delta.txt, vw_delta.txt, t_delta.txt, stepper_pos.txt,FetchOrder.txt,motion_type.txt);

        // 1.%前面的字符为跳过的字符，如果要跳过多个字符，应全部放在%之前；
        // 2.^为读取的字符串，后面所跟字符为截至字符，就是到逗号为止且丢掉逗号；
        // 3.^须用[]括起，所以一定要核对[]符号的数量
        for (int i = 0; i < INFO_NUM; i++)
        {
          // // comminfo[i].current = comminfo[i].txt.toFloat();
          // assistString=comminfo[i].txt;
          // comminfo[i].current=assistString.toFloat();
          comminfo[i].current = atof(comminfo[i].txt);
          // assistString
        }
        public_instruction = 1;
        // memset(rxBuf, ' ', sizeof(rxBuf)); // 数组清零，不清的话串口会有问题
        pHead2Instruction = pTail2Instruction + 1;
        // Serial.print("pub");
      }
      if (public_instruction)
      {
        public_instruction = 0;
        // Serial.print('F');
        if (InsNum.current==2){
          start();
        }

        // vx vy vw t
        // Serial.write(rxBuf, 50);
        Serial.print('F');
        Serial.print(InsNum.current);
        Serial.print(serialRxFlag);
        Serial.print(FetchOrder.current);
        Serial.print(stepper_pos.current);
        Serial.print('!');

        // Serial.print(InsNum.txt);
        // Serial.print(vx_delta.current);
        // Serial.print(vx_delta.txt);
        // Serial.print(vy_delta.current);
        // Serial.print(vy_delta.txt);
        Serial.print('\n');
        if (FetchOrder.current!=-999){
          displayTask(FetchOrder.current);
        }
        if (vx_delta.current != -999 && vy_delta.current != -999 && vw_delta.current != -999 && t_delta.current != -999)
        {
          MotionControl(vx_delta.current, vy_delta.current, vw_delta.current, t_delta.current);
        }
        if (stepper_pos.current != -999)
        {
          StepperControl(abs(stepper_pos.current), (bool)(stepper_pos.current > 0));
        }
        Serial.print('K');
        Serial.print('!');
        Serial.print('\n');
        for (int i = 0; i < INFO_NUM; i++)
        {
          if (comminfo[i].current != -999)
          {
            comminfo[i].last = comminfo[i].current;
          }
        }
      }
      // pChar
    }
    for (unsigned int i = 0; i < pChar - pHead2Instruction; i++)
    {
      rxBuf[i] = pHead2Instruction[i];
    }
    memset(rxBuf + (pChar - pHead2Instruction), ' ', sizeof(rxBuf)-(pChar - pHead2Instruction)); // 数组清零，不清的话串口会有问题
    pChar = rxBuf + (pChar - pHead2Instruction);
    pHead2Instruction = pChar;
  }
}

// ardunio peinrln 为\r\n