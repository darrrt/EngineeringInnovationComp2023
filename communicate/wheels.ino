#include <Stream.h>

// String inString[128];
// short readinLen=0;

unsigned char rxBuf[64];	  // 串口接收缓冲
unsigned char *pChar = rxBuf; // 字符指针

// char kp1[10], ki1[10], kd1[10], sv1[10], mode[10], out[10], LR[10];
int serialRxFlag = 0; // 串口接收完标志
// char rou1[10], theta1[10];
// float rou, theta;


#include <SunConfig.h>
#include <Version.h>

//走矩形线路测试
//运行前先校准陀螺仪
// 20210406更新 使用枚举类型 imu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <FlexiTimer2.h>  //定时中断
#include "SunConfig.h"
#define S1 49
#define S2 47
#define S3 43
#define S4 41
#define S5 39
#define S6 37
#define S7 35
#define trig 30 
// 串口总线舵机配置参数
#define BAUDRATE 115200 // 波特率

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

uint16_t interval;  // 运行周期 单位ms
uint16_t t_acc = 100;   // 加速时间 单位ms
uint16_t t_dec = 100;   // 减速时间 单位ms
float velocity = 150;         // 目标转速 单位°/s
float power = 10000;
float last2,last3;
MPU6050 mpu;
//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
  LR_WHEELS_DISTANCE);
//新建小车电机实例
A4950MotorShield motors;
//建立4个编码器脉冲数变量
long newPulses[4] = { 0, 0, 0, 0 };
//新建小车编码器实例
Encoder ENC[4] = {
  Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
  Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D) };
float targetPulses[4] = { 0 };
float Kp = 15, Ki = 0.1, Kd = 0;
double outPWM[4] = { 0, 0, 0, 0 };
//新建4个电机速度PID实例
PID VeloPID[4] = {
  PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
  PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd) };

Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;   //附体坐标系下x方向速度，单位 m/s
float linear_vel_y = 0;   //附体坐标系下y方向速度，单位 m/s
float angular_vel_z = 0;  //附体坐标系下z方向角速度，单位 rad/s
unsigned long currentMillis = millis();
unsigned long previousMillis = 0;  // will store last time run
const long period = 5000;          // period at which to run in ms

//运行状态标记 c++ enum实际上并没有创建任何变量，只是定义数据类型。
enum CARMOTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD };
CARMOTION direction = PAUSE;
char var;//上下位机通信
void control() {
  sei();  //全局中断开启
  //设置各电机目标脉冲数（速度）=逆运动学解算各电机脉冲数（速度）
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;

  //获取各电机实测脉冲数（速度），然后置零
#ifdef PINS_REVERSE
  newPulses[0] = -ENC[0].readAndReset();  // A
  newPulses[1] = ENC[1].readAndReset();   // B
  newPulses[2] = -ENC[2].readAndReset();  // C
  newPulses[3] = ENC[3].readAndReset();   // D
#else
  newPulses[0] = ENC[0].readAndReset();   // A
  newPulses[1] = -ENC[1].readAndReset();  // B
  newPulses[2] = ENC[2].readAndReset();   // C
  newPulses[3] = -ENC[3].readAndReset();  // D
#endif
  //通过pid控制器输出各电机PWM,并复位编码器计数器
  for (int i = 0; i < WHEEL_NUM; i++) {
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
  }
  //设置电机速度
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}
//陀螺仪变量
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
            // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;    // [w, x, y, z]         quaternion container 四元数容器
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16 gy;  // [x, y, z]            gyro sensor measurements
VectorInt16
aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float
ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
/* 等待并报告当前的角度*/

int move(int number, char sensor)
{
  static int count = 0, flag = 0;
  if (digitalRead(sensor) == 1) //如果发送高电平，识别到黑线进入
  {
    if (flag == 0)
    { //如果上一次检测时，白线没有进入，则视为第一次进入。
      flag = 1;  //先将标志记录为白线已进入。
      count++;//计数加一
    }
    if (count == number)
    {
      count = 0;
      flag = 0;
      return 1;
    }
  }
  else
  {  //如果检测到障白线移出
    flag = 0;  //将标志记录为白线已移出。
  }
  return 0;
}

void allstop() {
  linear_vel_x = 0;   // m/s
  linear_vel_y = 0;   // m/s
  angular_vel_z = 0;  // rad/s
  //使用millis函数进行定时控制，代替delay函数
    if (!dmpReady) return;
  // read a packet from FIFO
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
}

void left() {
  linear_vel_x = 0;    // m/s
  linear_vel_y = 0.25;  // m/s
  angular_vel_z = 0;   // rad/s
 if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

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
  int motor1_feedback = newPulses[0];  // in rpm
  int motor2_feedback = newPulses[1];  // in rpm
  int motor3_feedback = newPulses[2];  // in rpm
  int motor4_feedback = newPulses[3];  // in rpm
  //sensors_read();
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

void right() {
  linear_vel_x = 0;    // m/s
  linear_vel_y = -0.25;  // m/s
  angular_vel_z = 0;   // rad/s
 if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

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
  int motor1_feedback = newPulses[0];  // in rpm
  int motor2_feedback = newPulses[1];  // in rpm
  int motor3_feedback = newPulses[2];  // in rpm
  int motor4_feedback = newPulses[3];  // in rpm
  //sensors_read();

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

void forward() {
  linear_vel_x = 0.2;  // m/s
  linear_vel_y = 0;    // m/s
  angular_vel_z = 0;   // rad/s
 if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

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
  int motor1_feedback = newPulses[0];  // in rpm
  int motor2_feedback = newPulses[1];  // in rpm
  int motor3_feedback = newPulses[2];  // in rpm
  int motor4_feedback = newPulses[3];  // in rpm
  //sensors_read();

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

void backward() {
  linear_vel_x = -0.2;  // m/s
  linear_vel_y = 0;    // m/s
  angular_vel_z = 0;   // rad/s
 if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

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
  int motor1_feedback = newPulses[0];  // in rpm
  int motor2_feedback = newPulses[1];  // in rpm
  int motor3_feedback = newPulses[2];  // in rpm
  int motor4_feedback = newPulses[3];  // in rpm
  //sensors_read();


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
void track(){
  if(S4==0&&S5==1){
    right();
  }
  else if(S4==0&&S3==1){
    left();
  }else{
    forward();
  }
}

void setup() {

	Serial.begin(115200);
	while (Serial.read()>=0)
	{
	/* code */
	}
  DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUDRATE); // 初始化软串口的波特率
  DEBUG_SERIAL.println("Set Servo Angle");
  //电机初始化
  motors.init();
    pinMode(49, INPUT);
    pinMode(47, INPUT);
    pinMode(43, INPUT);
    pinMode(41, INPUT);
    pinMode(39, INPUT);
    pinMode(37, INPUT);
    pinMode(35, INPUT);
    pinMode(31,INPUT);
  //陀螺仪IIC初始化
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having
              // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
    : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-522);
  mpu.setYAccelOffset(1047);
  mpu.setZAccelOffset(1244);
  mpu.setXGyroOffset(126);
  mpu.setYGyroOffset(-1);
  mpu.setZGyroOffset(-8);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100);                               //延时等待初始化完成
  FlexiTimer2::set(TIMER_PERIOD, control);  // 10毫秒定时中断函数
  FlexiTimer2::start();                     //中断使能
  delay(100);                               //延时等待初始化完成

  Serial.println("Sunnybot Basic  Mecanum Rectangle Test:");
}
void sensors_read(){
   Serial.print(digitalRead(S1));
   Serial.print(digitalRead(S2));
   Serial.print(digitalRead(S3));
   Serial.print(digitalRead(S4));
   Serial.print(digitalRead(S5));
   Serial.print(digitalRead(S6));
   Serial.print(digitalRead(S7));
   Serial.println(digitalRead(trig));
  }
void commu(){
  while(1){
    allstop();
    if (Serial.available() > 0)
    {
      var = Serial.read();
      if (var=='1')
      {
        var = 0;
        break;
      }
    }
  }
 }
void back(){
  linear_vel_x = -0.2;  // m/s
  linear_vel_y = -0.2;    // m/s
  angular_vel_z = 0;   // rad/s
 if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angular_vel_z = ypr[0] * 5;
  }
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);

  Kinematics::velocities vel;
  int motor1_feedback = newPulses[0];  // in rpm
  int motor2_feedback = newPulses[1];  // in rpm
  int motor3_feedback = newPulses[2];  // in rpm
  int motor4_feedback = newPulses[3];  // in rpm
  //sensors_read();


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

void wait(int t){
  previousMillis = millis();
  while (1) {
     allstop();  
     currentMillis = millis();
     if(currentMillis - previousMillis >= t*1000){
       previousMillis = currentMillis;
       break;
     }
  }
}
void test(){
  previousMillis = millis();
  while (1) {
     backward();  
     currentMillis = millis();
     if(currentMillis - previousMillis >= 5000){
       previousMillis = currentMillis;
       break;
     }
  }
  wait(0.1);
  previousMillis = millis();
  while (1) {
     forward();  
     currentMillis = millis();
     if(currentMillis - previousMillis >= 5000){
       previousMillis = currentMillis;
       break;
     }
  }
  wait(0.1);
  previousMillis = millis();
  while (1) {
     right();  
     currentMillis = millis();
     if(currentMillis - previousMillis >= 5000){
       previousMillis = currentMillis;
       break;
     }
  }
  wait(0.1);
  previousMillis = millis();
  while (1) {
     left();  
     currentMillis = millis();
     if(currentMillis - previousMillis >= 5000){
       previousMillis = currentMillis;
       break;
     }
  }
  wait(0.1);
  while(1){
    allstop();
  }
}
void motion(double vx, double vy, double vz, double t){
  previousMillis = millis();
  while (1) {
    linear_vel_x = vx;  // m/s
    linear_vel_y = vy;    // m/s
    angular_vel_z =vz;   // rad/s
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
  
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      angular_vel_z = ypr[0] * 5;
    }
    rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
    pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
  
    Kinematics::velocities vel;
    int motor1_feedback = newPulses[0];  // in rpm
    int motor2_feedback = newPulses[1];  // in rpm
    int motor3_feedback = newPulses[2];  // in rpm
    int motor4_feedback = newPulses[3];  // in rpm
    //sensors_read();
  
  
  #ifndef DEBUG
    vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                               motor3_feedback, motor4_feedback);
  #endif
     currentMillis = millis();
     if(currentMillis - previousMillis >= t){
       previousMillis = currentMillis;
       break;
     }
  }
  wait(0.1);
}

float vx_delta,vy_delta,vw_delta,t_delta,servo0_pos,servo1_pos,servo2_pos,servo3_pos,stepper_pos,motion_type;
float vx_delta_last,vy_delta_last,vw_delta_last,t_delta_last,servo0_pos_last,servo1_pos_last,servo2_pos_last,servo3_pos_last,stepper_pos_last,motion_type_last;
char vx_delta1[20],vy_delta1[20],vw_delta1[20],t_delta1[20],servo0_pos1[20],servo1_pos1[20],servo2_pos1[20],servo3_pos1[20],stepper_pos1[20],motion_type1[20],trans[20];

unsigned int public_instruction=0;
void loop()
{
	// Serial.print("aaa\n");
	while(!(Serial.available()>0)){
		// Serial.write("wait\n");
	}
	while (Serial.available()>0)
	{
		// Serial.print("j");
		int inByte = Serial.read(); // 串口读进来的始终是一个整数，即ASCII码值80（助手以ASCII码发送，如'P'）
		// Serial.write(inByte);
		if (('0' <= inByte and '9' >= inByte) or ('A' <= inByte or inByte <= 'Z') or inByte == ',' or inByte == '\n')
		{
			*pChar++ = inByte; // 保存一个字符
			if (rxBuf[0] != 'I')
			{
				memset(rxBuf, '\0', sizeof(rxBuf)); // 数组清零，不清的话串口会有问
				pChar = &rxBuf[0];					// 指针回指，准备下次接收题
			}

			if (inByte == '\n' or inByte=='!')
			{ // 检查是不是最后一个字符：回车换行符0x0D 0x0A
				serialRxFlag = 1;
			}
			// Serial.print(inByte);
		}
	}
	if (serialRxFlag == 1)
	{ // 处理接收的字符串
		serialRxFlag = 0;
		pChar = &rxBuf[0];				   // 指针回指，准备下次接收
		// Serial.print("串口发送的命令串:"); // 调试时可以取消注释
		Serial.write(rxBuf, 50);
		Serial.write('!');
		// Serial.println();
		// 串口发送例子N,P6.0,I2.5,D0.6,S45.5,M1,T50,R1    第一个字母区别是内回路参数还是外回路参数，注意要发送新行
		// 串口发送例子W,P6.0,I2.5,D0.6,S45.5,M1,T50       第一个字母区别是内回路参数还是外回路参数，注意要发送新行
		if (rxBuf[0] == 'I')
		{ // 解析外回路命令
			// Serial.println("wok");   //调试时可以取消注释
			sscanf(rxBuf, "I,X%[^','],Y%[^','],W%[^','],T%[^','],S%[^','],E%[^','],R%[^','],V%[^','],O%[^','],P[^'\n']\n", vx_delta,vy_delta,vw_delta,t_delta,servo0_pos,servo1_pos,servo2_pos,servo3_pos,stepper_pos,motion_type);

			// 1.%前面的字符为跳过的字符，如果要跳过多个字符，应全部放在%之前；
			// 2.^为读取的字符串，后面所跟字符为截至字符，就是到逗号为止且丢掉逗号；
			// 3.^须用[]括起，所以一定要核对[]符号的数量
			vx_delta=atof(vx_delta1);
			vy_delta=atof(vy_delta1);
			w_delta=atof(w_delta1);
			t_delta=atof(t_delta1);
			servo0_pos=atof(servo0_pos1);
			servo1_pos=atof(servo1_pos1);
			servo2_pos=atof(servo2_pos1);
			servo3_pos=atof(servo3_pos1);
			stepper_pos=atof(stepper_pos1);
			motion_type=atof(motion_type1);

			// theta = atof(theta1); // 字符串转成浮点数
			// rou = atof(rou1);

			// Serial.print("rou");
			// Serial.print(rou);
			// Serial.print("theta");
			// Serial.print(theta);
			// Serial.write("!\n");

			memset(rxBuf, '\0', sizeof(rxBuf)); // 数组清零，不清的话串口会有问题
			public_instruction=1;
		}
	}
	if (public_instruction){
		public_instruction=-1;
		motion(0,0,3.14,10);
		// vx vy vw t 
	}
}

// ardunio peinrln 为\r\n 