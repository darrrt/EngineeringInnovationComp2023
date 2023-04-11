/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper
 * motor in alternating directions.
 */
#include <Arduino.h>
#include <TMCStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// our servo # counter
uint8_t servonum = 0;
char comchar;

// 默认地址 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
//这是“最小”脉冲长度计数（在4096）中
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
//这是“最大”脉冲长度计数（在4096中）

#define Servo0           30
#define Servo1           31
#define Servo2           32
#define EN_PIN           33 // Enable
#define DIR_PIN          40 // Direction
#define STEP_PIN         34 // Step
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
//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC2660Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2660Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
//TMC5160Stepper driver(CS_PIN, R_SENSE);
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware Serial
//TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software serial
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
// Servo Servo0,Servo1,Servo2;
// void ServoTurn(int pos, Servo myServo){
//   int i=0;
//   for (i=0;i<=pos;i++)
//   {
//     myServo.write(i);
//     delay(10);
//   }
// }

void ServoControl(int servoAngle,int servoPin)
{
  double thisAngle = map(servoAngle, 0, 180, 500, 2500);//等比例角度值范围转换高电平持续时间范围
  unsigned char i = 8;//50Hz 每秒的周期次数(周期/秒) 即1S 50 个周期 每个周期20ms
  while (i--)
  {
    digitalWrite(servoPin, HIGH); 
    delayMicroseconds(thisAngle); //高电平时间
    digitalWrite(servoPin, LOW); 
    delayMicroseconds(20000 - thisAngle);//每个周期20ms减去高电平持续时间
  }
}
//单个舵机转动一个来回
void setAngel(int servoPin,int startAngle,int endAngle)
{
  if(endAngle>startAngle)
  {
       for (int i = startAngle; i <= endAngle ; i += 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
     for (int i = endAngle; i >= startAngle ; i -= 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
  }
  else{
     for (int i = startAngle; i >= endAngle ; i -= 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
     for (int i = endAngle; i <= startAngle ; i += 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
  }
}

void setServoPulse(uint8_t n, double pulse) {
   double pulselength;//精度浮点数
   
   pulselength = 1000000;   // 1,000,000 us per second 每秒100万
   pulselength /= 60;   // 60 Hz
   Serial.print(pulselength); Serial.println(" us per period"); 
   pulselength /= 4096;  // 12 bits of resolution 12位分辨率
   Serial.print(pulselength); Serial.println(" us per bit"); 
   pulse *= 1000;
   pulse /= pulselength;
   Serial.println(pulse);
   pwm.setPWM(n, 0, pulse);
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(60);  // 50HZ更新频率，相当于20ms的周期

  delay(10);
  // Servo0.attach(30);
  // Servo1.attach(31);
  // Servo2.attach(32);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);      // Enable driver in hardware
  digitalWrite(DIR_PIN ,LOW);
  digitalWrite(STEP_PIN, LOW);
                                  // Enable one according to your setup
//SPI.begin();                    // SPI drivers
  // Serial.begin(115200);
  SERIAL_PORT.begin(115200);      // HW UART drivers
//driver.beginSerial(115200);     // SW UART drivers
  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(16);          // Set microsteps to 1/16th

//driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
  digitalWrite(EN_PIN, LOW);
}

bool shaft = false;

void loop() {
  // Run 5000 steps and switch direction in software
  // ServoTurn(90,Servo0);
  // Servo0.write(0);
  // setAngel(Servo2,10,100);

  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
     pwm.setPWM(0, 0, pulselen);
     pwm.setPWM(1, 0, pulselen);
     pwm.setPWM(2, 0, pulselen);
     pwm.setPWM(3, 0, pulselen);
     pwm.setPWM(4, 0, pulselen);
     pwm.setPWM(5, 0, pulselen);
     pwm.setPWM(6, 0, pulselen);
     pwm.setPWM(7, 0, pulselen);
     pwm.setPWM(8, 0, pulselen);
     pwm.setPWM(9, 0, pulselen);
     pwm.setPWM(10, 0, pulselen);
     pwm.setPWM(11, 0, pulselen);
     pwm.setPWM(12, 0, pulselen);
     pwm.setPWM(13, 0, pulselen);
     pwm.setPWM(14, 0, pulselen);

     pwm.setPWM(15, 0, pulselen);
   }
   delay(500);
   for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
     pwm.setPWM(0, 0, pulselen);
     pwm.setPWM(1, 0, pulselen);
     pwm.setPWM(2, 0, pulselen);
     pwm.setPWM(3, 0, pulselen);
     pwm.setPWM(4, 0, pulselen);
     pwm.setPWM(5, 0, pulselen);
     pwm.setPWM(6, 0, pulselen);
     pwm.setPWM(7, 0, pulselen);
     pwm.setPWM(8, 0, pulselen);
     pwm.setPWM(9, 0, pulselen);
     pwm.setPWM(10, 0, pulselen);
     pwm.setPWM(11, 0, pulselen);
     pwm.setPWM(12, 0, pulselen);
     pwm.setPWM(13, 0, pulselen);
     pwm.setPWM(14, 0, pulselen);

     pwm.setPWM(15, 0, pulselen);
   }
   delay(500);

  // for (uint16_t i = 1000; i>0; i--) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(200);
  //   // Serial.println("HIGH");
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(200);
  //   // Serial.println("LOW");

  // }
  //     shaft = !shaft;
  //   driver.shaft(shaft);
  //   digitalWrite(DIR_PIN ,shaft);
}


// // #include <Arduino.h>
// // #include <TMC2209.h>
// // #include <TMCStepper.h>

// // HardwareSerial & serial_stream = Serial1;

// // const long SERIAL_BAUD_RATE = 115200;
// // const int DELAY = 2000;
// // const int32_t VELOCITY = 20000;
// // // current values may need to be reduced to prevent overheating depending on
// // // specific motor and power supply voltage
// // const uint8_t RUN_CURRENT_PERCENT = 100;


// // // Instantiate TMC2209
// // TMC2209 stepper_driver;


// // void setup()
// // {
// //   Serial.begin(SERIAL_BAUD_RATE);

// //   stepper_driver.setup(serial_stream);

// //   if (stepper_driver.isSetupAndCommunicating())
// //   {
// //     Serial.println("Stepper driver setup and communicating!");
// //     Serial.println("");
// //   }
// //   else
// //   {
// //     Serial.println("Stepper driver not setup and communicating!");
// //     return;
// //   }

// //   stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
// //   stepper_driver.enable();
// //   stepper_driver.moveAtVelocity(VELOCITY);
// // }

// // void loop()
// // {
// //   if (not stepper_driver.isSetupAndCommunicating())
// //   {
// //     Serial.println("Stepper driver not setup and communicating!");
// //     return;
// //   }

// //   bool disabled_by_input_pin = stepper_driver.disabledByInputPin();
// //   TMC2209::Settings settings = stepper_driver.getSettings();
// //   TMC2209::Status status = stepper_driver.getStatus();

// //   if (disabled_by_input_pin)
// //   {
// //     Serial.println("Stepper driver is disabled by input pin!");
// //   }
// //   else if (not settings.enabled)
// //   {
// //     Serial.println("Stepper driver is disabled by firmware!");
// //   }
// //   else if ((not status.standstill))
// //   {
// //     Serial.print("Moving at velocity ");
// //     Serial.println(VELOCITY);

// //     uint32_t interstep_duration = stepper_driver.getInterstepDuration();
// //     Serial.print("which is equal to an interstep_duration of ");
// //     Serial.println(interstep_duration);
// //   }
// //   else
// //   {
// //     Serial.println("Not moving, something is wrong!");
// //   }

// //   Serial.println("");
// //   delay(DELAY);
// // }

// /*
//     Controlling two stepper with the AccelStepper library

//      by Dejan, https://howtomechatronics.com
// */

// #include <AccelStepper.h>
// #include <Arduino.h>
// #include <Servo.h>
// #include <TMCStepper.h>
// #define ROUND 30*28 
// // Define the stepper motor and the pins that is connected to
// AccelStepper stepper1(3, 33, 34, 40); // (Typeof driver: with 3 pins,EN, STEP, DIR)
// Servo Servo0,Servo1,Servo2;
// void ServoTurn(int pos, Servo myServo){
//   int i=0;
//   for (i=0;i<=pos;i++)
//   {
//     myServo.write(i);
//     delay(10);
//   }
// }

// void setup() {
//   Servo0.attach(A0);
//   Servo1.attach(A1);
//   Servo2.attach(A2);
//   stepper1.setMaxSpeed(50); // Set maximum speed value for the stepper
//   stepper1.setAcceleration(50); // Set acceleration value for the stepper
//   stepper1.setCurrentPosition(0); // Set the current position to 0 steps
// }

// void loop() {
//   // ServoTurn(90,Servo1);
//   stepper1.runToNewPosition(100);
//   // stepper1.runToNewPosition(2048);
//   // stepper1.runToNewPosition(0);
//   // stepper1.moveTo(800); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
//   // stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position

//   // // Move back to position 0, using run() which is non-blocking - both motors will move at the same time
//   // stepper1.moveTo(0);
//   // while (stepper1.currentPosition() != 0) {
//   //   stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
//   // }
// }
