#include <Messenger.h> //Library for communicating with the computer
#include <kinematics_arduino.h> //Library by Juan for the kinematics
#include <odometer.h> //Library by Juan for the Odometry
#include <PID_v1.h>  //PID library for Arduino, Modified Integral term reset when in setpoint and output =0
#include <Servo.h>  //Arduino servo library
#include <Wire.h>  //Arduino I2C Library
#include <I2CEncoder.h>  //Library from alexhenning on github for driving the 269 Vex Encoders with arduino
#include <Ultrasonic.h> //Library to drive the Ultrasonic sensor on the shovel of the robot
#include "I2Cdev.h" //Necessary library for using MPU
#include "MPU6050_6Axis_MotionApps20.h" //Library for interfacing the MPU

#define encoders_on 53 //Encoders switch on
#define IMU_ON 51 //Imu switch on

//Declaring the MPU object
MPU6050 mpu(0x69); //With the I2C address

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw = 0.0, pitch = 0.0, roll = 0.0;
bool imu_use = false;
//Odometer send values
int sx, sy, sth, svx, svy, sw;
//Declaring the Ultrasonic object
Ultrasonic ultrasonic(10, 9); // (Trig PIN, Echo PIN)
//Declaring the servos
Servo FR;
Servo FL;
Servo RR;
Servo RL;
//Declaring the Encoders
I2CEncoder FRencoder;
I2CEncoder FLencoder;
I2CEncoder RRencoder;
I2CEncoder RLencoder;
//Declaring servo pins
const int servoFL = 22; //correct
const int servoFR = 24; //correct
const int servoRR = 30; //correct
const int servoRL = 28; //correct
//Messenger declaration
Messenger omMessenger = Messenger();
bool initialized = false;
unsigned long updte = 0;
//Declaring PID objects an dvariables
double FLsetpoint, FLinput, FLoutput, FLlastp;
double FRsetpoint, FRinput, FRoutput, FRlastp;
double RRsetpoint, RRinput, RRoutput, RRlastp;
double RLsetpoint, RLinput, RLoutput, RLlastp;
double flKp = 0.33, flKi = 1.1, flKd = 0.001;
double frKp = 0.33, frKi = 1.1, frKd = 0.001;
double rrKp = 0.33, rrKi = 1.1, rrKd = 0.001;
double rlKp = 0.33, rlKi = 1.1, rlKd = 0.001;
//Declaring the PID
PID FLpid(&FLinput, &FLoutput, &FLsetpoint, flKp, flKi, flKd, DIRECT);
PID FRpid(&FRinput, &FRoutput, &FRsetpoint, frKp, frKi, frKd, REVERSE);
PID RRpid(&RRinput, &RRoutput, &RRsetpoint, rrKp, rrKi, rrKd, REVERSE);
PID RLpid(&RLinput, &RLoutput, &RLsetpoint, rlKp, rlKi, rlKd, DIRECT);
//kinematics variables
double vx, vy, vth;
//Correction values
double linearscale = 1;
double angularscale = 0.984;
//Kinematics declaration
kinematics_arduino kinematics(&vx, &vy, &vth, &FLsetpoint, &FRsetpoint, &RRsetpoint, &RLsetpoint, &linearscale, &angularscale);
//Odometer declaration
Odometer odometer(&FLinput, &FRinput, &RRinput, &RLinput, &yaw, &linearscale, &angularscale);
//Support variables
int ultra_delay = 0;
unsigned long timesincelastcommand = 0;
boolean flagzero = true;
boolean connection_mpu;
boolean using_imu = false ;   //Flag for indicating use of Imu
boolean auto_imu = false; //Flag for activating the imu after 30 seconds
boolean no_imu = false;


//
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE  MPU             ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Wire.begin();
  imu_on();
  Wire1.begin();
  Serial.begin(57600);
  omMessenger.attach(OnReceived);
  mpu.initialize(); //Initialize Imu
  connection_mpu = mpu.testConnection(); //CHeck if connection was succesful
  devStatus = mpu.dmpInitialize(); //Initialize the Digital Motion Proccesing
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if ( devStatus == 0)
  {
    Serial.print("p");
    Serial.print("\t");
    Serial.print("IMU initiated");
    Serial.print("\n");



    mpu.setDMPEnabled(true);
    attachInterrupt(50, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print("p");
    Serial.print("\t");
    Serial.print("IMU failed");
    Serial.print("\n");
  }
  //encodersPowerOn();
  encoderSetup();
  motorsSetup();
  pidSetup();
  delay(100);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{
  ReadSerial();

  if ( millis() - updte >= 20) //Was 30,  update each 20 ms
  {
    updte = millis();
    if (initialized)
    {
      doOm();
      Serial.flush();
    }
    else
    {
      req_init();
    }
  }
  if (millis() > 30000 && !auto_imu) //30 seconds for IMU stabilisation
  {
    odometer.useImu();
    auto_imu = true;
  }
  if (mpuInterrupt || fifoCount >= packetSize)
  {
    //if (mpuInterrupt)// || fifoCount >= packetSize) //Retrieve IMU data if interrupt or new packet available
    getIMUdata();
  }
}

// ================================================================
// ===                    FUNCTIONS                     ===
// ================================================================

void getIMUdata()
{
  /*
  Serial.print("p");
    Serial.print("\t");
    Serial.print(" Here I am ");
    Serial.print(millis());
    Serial.print("\n");
    */
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY MAIN PROGRAM LOOPMAIN PROGRAM LOOPshort wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /*
    Serial.print("p");
    Serial.print("\t");
    Serial.print(" Aqui estoy ");
    Serial.print(millis());
    Serial.print("\n");
    */
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;
  }
}
void imu_on()  //Turns on the IMU
{
  pinMode(IMU_ON, OUTPUT);
  digitalWrite(IMU_ON, LOW);
  delay(100);
  digitalWrite(IMU_ON, HIGH);
  delay(30);
}
void doOm() //Will work as robot
{
  if ( odometer.use_imu && !using_imu)
  {
    Serial.print("p");
    Serial.print("\t");
    Serial.print("Now using IMU");
    Serial.print("\n");
    using_imu = true;
    no_imu = false;
  }
  if ( !odometer.use_imu && !no_imu)
  {
    Serial.print("p");
    Serial.print("\t");
    Serial.print("Not using IMU");
    Serial.print("\n");
    no_imu = true;
    using_imu = false;
  }
  //Serial.print("Running");
  //Serial.print("\n");
  if ( millis() - timesincelastcommand > 10000)
  {
    //encodersPowerOn();
    //encoderSetup();
    //Serial.print("p");
    //Serial.print("\t");
    //Serial.print("Encoder reset due to no command in 10 seconds");
    //Serial.print("\n");
    //timesincelastcommand = millis();
    //flagzero = true;
  }

  if (millis() - timesincelastcommand > 1500 && !flagzero) //Stop motors after 1.5 seconds of no command
  {
    setpointzero();
    //encodersPowerOn();
    //encoderSetup();
    flagzero = true;
    Serial.print("p");
    Serial.print("\t");
    Serial.print("Set to Zero. No command received in 1.5 second");
    Serial.print("\n");
  }

  inputsetpoint(); //Get the inputs and setpoints
  pidcompute(); //Compute the speeds
  motorwrite(); //output for the motors
  odometer.update(); //Update the Odometry
  printodometry(); //Sends the Odometry information to the computer
  printultrasonic(); //Sends the ultrsonic information to the computer
  //*Just debugging*/printypr();

}
void printypr()
{

  Serial.print("p");
  Serial.print("\t");
  Serial.print("Yaw ");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print("Pitch ");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print("Roll ");
  Serial.print(roll);
  Serial.print("\n");

}
void encodersPowerOn() //Will start the encoders
{
  pinMode(encoders_on, OUTPUT);
  digitalWrite(encoders_on, LOW);
  delay(50);
  digitalWrite(encoders_on, HIGH);
  delay(50);
}
void encoderSetup() //Will setup the encoders
{
  //Wire.setClock(10000);
  RRencoder.init(MOTOR_269_ROTATIONS, MOTOR_269_TIME_DELTA);  //For these settings the output will be given in RPM
  RLencoder.init(MOTOR_269_ROTATIONS, MOTOR_269_TIME_DELTA);
  FLencoder.init(MOTOR_269_ROTATIONS, MOTOR_269_TIME_DELTA);
  FRencoder.init(MOTOR_269_ROTATIONS, MOTOR_269_TIME_DELTA);
  RLencoder.setReversed(true); //Function in case the encoders are not counting in the right direction
  FLencoder.setReversed(true);
  FLlastp = FLencoder.getPosition();
  FRlastp = FRencoder.getPosition();
  RRlastp = RRencoder.getPosition();
  RLlastp = RLencoder.getPosition();
}
//Setting up the motors
void motorsSetup()
{
  pinMode(24, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(28, OUTPUT);
  FR.attach(servoFR, 1000, 2000);
  FL.attach(servoFL, 1000, 2000);
  RR.attach(servoRR, 1000, 2000);
  RL.attach(servoRL, 1000, 2000);
}
void pidSetup() //Setting up the PID control
{

  //Test PID
  FLsetpoint = 0;
  FRsetpoint = 0;
  RRsetpoint = 0;
  RLsetpoint = 0;
  FLinput = InputPID(FLlastp, 4);
  FRinput = InputPID(FRlastp, 3);
  RRinput = InputPID(RRlastp, 1);
  RLinput = InputPID(RLlastp, 2);

  FLpid.SetMode(AUTOMATIC);
  FRpid.SetMode(AUTOMATIC);
  RRpid.SetMode(AUTOMATIC);
  RLpid.SetMode(AUTOMATIC);
  FLpid.SetOutputLimits(-75.0, 75.0);
  FRpid.SetOutputLimits(-75.0, 75.0);
  RRpid.SetOutputLimits(-75.0, 75.0);
  RLpid.SetOutputLimits(-75.0, 75.0);
}
void ReadSerial() //Read the serial for the Messenger
{
  while (Serial.available())
  {
    omMessenger.process(Serial.read());
  }
}

void OnReceived() //Will execute this everytime a message is received
{
  if (omMessenger.checkString("Startimu"))  //Initiate communication
  {
    initialized = true;

    Serial.print("Active");
    Serial.print("\n");
    return;
  }
  if (omMessenger.checkString("Startnoimu"))
  {
    initialized = true;
    auto_imu = true;
    Serial.print("Active");
    Serial.print("\n");
    return;
  }
  if (omMessenger.checkString("lscale"))
  {
    linearscaleR();
    Serial.print("p");
    Serial.print("\t");
    Serial.print("Received linear correction");
    Serial.print("\n");
    return;
  }
  if (omMessenger.checkString("ascale"))
  {
    angularscaleR();
    Serial.print("p");
    Serial.print("\t");
    Serial.print("Received angular correction");
    Serial.print("\n");
    return;
  }
  if (omMessenger.checkString("s")) // Get speed commands
  {
    SpeedReceived();
    return;
  }
  if (omMessenger.checkString("noimu"))
  {
    odometer.noImu();
    return;
  }
  if (omMessenger.checkString("imu"))
  {
    odometer.useImu();
    return;
  }

  /*
  if (omMessenger.checkString("odom"))
  {
    printodometry(); //Sends the Odometry information to the computer
    return;
  }
  if (omMessenger.checkString("ultra"))
  {
    printultrasonic();
    return;
  }*/
  // clear out unrecognized content
  while (omMessenger.available())
  {
    omMessenger.readInt();
  }
}

void req_init() //Requesting the initial message
{
  Serial.print("InitializeBaseController");
  Serial.print("\n");

}

void setpointzero() //Set the inputs to zero
{
  FLsetpoint = 0;
  FRsetpoint = 0;
  RLsetpoint = 0;
  RRsetpoint = 0;
}
void printultrasonic() //Send ultrasonic information to the computer
{
  //ultra_delay++;
  //if (ultra_delay > 5)
  //{
  Serial.print("u");
  Serial.print("\t");
  Serial.print(ultrasonic.Ranging(CM));
  Serial.print("\n");
  //  ultra_delay = 0;
  //}
}
void printodometry() //Sends odometry information to the computer
{
  /*
  sx = (int)(odometer.x * 1000);
  sy = (int)(odometer.y * 1000);
  sth = (int)(odometer.th * 1000);
  svx = (int)(odometer.vx * 1000);
  svy = (int)(odometer.vy * 1000);
  sw = (int)(odometer.w * 1000);
  Serial.print("o"); //Sends the Odometry values
  Serial.print("\t");
  Serial.print(sx);
  Serial.print("\t");
  Serial.print(sy);
  Serial.print("\t");
  Serial.print(sth);
  Serial.print("\t");
  Serial.print(svx);
  Serial.print("\t");
  Serial.print(svy);
  Serial.print("\t");
  Serial.print(sw);
  Serial.print("\n");
  */
  /////// Just for debugging receiving wrong data on computer

  Serial.print("o"); //Sends the Odometry values
  Serial.print("\t");
  Serial.print(odometer.x, 4);
  Serial.print("\t");
  Serial.print(odometer.y, 4);
  Serial.print("\t");
  Serial.print(odometer.th, 4);
  Serial.print("\t");
  Serial.print(odometer.vx, 4);
  Serial.print("\t");
  Serial.print(odometer.vy, 4);
  Serial.print("\t");
  Serial.print(odometer.w, 4);
  Serial.print("\n");

  /*Serial.print("p"); //Sends the Odometry values
  Serial.print("\t");
  Serial.print(odometer.x, 4);
  Serial.print("\t");
  Serial.print(odometer.y, 4);
  Serial.print("\t");
  Serial.print(odometer.th, 4);
  Serial.print("\t");
  Serial.print(odometer.vx, 4);
  Serial.print("\t");
  Serial.print(odometer.vy, 4);
  Serial.print("\t");
  Serial.print(odometer.w, 4);
  Serial.print("\t");
  Serial.print("time: ");
  Serial.print(millis());
  Serial.print("\n");*/
}
void inputsetpoint() //Get the current speed of the motors
{
  FLinput = InputPID(FLlastp, 4);
  FRinput = InputPID(FRlastp, 3);
  RRinput = InputPID(RRlastp, 1);
  RLinput = InputPID(RLlastp, 2);

  if (RLsetpoint > 85)
    RLsetpoint = 85;
  if (RLsetpoint < -85)
    RLsetpoint = -85;
  if (RRsetpoint > 85)
    RRsetpoint = 85;
  if (RRsetpoint < -85)
    RRsetpoint = -85;
  if (FRsetpoint > 85)
    FRsetpoint = 85;
  if (FRsetpoint < -85)
    FRsetpoint = -85;
  if (FLsetpoint > 85)
    FLsetpoint = 85;
  if (FLsetpoint < -85)
    FLsetpoint = -85;
}
void motorwrite() //Write the current values to the motors
{
  FL.write(90 + FLoutput);
  FR.write(90 + FRoutput);
  RR.write(90 + RRoutput);
  RL.write(90 + RLoutput);
}
void motorzero() //Function to stop the motors
{
  FL.write(90);
  FR.write(90);
  RR.write(90);
  RL.write(90);
}
void pidcompute() //Does the control computation
{
  FLpid.Compute();
  FRpid.Compute();
  RRpid.Compute();
  RLpid.Compute();
  if ( abs(FLsetpoint) < 5)
    FLoutput = 0;
  if ( abs(FRsetpoint) < 5)
    FRoutput = 0;
  if ( abs(RRsetpoint) < 5)
    RRoutput = 0;
  if ( abs(RLsetpoint) < 5)
    RLoutput = 0;
  /*
  Serial.print("p");
  Serial.print("\t");
  Serial.print(FLsetpoint, 3);
  Serial.print("\t");
  Serial.print(FRsetpoint, 3);
  Serial.print("\t");
  Serial.print(RRsetpoint, 3);
  Serial.print("\t");
  Serial.print(RLsetpoint, 3);
  Serial.print("\n");
  */
}
void SpeedReceived() //Manage the speed received
{
  timesincelastcommand = millis();
  flagzero = false;
  float cmd_vx = GetFloatFromBaseAndExponent(omMessenger.readInt(), omMessenger.readInt());
  float cmd_vy = GetFloatFromBaseAndExponent(omMessenger.readInt(), omMessenger.readInt());
  float cmd_w = GetFloatFromBaseAndExponent(omMessenger.readInt(), omMessenger.readInt());
  setSpeedWheels(cmd_vx, cmd_vy, cmd_w);
  /* Serial.print("p");
   Serial.print("\t");
   Serial.print("Speed received");
   Serial.print("\n");
  */
}
void linearscaleR()
{
  float lscale = GetFloatFromBaseAndExponent(omMessenger.readInt(), omMessenger.readInt());
  if (lscale > 0)
  {
    linearscale = lscale;
  }
}
void angularscaleR()
{
  float ascale = GetFloatFromBaseAndExponent(omMessenger.readInt(), omMessenger.readInt());
  if (ascale > 0)
  {
    angularscale = ascale;
  }
}

float GetFloatFromBaseAndExponent(int base, int exponent) //Get the actual value of number received
{
  return base * pow(10, exponent);
}

void setSpeedWheels(float _vx, float _vy, float _w) //This function will set the new speed through the kinematics class
{
  vx = _vx;
  vy = _vy;
  vth = _w;
  /*
  Serial.print("p");
  Serial.print("\t");
  Serial.print(vx, 3);
  Serial.print("\t");
  Serial.print(vy, 3);
  Serial.print("\t");
  Serial.print(vth, 3);
  Serial.print("\n");
  */
  kinematics.compute(); //Calculates the wheel speeds through the kinematics class

  /*
  Serial.print("p");
  Serial.print("\t");
  Serial.print(FLsetpoint, 3);
  Serial.print("\t");
  Serial.print(FRsetpoint, 3);
  Serial.print("\t");
  Serial.print(RRsetpoint, 3);
  Serial.print("\t");
  Serial.print(RLsetpoint, 3);
  Serial.print("\n");
  */
}
double InputPID(double lpos, int slct) //Get the correct value of the speed based on last and current position of the wheels
{
  double currentPosition = 0;
  double InPID = 0;
  double diffPosition = 0;
  int signo;
  switch (slct)
  {
    case 1:
      currentPosition = RRencoder.getPosition();
      RRlastp = currentPosition;
      break;
    case 2:
      currentPosition = RLencoder.getPosition();
      RLlastp = currentPosition;
      break;
    case 3:
      currentPosition = FRencoder.getPosition();
      FRlastp = currentPosition;
      break;
    case 4:
      currentPosition = FLencoder.getPosition();
      FLlastp = currentPosition;
      break;
  }
  diffPosition = currentPosition - lpos;
  if (diffPosition < 0)
  {
    signo = -1;
  }
  else if (diffPosition == 0)
  {
    signo = 0;
  }
  else    {
    signo = 1;
  }
  switch (slct)
  {
    case 1:
      InPID = RRencoder.getSpeed() * signo ;
      break;
    case 2:
      InPID = RLencoder.getSpeed() * signo ;
      break;
    case 3:
      InPID = FRencoder.getSpeed() * signo ;
      break;
    case 4:
      InPID = FLencoder.getSpeed() * signo ;
      break;
  }
  return InPID;
}
