#include <Sleep_n0m1.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Sleep sleep;
unsigned long sleepTime = 600000;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool sdCheck = 0;
File dataLog;
const int sensorPin = A0;
const int chipSelect = 10;
int anemometerValue;
float windSpeed;
int readingCount = 1;
uint32_t period = 60000;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize devices
  while (!SD.begin(chipSelect));

  dataLog = SD.open("datalog.txt", FILE_WRITE);
  dataLog.println(F("SD card initialized."));
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  dataLog.println(mpu.testConnection() ? F("MPU6050 connection successful.") : F("MPU6050 connection failed."));
  pinMode(sensorPin, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(103);
  mpu.setYGyroOffset(33);
  mpu.setZGyroOffset(38);
  mpu.setXAccelOffset(-612);
  mpu.setYAccelOffset(-858);
  mpu.setZAccelOffset(781); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    dataLog.println(F("DMP enabled."));
    
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dataLog.println(F("Interrupt detection enabled."));
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    dataLog.println(F("DMP ready. Waiting for MPU6050 to stabilize..."));
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    dataLog.print(F("DMP Initialization failed (code "));
    dataLog.print(devStatus);
    dataLog.println(F(")"));
    }
    
  dataLog.close();
  delay(30000);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  
  dataLog = SD.open("datalog.txt", FILE_WRITE);
    
  //Run capture and log routines
  dataLog.print(F("Reading Count: "));
  dataLog.println(readingCount);
  for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
  readAnem();
  readMPU();
  SDLog();
  } 

  dataLog.close();
  readingCount ++;
  delay(100);
  sleep.pwrDownMode();
  sleep.sleepDelay(sleepTime);  
}

void readAnem(){

  //read anemometer data
  anemometerValue = analogRead(sensorPin);
  windSpeed = (float)anemometerValue;
  windSpeed = (((windSpeed-77)*5)/1024)*20.25; 
}

// ================================================================
// ===                    DATA CAPTURE LOOP                     ===
// ================================================================
void readMPU(){

  mpu.resetFIFO();
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    #endif
    
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
    #endif

    //mpu.resetFIFO();
  }  
}

// ================================================================
// ===                      DATA LOG LOOP                       ===
// ================================================================
void SDLog(){
  //Log read count
  dataLog.print(millis());
  dataLog.print(F(","));
  dataLog.print(windSpeed);
  dataLog.print(F(","));
  dataLog.print(aaReal.x);
  dataLog.print(F(","));
  dataLog.print(aaReal.y);
  dataLog.print(F(","));
  dataLog.print(aaReal.z);
  dataLog.print(F(","));
  dataLog.print(ypr[1] * 180/M_PI);
  dataLog.print(F(","));
  dataLog.println(ypr[2] * 180/M_PI);
}
