#include <Sleep_n0m1.h>
#include <math.h>
#include <SPI.h>
#include <SD.h>
#include <MsTimer2.h>
#include <ResponsiveAnalogRead.h>
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
unsigned long sleepTime;

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
ResponsiveAnalogRead anemometer(sensorPin, true, 0.01);
int anemometerValue;
double windSpeed;
int readingCount = 1;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  sleepTime = 300000;
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
  
  // initialize devices
  Serial.println(F("Initializing SD card..."));

  if (!SD.begin(chipSelect)) {
    Serial.println(F("SD card initialization failed!"));
    sdCheck = 1;
    }
  else {
    Serial.println(F("SD card initialization complete!"));
  }
  
  Serial.println(F("Initializing MPU6050..."));
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  pinMode(sensorPin, INPUT);

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
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

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for MPU6050 to stabilize..."));
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
  
  delay(30000);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  uint32_t period = 20000;
  dataLog = SD.open("datalog.txt", FILE_WRITE);
  
  Serial.print(F("Reading Count: "));
  Serial.println(readingCount);
  //Log read count
  dataLog.print(F("Reading Count: "));
  dataLog.println(readingCount);
  
  //Run capture routine
  for( uint32_t tStart = millis();  (millis()-tStart) < period;  ){
  readAnem();
  readMPU();
  SDLog();
  Serial.print(millis());
  Serial.print(F("\tWindspeed = ")); Serial.print(windSpeed,2);
  Serial.print(F("\tAcX: ")); Serial.print(aaReal.x);
  Serial.print(F("\tAcY: ")); Serial.print(aaReal.y);
  Serial.print(F("\tAcZ: ")); Serial.print(aaReal.z);
  Serial.print(F("\tPitch: ")); Serial.print(ypr[1] * 180/M_PI);
  Serial.print(F("\tRoll ")); Serial.println(ypr[2] * 180/M_PI);
  
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
  //mpu.resetFIFO(); 
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
      Serial.println(F("FIFO overflow!"));
      //dataLog.println(F("FIFO overflow!"));

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
  
  dataLog.print(millis());
  dataLog.print(F(","));
  dataLog.print(windSpeed);
  dataLog.print(F(","));
  dataLog.print(aaReal.x);
  dataLog.print(F(","));
  dataLog.print(aaReal.y);
  dataLog.print(F(","));
  dataLog.print(aaReal.z);

//  Serial.println("Acc written");

  dataLog.print(F(","));
  dataLog.print(ypr[1] * 180/M_PI);
  dataLog.print(F(","));
  dataLog.println(ypr[2] * 180/M_PI);

//  Serial.println("PR written");
//  Serial.println(millis());
  //mpu.resetFIFO();
}
