#include "I2Cdev.h"
#include <SPI.h>
#include <SD.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


/*
  components:
  acceleromenter
  buzzerer
  sd
*/

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

String launchDataName;
bool isLaunching = false; // if false, standby
bool doneLaunching = false; // if false, launch hasn't happened yet

unsigned long timeStarted;
unsigned long currentTime;
double normacc;

MPU6050 mpu; // accelerometer
const int cs_sd = 4; // chip select SD
const int buzzer = 7; // buzzer pin

const double LAUNCH_THRESHOLD = 10000; // in ft/s^2
// g = 9.8 m/s^2 = 32 ft/s^2
const double TIME_LIMIT_SEC = 30;
File root;

// String dataName = "data_";

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

double normalize(double x, double y, double z) {
  return sqrt(x * x + y * y + z * z);
}


String fileName(String dataName) {
  // check how many existing files there are
  // create a new file name
  unsigned int count = 0;
  File entry;
  root = SD.open("/");
  while (true) {
    entry = root.openNextFile();
    if (!entry) {
      entry.close();
      return dataName + count;
    }
    if (!entry.isDirectory()) count++;
    entry.close();
  }
  return;
}

void setup() {
  // begin buzzer
  pinMode(buzzer, OUTPUT);

  // begin sd
  pinMode(cs_sd, OUTPUT);
  if (!SD.begin(cs_sd)) {
    tone(buzzer, 1000);
    delay(1000);
    tone(buzzer, 800);
    delay(1000);
    noTone(buzzer);
  } else {
    tone(buzzer, 2000);
    delay(1000);
    noTone(buzzer);
  }

  Serial.begin(9600);
  // begin mpu6050
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();

  if (mpu.testConnection()) {
    delay(1000);
    tone(buzzer, 500);
    delay(100);
    noTone(buzzer);
    delay(100);
    tone(buzzer, 500);
    delay(1000);
    noTone(buzzer);
    delay(1000);
  }

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(-53);
  mpu.setYGyroOffset(-23);
  mpu.setZGyroOffset(-6);
  mpu.setXAccelOffset(-983);
  mpu.setYAccelOffset(-3659);
  mpu.setZAccelOffset(1575);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  launchDataName = fileName("launch_") + ".txt";
}

void loop() {
  if (doneLaunching && SD.begin(cs_sd)) {
    for (int i = 0; i < 10; i++) {
      tone(buzzer, 500 + 100 * i);
      delay(100);
      noTone(buzzer);
      delay(100);
    }
    for (int i = 9; i >= 0; i--) {
      tone(buzzer, 500 + 100 * i);
      delay(100);
      noTone(buzzer);
      delay(100);
    }
  }
  
  if (!SD.begin(cs_sd) || !mpu.testConnection()) {
    return;
  }

  
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {

  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    // standby buzz
    if (!isLaunching) {
      
      normacc = normalize(aaReal.x, aaReal.y, aaReal.z);
      Serial.println((String(aaReal.x) + " " + String(aaReal.y) + " " + String(aaReal.z)));
      Serial.println(normacc);
      timeStarted = millis();
      Serial.println("time start: " + String(timeStarted));
      if (normacc > LAUNCH_THRESHOLD) {
        
        isLaunching = true;
        return;
      }
      
      tone(buzzer, 500);
      delay(100);
      noTone(buzzer);
      delay(1000);
  
      return;
    }
  }

  

  currentTime = millis();

  File dataFile = SD.open(launchDataName, FILE_WRITE);
  if (dataFile && isLaunching) {
    String dataString = "";
    

  
    dataString += String(q.w) + "\t" + String(q.x) + "\t" + String(q.y) + "\t" + String(q.z) + "\t";
    dataString += String(aaReal.x) + "\t" + String(aaReal.y) + "\t" + String(aaReal.z);
//  for (int count = 0; count < 64; count++) {
//    dataString += String(fifoBuffer[count]);
//  }

    dataString += "\t" + String(currentTime - timeStarted);
    dataFile.println(dataString);
//    Serial.println(dataString);
//    Serial.println(String(currentTime - timeStarted));
//    Serial.println(String(currentTime - timeStarted > TIME_LIMIT_SEC * 1000));
    dataFile.close();
  }

  
  if ((currentTime - timeStarted > TIME_LIMIT_SEC * 1000) && isLaunching) doneLaunching = true;
}




// for testing purposes
/*
  int fileName() {
  // check how many existing files there are
  // create a new file name
  unsigned int count = 0;
  File entry;
  root = SD.open("/");
  while (true) {
    entry = root.openNextFile();
    if (!entry) {
      entry.close();
      return count;
    }
    if (!entry.isDirectory()) count++;
    entry.close();
  }
  return;
  }
*/
