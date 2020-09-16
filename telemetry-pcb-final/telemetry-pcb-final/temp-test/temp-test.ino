#include <SD.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

#define BUZZ 7
#define BME_CS 9
#define SD_CS 10
#define MOSI 11
#define MISO 12
#define SCK 13

Adafruit_BME280 bme(BME_CS);

unsigned long DATA_POINTS;
const int data_frequency_hz = 100;
int counter = 0;
float temp = 0;
boolean ran = false;
File dataLog;

void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZ, OUTPUT); // Buzzer
  pinMode(BME_CS, OUTPUT); // BME CS
  pinMode(SD_CS, OUTPUT); // SD CS
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  // LOW = activate line; HIGH = deactivate line

  tone(BUZZ, 500);
  delay(1000);
  noTone(BUZZ);
  tone(BUZZ, 700);
  delay(500);
  noTone(BUZZ);
  delay(1000);

  Serial.begin(9600);
  SPI.begin();
  while(!Serial);
  DATA_POINTS = 1000;
  digitalWrite(SD_CS, HIGH);
  digitalWrite(BME_CS, LOW);
  
  
  if (!bme.begin()) {
    tone(BUZZ, 1000);
    delay(1000);
    noTone(BUZZ);
  }
  
  digitalWrite(BME_CS, HIGH);
  digitalWrite(SD_CS, LOW);
  
  if (!SD.begin(SD_CS)) {
    tone(BUZZ, 1000);
    delay(1000);
    noTone(BUZZ);
  }
  
 
  dataLog = SD.open("test.txt", FILE_WRITE);
  
}

void loop() {
  // put your main code here, to run repeatedly:
 while (counter < DATA_POINTS) {
    digitalWrite(SD_CS, HIGH);
    digitalWrite(BME_CS, LOW);
    
    temp = bme.readTemperature();
    
    digitalWrite(BME_CS, HIGH);
    digitalWrite(SD_CS, LOW);

    dataLog.println(temp);
    counter++;
    delay(DATA_POINTS / data_frequency_hz);
  }
  if ((counter >= DATA_POINTS) && !ran) {
    digitalWrite(SD_CS, LOW);
    dataLog.close();
    ran = true;
    digitalWrite(SD_CS, HIGH);
  }
  return;
} 
