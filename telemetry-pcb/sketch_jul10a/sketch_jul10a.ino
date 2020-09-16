#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <SPI.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SD_CS 9

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS);
  
Sd2Card card;
SdVolume volume;
SdFile root;

const int bme_cs = 10;
const int sd_cs = 9; 
unsigned long DATA_POINTS;
const int data_frequency_hz = 100;

File data_log;

boolean ran = false;

void setup() {
  // put your setup code here, to run once:
  unsigned status;
  pinMode(BME_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(BME_SCK, OUTPUT);
  pinMode(BME_MISO, INPUT);
  pinMode(BME_MOSI, OUTPUT);
  Serial.begin(9600);
  SPI.begin();
  digitalWrite(SD_CS, HIGH);
  digitalWrite(BME_CS, LOW);
  while(!Serial);
  DATA_POINTS = 1000;

 
  status = bme.begin(); //bme begin
  if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1);
  }
  digitalWrite(BME_CS, HIGH);  
  digitalWrite(SD_CS, LOW);
  
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    while(1);
  }
  Serial.println("initialization done.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  //String fileName = "flight" + String(DATA_POINTS) + "_" + String(data_frequency_hz);
  String fileName = "test.txt";
  //Serial.println(fileName + ".txt");
  data_log = SD.open(fileName, FILE_WRITE);
 
  // if the file opened okay, write to it:
  //Serial.println(data_log);
  if (data_log) {
    Serial.print("writing to file: ");
    Serial.println(fileName + ".txt");
    digitalWrite(SD_CS, HIGH);
    digitalWrite(BME_CS, LOW);
    // if the file didn't open, print an error:
    
  } else {
    Serial.println("error opening " + fileName + ".txt");
    data_log.close();
    while(1);
  }
  
}

float sensorVals[] = {0, 0, 0};
int counter = 0;

void loop() {
  // put your main code here, to run repeatedly:
  while (counter < 1000) {
    digitalWrite(SD_CS, HIGH);
    digitalWrite(BME_CS, LOW);
    
    /*
    data_point.concat(String(bme.readTemperature()));
    data_point.concat("\t");
    data_point.concat(String(bme.readPressure() / 1000.0F));
    data_point.concat("\t");
    data_point.concat(String(bme.readHumidity()));
    */

    sensorVals[0] = bme.readTemperature();
    sensorVals[1] = bme.readPressure() / 1000.0F;
    sensorVals[2] = bme.readHumidity();
    
    digitalWrite(BME_CS, HIGH);
    digitalWrite(SD_CS, LOW);
    /*
    data_log.println(DATA_POINTS + "/" + data_frequency_hz); // num of pts / freq = recording duration (s)
    data_log.print(data_point.concat("\n"));
    */
    data_log.print(String(sensorVals[0]) + ",");
    data_log.print(String(sensorVals[1]) + ",");
    data_log.println(sensorVals[2]);
    digitalWrite(SD_CS, HIGH);
    Serial.print(counter + ": ");
    Serial.print(sensorVals[0]);
    Serial.print(",");
    Serial.print(sensorVals[1]);
    Serial.print(",");
    Serial.println(sensorVals[1]);
    counter++;
    delay(1000 / data_frequency_hz);
    
    
  }
  if ((counter >= DATA_POINTS) && !ran) {
    digitalWrite(SD_CS, LOW);
    data_log.close();
    Serial.println("data recording finished");
    ran = true;
    digitalWrite(SD_CS, HIGH);
  }
  return;
}
