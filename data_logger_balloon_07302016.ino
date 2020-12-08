/**********************************************************************************************************************************

 __   _   _____   _     _   _   _   _____        _____   _____   _____    _____        ___        _____   _____   _____    _____  
|  \ | | /  _  \ | |   / / | | | | /  ___/      |_   _| | ____| |  _  \  |  _  \      /   |      /  ___| /  _  \ |  _  \  |  _  \ 
|   \| | | | | | | |  / /  | | | | | |___         | |   | |__   | |_| |  | |_| |     / /| |      | |     | | | | | |_| |  | |_| | 
| |\   | | | | | | | / /   | | | | \___  \        | |   |  __|  |  _  /  |  _  /    / / | |      | |     | | | | |  _  /  |  ___/ 
| | \  | | |_| | | |/ /    | |_| |  ___| |        | |   | |___  | | \ \  | | \ \   / /  | |      | |___  | |_| | | | \ \  | |     
|_|  \_| \_____/ |___/     \_____/ /_____/        |_|   |_____| |_|  \_\ |_|  \_\ /_/   |_|      \_____| \_____/ |_|  \_\ |_|     

/**********************************************************************************************************************************
*Author   : Shawn Hartley
*
*Date     : 07/30/2016
*
*Revision   : 1
*
*Script   : nt_balloon_datalogger
*
*Description:
*
*This program will record data from a MEMSIC 2125 acceleromenter, BMP 280 temp and press sensor, MQ 4 gas sensor and write it to an sd card
*
*
*
**********************************************************************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

#define SCK 13
#define MISO 12
#define MOSI 11 
#define BMP_CS 10       //Chip select of bmp 280
#define SD_CS 9         //Chip select of SD
const int XPIN = 2;     // X output of the accelerometer
const int YPIN = 3;    // Y output of the accelerometer
const int MQ_4_PIN = A0; // Gas Sensor Output
int x_pulse;
int y_pulse;
int DELAY = 5000;


struct memsic_2125 {
  int accel_X;
  int accel_Y;
} accelerometer;

int read_accelerometer (int x, int y){
accelerometer.accel_X = ((x / 10) - 500) * 8;
accelerometer.accel_Y = ((y / 10) - 500) * 8;
}

void write_sd_card(int x_accel, int y_accel, float temp, float press, float alt, int gas){
  String out= String(x_accel) + "," + String(y_accel) + "," + String(temp) + "," + String(press) +
  "," + String(alt) + ","  + String(gas);
  File outfile = SD.open("test.txt" , O_CREAT | O_WRITE | O_APPEND);
  outfile.println(out);
  outfile.close();
  Serial.println(out);
  }

Adafruit_BMP280 bme(BMP_CS, MOSI, MISO, SCK);


void setup() {
  Serial.begin(9600);
  pinMode(XPIN, INPUT);
  pinMode(YPIN, INPUT);

   if (!bme.begin()) {  
    Serial.println(F("BMP_280 SENSOR ERROR"));
    while (true);
  }

 if (!SD.begin(SD_CS)) {
   Serial.println("SD CARD WAS NOT INITIALIZED");
   while (true);
  }
  
}

void loop() {
  
  x_pulse = pulseIn(XPIN, HIGH);
  y_pulse = pulseIn(YPIN, HIGH);
  read_accelerometer(x_pulse , y_pulse);
  write_sd_card(accelerometer.accel_X, accelerometer.accel_Y, bme.readTemperature(), bme.readPressure(), bme.readAltitude(1013.25), analogRead(MQ_4_PIN));
  delay(DELAY);
}


