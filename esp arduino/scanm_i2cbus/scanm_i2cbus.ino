//scan i2c bus
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_Si7021.h"
#include <Adafruit_LPS35HW.h>

#include <U8g2lib.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
Adafruit_Si7021 sensor = Adafruit_Si7021();
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

ADS myFlexSensor; //Create instance of the Angular Displacement Sensor (ADS) class

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

float mast_angle = 0;
float T[3];
float P[2];
float H[2];

unsigned long delayTime=1000;
TwoWire I2C = TwoWire(0);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Serial.println("\nDisplay...");
  u8g2.begin();
 
drawFontFaceDemo();
  
  Serial.println("\nI2C Scanner");
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }

  if (myFlexSensor.begin() == false)
  {
    Serial.println(F("No Bend sensor detected. Check wiring."));
    
  }
  bool status;

  //BME Pressure
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76, &I2C);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    
  }
  if (!lps35hw.begin_I2C()) {
    Serial.println("Couldn't find LPS35HW chip");

  }

   if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
   }
  delay(1000);}
 
void loop() { 
  printValues();
  printDisplay();
  delay(delayTime);
}

void printValues() {
  T[0] = bme.readTemperature();
  T[1] = sensor.readTemperature();
  T[2] = lps35hw.readTemperature();
  Serial.print("BME280 Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  P[0] = bme.readPressure() / 100.0F;
  P[1] = lps35hw.readPressure();
  Serial.print("BME280 Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  H[0] = bme.readHumidity()+25.9;
  H[1] = sensor.readHumidity();
  Serial.print("BME280 Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
  Serial.print("Si7021 Humidity:    ");
  Serial.print(sensor.readHumidity(), 2);
  Serial.print("\tSi7021 Temperature: ");
  Serial.println(sensor.readTemperature(), 2);
  
  Serial.print("lps35hw Temperature: ");
  Serial.print(lps35hw.readTemperature());
  Serial.println(" C");
  
  Serial.print("lps35hw Pressure: ");
  Serial.print(lps35hw.readPressure()+13.07);
  Serial.println(" hPa");

    if (myFlexSensor.available() == true)
  {
    mast_angle = myFlexSensor.getX();
    Serial.print("Bend Sensor (Mast): ");
    Serial.print(mast_angle);
    Serial.println();
  }
  }

void printDisplay() {
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_haxrcorp4089_tr      );  // choose a suitable font
  String s = "Temp :    "+String(T[0],1)+"   "+String(T[1],1)+"   "+String(T[2],1);
  u8g2.drawStr(0,10,s.c_str());
  s = "Pressure :   "+String(P[0],1)+" "+String(P[1],1);
  u8g2.drawStr(0,22,s.c_str());
  s = "Humidity :   "+String(H[0],1)+" "+String(H[1],1);
  u8g2.drawStr(0,34,s.c_str());

  s = "Mast: "+String(mast_angle,1);
  u8g2.drawStr(0,50,s.c_str());
  u8g2.sendBuffer(); // actually display all of the above
}

void drawFontFaceDemo() {
  u8g2.clearBuffer();          // clear the internal memory
  u8g2.setFont(u8g2_font_haxrcorp4089_tr      );  // choose a suitable font
   
  u8g2.drawStr(0,10,"Hello WORLD !:");
  u8g2.sendBuffer(); // actually display all of the above
}
