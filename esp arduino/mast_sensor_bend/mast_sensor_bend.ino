//scan i2c bus
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_Si7021.h"
#include <Adafruit_LPS35HW.h>

#include <U8g2lib.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
#include <WiFi.h>

const char* ssid     = "YDWG";
const char* password = "12345678";
//const char* host     = "192.168.4.1";
const char* url      = "";
IPAddress hostIP(192, 168, 4, 1);

WiFiClient client;
int httpPort = 1457;
byte bdata[100];
String sbdata;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
Adafruit_Si7021 sensor = Adafruit_Si7021();
Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

ADS myFlexSensor; //Create instance of the Angular Displacement Sensor (ADS) class
const int FlexDataReadyPin = 19;
const int FlexResetPin = 18;
unsigned long samples = 0;
unsigned long lastmilli = 0;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

float mast_angle = 0;
float T[3];
float P[2];
float H[2];

unsigned long delayTime=40;
TwoWire I2C = TwoWire(0);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  Serial.println("\nCheck Display...");
  u8g2.begin();
 
drawFontFaceDemo();
  pinMode(FlexDataReadyPin, INPUT);
  pinMode(FlexResetPin, INPUT);
  //myFlexSensor.setResetPin(FlexResetPin);
  //myFlexSensor.hardwareReset();
  delay (500);

  //digitalWrite(FlexResetPin,HIGH);

  Serial.println("\nconnect Wifi...");
  wifiConnect();
  
  Serial.println("\nI2C Scanner...");
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
  } else {
    myFlexSensor.setSampleRate(50); 
    myFlexSensor.poll(); //Begin sensor outputting readings
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
  readWriteWeb();
  readValues();
  //printValues();
  Serial.println(mast_angle);
  printDisplay();
  delay(delayTime);
}

void readWriteWeb() {
  // Use WiFiClient class to create TCP connections
  if (!client.connect(hostIP, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  Serial.print("remoteIP: ");
  //Serial.println(client.remoteIP()+" : "+client.remotePort());

  //Serial.print("peek: ");
  //Serial.println(client.peek());
  
  client.println("09F11299 FF D0 08 D0 03 00 55 FD\n");  
  //client.println();
  delay(100);
  // Read all the lines of the reply from server and print them to Serial
  unsigned long numc = client.available();
  Serial.println("bytes available:"+String(numc));
  String line,scmd,sdata;
  
  int idx = 0;
  while (client.available()) {
    char c = client.read();
    if (c==0x0D) {
      int pos = line.indexOf("T ");
      if (pos <0) pos = line.indexOf("R ");
      scmd = line.substring(pos+2,23);
      sdata = line.substring(pos+11);
      ConvertHexStringToByteArray(sdata);
      //Serial.println(line);
      Serial.println(scmd+"--"+sdata+"---"+sbdata);
      line =""; 
    } else {
    line += c;
    
    }
  }
  Serial.println("EOF read:");

}

void readValues() {
  T[0] = bme.readTemperature();
  T[1] = sensor.readTemperature();
  T[2] = lps35hw.readTemperature();
  P[0] = bme.readPressure() / 100.0F;
  P[1] = lps35hw.readPressure();
  H[0] = bme.readHumidity();
  H[1] = sensor.readHumidity();
  
  if (myFlexSensor.available() == true)  //new data?
    {
      mast_angle = myFlexSensor.getX();
      samples++;
    } else {
      Serial.print("Bend Sensor DATA unavailable");
      Serial.println();
      //myFlexSensor.hardwareReset();
    }
  
}

void printValues() {
  Serial.print("BME280 Temperature = ");
  Serial.print(T[0]);
  Serial.println(" *C");
  
  Serial.print("BME280 Pressure = ");
  Serial.print(P[0]);
  Serial.println(" hPa");

  Serial.print("BME280 Humidity = ");
  Serial.print(H[0]);
  Serial.println(" %");

  //Serial.println();
  Serial.print("Si7021 Humidity:    ");
  Serial.print(H[1], 2);
  Serial.print("\tSi7021 Temperature: ");
  Serial.println(T[1], 2);
  
  Serial.print("lps35hw Temperature: ");
  Serial.print(T[2]);
  Serial.println(" C");
  
  Serial.print("lps35hw Pressure: ");
  Serial.print(P[1]);
  Serial.println(" hPa");

  Serial.print(samples);// / ((millis()-lastmilli) / 1000.0), 2);
  lastmilli = millis();
  //samples=0;
  Serial.print("samples");
  Serial.print(",");
  Serial.print("Bend Sensor (Mast): ");
  Serial.println(mast_angle);
  Serial.println();
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

void wifiConnect() {
 /* if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
    Serial.println("STA Failed to configure");
  }
*/
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(WiFi.status());
    if (WiFi.status()==6) ESP.restart();
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());
}  

void ConvertHexStringToByteArray(String hexString)
{

    int num = 8;
    sbdata="";
    for (int index = 0; index < num; index++)
    {
        String byteValue = "0x"+hexString.substring(index * 3, index * 3+2);
        //Serial.print(byteValue);
        bdata[index] = (byte)strtoul(byteValue.c_str(),0,16);
        char car[5];
        sprintf(car,"%02X",bdata[index]);
        sbdata+=String(car);

    }
    //Serial.println();

    
}
