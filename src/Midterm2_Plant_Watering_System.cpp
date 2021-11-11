/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/benja/Documents/IoT/My_Plant_Watering_System/src/Midterm2_Plant_Watering_System.ino"
/*
 * Project Midterm2_Plant_Watering_System
 * Description:Midterm 2 porject
 * Author:Ben Klein 
 * Date: 11-NOV-2021
 */

#include <SPI.h>
#include <Wire.h>
#include"Arduino.h"
#include"Air_Quality_Sensor.h"
#include "credentials.h"
#include "Adafruit_GFX.h"
#include <Adafruit_MQTT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SSD1306.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 

void setup();
void loop();
void readMoist();
void thirstyPlant();
void BME280Values();
void MQTT_connect();
void listen();
void sendData();
void runPump();
void stopPump();
void dustSensor();
void aqsLoop();
void OLEDdisplay();
#line 21 "c:/Users/benja/Documents/IoT/My_Plant_Watering_System/src/Midterm2_Plant_Watering_System.ino"
const int MOIST = A0;
const int LED = D7;
const int PUMP = D2;
const int DUST = D3;

int thirsty, hum, temp, waterMe;

unsigned long last, lastTime, startPumpTime, autoLastTime, displayTime, bmeTime, readTime, dustTime;

float press;

bool pumpOn;

//SEEED GROVE DUST
float ratio = 0;
float concentration = 0;

//SEEED GROVE AQS 1.3
AirQualitySensor sensor(A1);
int current_quality =-1;


#define OLED_RESET D4
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; 

Adafruit_SSD1306 display(OLED_RESET);\



TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Publish tempObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");
Adafruit_MQTT_Publish pressObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
Adafruit_MQTT_Publish humObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish thirstyObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture-level");
Adafruit_MQTT_Publish dustyObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/dust"); 
Adafruit_MQTT_Publish aqObj1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/air-quality");
Adafruit_MQTT_Subscribe waterObj1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/water");

SYSTEM_MODE(SEMI_AUTOMATIC);


void setup() {
Serial.begin(9600);
waitFor(Serial.isConnected, 15000); //wait for Serial Monitor to startup

//Moisture Seneor Read Timer Setup
readTime = millis();

//BME280 setup
unsigned status;
  status = bme.begin();
  if (!status){
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1);
  }
bmeTime = millis();

pinMode(MOIST, INPUT);
pinMode(LED, OUTPUT);
pinMode(PUMP, OUTPUT);

//DUST SENSOR SETUP

dustTime = millis();
pinMode(DUST, INPUT);

//AQS 1.3 setup
Serial.println("Waiting sensor to init...");
delay(10000);
if (sensor.init()) {
 Serial.println("Sensor ready.");
}
else {
 Serial.println("Sensor ERROR!");
}

//OLED setup
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
display.display(); // show splashscreen
delay(2000);
display.clearDisplay();// clears splashscreen 
display.display();
displayTime = millis();

//Connect to WiFi without going to Particle Cloud
WiFi.connect();
  while(WiFi.connecting()) {
  Serial.printf(".");
  }

// Setup MQTT subscription for onoff feed.
 mqtt.subscribe(&waterObj1);
}

void loop() {

MQTT_connect();
listen();
BME280Values();
OLEDdisplay();
aqsLoop();
readMoist();
thirstyPlant();
stopPump();
dustSensor();
sendData();
}

void readMoist(){
// Serial.printf("readMoist %i\n", millis());
  if (millis()-readTime >5000){
    thirsty = analogRead(MOIST);
    // Serial.printf("Sensor reading %i\n", thirsty);
    readTime = millis();
  }
}

void thirstyPlant(){
// Serial.printf("thirstyPlant %i\n", millis());
//change thirsty to about 2500 when you want to run automode
  if(millis()-autoLastTime > 3600000 && thirsty >= 2500) {
  runPump();
  autoLastTime = millis();
  }
}

void BME280Values(){
// Serial.printf("BME280Values %i\n", millis());
  if(millis()-bmeTime > 5000){ 
   temp = bme.readTemperature() * 9 / 5 + 32;
   press = bme.readPressure() / 100;
   hum = bme.readHumidity();
   bmeTime = millis();
   // Serial.printf(" Temp is %.2f fahrenheit\n Pressure is %.2fhPa\n Humidity is %i percent\n", temp, press, hum);
  }
}

void MQTT_connect() {
// Serial.printf("MQTT_Connect %i\n", millis());
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  // Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  // Serial.printf("MQTT Connected!\n");
}

void listen(){
// Serial.printf("listen %i\n", millis());
Adafruit_MQTT_Subscribe *subscription;
//readSubscription was set to 1000 and was causing a mandatory 1second delay, tested 200 millis ok
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &waterObj1) {
      waterMe = atof((char *)waterObj1.lastread);
      if (waterMe == true){
        digitalWrite(LED, HIGH);
        // Serial.printf("input recieved\n"); 
        runPump();
      }
      else{
        digitalWrite(LED, LOW);  
      }  
    }
  }
}

void sendData(){ // Validate connected to MQTT Broker
// Serial.printf("sendData %i\n", millis());
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  // publish to cloud every minute
  if((millis()-lastTime > 60000)) {
    if(mqtt.Update()) {
      tempObj1.publish(temp);
      pressObj1.publish(press);
      humObj1.publish(hum);
      thirstyObj1.publish(thirsty);
      dustyObj1.publish(concentration);
      aqObj1.publish(sensor.getValue());
      // Serial.printf(" Publishing\n temp %i\n pressure %0.2f\n humidity %i\n dryness %i\n",temp, press, hum, thirsty); 
      } 
    lastTime = millis();
  }
}

void runPump(){
  // Serial.printf("runPump %i\n", millis());
  pumpOn = true;
  digitalWrite(PUMP, HIGH);
  startPumpTime = millis();
  // Serial.printf("start pump time %i\n", startPumpTime);
}

void stopPump(){
  if(millis() - startPumpTime >500){
  pumpOn = false;
  digitalWrite(PUMP, LOW);
  // Serial.printf("stop pump time %i\n", millis());
  }
}

void dustSensor(){

unsigned long duration, starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;

  if (!pumpOn && millis()-dustTime > 3600000){
    // Serial.printf("dustSensor %i\n", millis());
    starttime = millis();
    while(millis()-starttime < sampletime_ms){
      duration = pulseIn(DUST, LOW);
      lowpulseoccupancy = lowpulseoccupancy+duration;
      ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
      concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
      //  Serial.printf("lowpulseoccupancy %i, ratio %0.2f, concentration %0.2f \n", lowpulseoccupancy, ratio, concentration);
      lowpulseoccupancy = 0;
      }
    dustTime=millis();
  }
}

void aqsLoop(){
// Serial.printf("aqsLoop %i\n", millis());
  int quality = sensor.slope();
// Serial.printf("AQS: %i\n", sensor.getValue());
}

void OLEDdisplay(){
// Serial.printf("OLED %i\n", millis());
  if(millis()-displayTime > 5000){ 
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.printf(" thirst-meter %i\n temp %i %cF \n pressure %0.2fhPa\n humidity %i %c\n Air QUality %i\n Dust %0.2f\n", thirsty, temp, 248,  press, hum, 37, sensor.getValue(), concentration);
    display.display();
    displayTime = millis();
  }
}