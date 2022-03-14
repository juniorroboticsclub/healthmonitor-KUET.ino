#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
const int sensor = A2;
float tempc;
float tempf,vout;
#define vcc 7
#define gnd 8
const int gsr=A1;
int sensorv=0;
int gsrave=0;
int heartbeat = 0;
void setup() {
   Serial.begin(19200);  

   
    pinMode(sensor,INPUT);
    pinMode(vcc,OUTPUT);
    digitalWrite(vcc,HIGH);
    pinMode(gnd,OUTPUT);
    digitalWrite(gnd,LOW);
    
    
}

void loop() {
  
   vout=analogRead(sensor);
   vout=(vout*500)/1023;
   tempc=vout;
   tempf=(vout*1.8)+32;
  /*if(!accel.begin())
   {
      Serial.println("No ADXL345 sensor detected.");
      while(1);
   }*/

    long sum=0;
  for (int i=0;i<10;i++)
  {
    sensorv=analogRead(gsr);
    sum += sensorv;
    delay(5);
  }
  gsrave = sum/10;
  heartbeat = int(analogRead(0)/10) +20;
 
   sensors_event_t event; 
   accel.getEvent(&event);
   Serial.print(event.acceleration.x); Serial.print("     ");
   Serial.print(event.acceleration.y); Serial.print("     ");
   Serial.print(event.acceleration.z); Serial.print("     ");
  Serial.println("m/s^2");
   Serial.print(tempf);
   Serial.print("     ");
  Serial.print(gsrave);
   Serial.print("     ");
   Serial.print(heartbeat);
    Serial.print("     ");
  Serial.println();
   delay(500);
   

}
