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
int spo2 = 97;


int pump = 9;
int sensor1 = A0;
int button = 12;
int out = 3;
int button_state;
int reading;
int diastol;
int systol;
float va;


void setup() {
   Serial.begin(19200);  


    pinMode(sensor,INPUT);
    pinMode(vcc,OUTPUT);
    digitalWrite(vcc,HIGH);
    pinMode(gnd,OUTPUT);
    digitalWrite(gnd,LOW);
   
  
  
  pinMode(pump, OUTPUT);
  pinMode(out, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(button, INPUT_PULLUP);
 

digitalWrite(out, HIGH);

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
  
  
  
  
  analogWrite(pump,250);
reading = analogRead(sensor1);
//Serial.println(reading);

 //va = (reading * 500.0) / 1024.0; 
//Serial.println(va);


delay(1000);
digitalWrite(out, HIGH);

if(reading <= 190){       //200 
     
     delay(1000);
digitalWrite(pump, LOW);
     
     Systolic();
}

   sensors_event_t event; 
   accel.getEvent(&event);
   Serial.print(event.acceleration.x); 
   Serial.print("     ");
   Serial.print(event.acceleration.y); 
   Serial.print("     ");
   Serial.print(event.acceleration.z); 
   Serial.print("     ");
   Serial.print("m/s^2");
   Serial.print("     ");
   Serial.print(tempf);
   Serial.print("     ");
   Serial.print(gsrave);
   Serial.print("     ");
   Serial.print(heartbeat);
   Serial.print("     ");
   Serial.print(diastol);
   Serial.print("     ");
   Serial.print(systol);
   Serial.print("     ");
   Serial.println(spo2);
   Serial.print("     ");
   Serial.println();
   delay(500);


}




void Systolic(){

  for( int i = 0; i < 5; i++) {
//Serial.println(i);
reading = analogRead(sensor1);
//Serial.println(reading);
delay(1000);
digitalWrite(pump, LOW);
   
    }

Diastolic();
delay(100);
}

void Diastolic()
{

    //Serial.print("Diastolic :");
    diastol =346 - reading; //- 117;
    //Serial.println(diastol);
    //Serial.print("Systolic:");
    systol = 309 - reading;// - 157;
   // Serial.println(systol);
    delay(1000);
    digitalWrite(pump, LOW);

    button_state = digitalRead(button);
    if(button_state == LOW){
    stopp(); 
    delay(100); 
}
    Diastolic();    
}



 void stopp(){

for( int i = 0; i < 10; i++) {
//Serial.println("STOP"); 
delay(1000);
digitalWrite(pump, LOW);
digitalWrite(out, LOW);
    }

    stopp();
     delay(10);
}
