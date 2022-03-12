

int pump = 9;
int sensor = A0;
int button = 7;
int out = 3;
int button_state;
int reading;
int diastol;
int systol;
float va;

void setup() {
  pinMode(pump, OUTPUT);
  pinMode(out, OUTPUT);
  pinMode(sensor, INPUT);
  pinMode(button, INPUT_PULLUP);
  Serial.begin(9600);

digitalWrite(out, HIGH);
}


void loop() {
  
analogWrite(pump,250);
reading = analogRead(sensor);
Serial.println(reading);

 //va = (reading * 500.0) / 1024.0; 
//Serial.println(va);


delay(1000);
digitalWrite(out, HIGH);

if(reading <= 190){       //200 
     
     delay(1000);
digitalWrite(pump, LOW);
     
     Systolic();
}

}

void Systolic(){

  for( int i = 0; i < 5; i++) {
Serial.println(i);
reading = analogRead(sensor);
Serial.println(reading);
delay(1000);
digitalWrite(pump, LOW);
   
    }

Diastolic();
delay(100);
}

void Diastolic()
{

    Serial.print("Diastolic :");
    diastol =346 - reading; //- 117;
    Serial.println(diastol);
    Serial.print("Systolic:");
    systol = 309 - reading;// - 157;
    Serial.println(systol);
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
Serial.println("STOP"); 
delay(1000);
digitalWrite(pump, LOW);
digitalWrite(out, LOW);
    }

    stopp();
     delay(10);
}
