int sensor1=A1;
int sensor2=A2;
int sensor3=A3;
int sensor4=A4;
#define minimum 0
#define maxium 1023
float Map_sensor_value1,Map_sensor_value2,Map_sensor_value3,Map_sensor_value4;
#include <SoftwareSerial.h>     // import serial library
SoftwareSerial Bluetooth(8,9); //Bluetooth connection Tx – 8 and Rx-9

int measurePin = 0; //Connect dust sensor to Arduino A0 pin
int ledPower = 10;   //Connect 3 led driver pins of dust sensor to Arduino D2

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(sensor1,INPUT);
pinMode(sensor2,INPUT);
pinMode(sensor3,INPUT);
pinMode(sensor4,INPUT);
pinMode(ledPower,OUTPUT);
Bluetooth.begin(9600); 
}

void loop() {
  int sensor_value1=analogRead(A1);
  int sensor_value2=analogRead(A2);
  int sensor_value3=analogRead(A3);
  int sensor_value4=analogRead(A4);

//  Map_sensor_value1=map(sensor_value1,0,1023,0,100);
//  Map_sensor_value2=map(sensor_value2,0,1023,0,100);
//  Map_sensor_value3=map(sensor_value3,0,1023,0,100);
//  Map_sensor_value4=map(sensor_value4,0,1023,0,100);  
//  
float  Map_sensor_value1 = (float)((sensor_value1 - minimum) * 100) / (maxium - minimum);
float  Map_sensor_value2 = (float)((sensor_value2 - minimum) * 100) / (maxium - minimum);
float  Map_sensor_value3 = (float)((sensor_value3 - minimum) * 100) / (maxium - minimum);
float  Map_sensor_value4 = (float)((sensor_value4 - minimum) * 100) / (maxium - minimum);
digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); // read the dust value

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024.0);
  dustDensity = 170 * calcVoltage - 0.1;

  delay(1000);
  Serial.print("MQ2 sensor value= ");
  Serial.println( Map_sensor_value1);
  Serial.print("MQ7 sensor value= ");
  Serial.println( Map_sensor_value2);
  Serial.print("MQ135 sensor value= ");
  Serial.println( Map_sensor_value3);
  Serial.print("MQ136 sensor value= ");
  Serial.println( Map_sensor_value4);
  Serial.print("Dust sensor value= ");
  Serial.println(dustDensity); // unit: ug/m3
  Serial.println(" \n "); 
    
  Bluetooth.print( Map_sensor_value1);
  Bluetooth.print("|");
  Bluetooth.print( Map_sensor_value2);
  Bluetooth.print("|"); 
  Bluetooth.print( Map_sensor_value3);
  Bluetooth.print("|");
  Bluetooth.print( Map_sensor_value4);
  Bluetooth.print("|");
  Bluetooth.println(dustDensity);
  
  delay(1000);
if( Map_sensor_value1<=10 || Map_sensor_value2<=15 ||Map_sensor_value3<=25 ||Map_sensor_value4<=10 || dustDensity<=20){

  digitalWrite(2,HIGH);
  delay(1000);

  digitalWrite(2,LOW);
  delay(1000);
}
   else{
  digitalWrite(2,LOW);
    
   }
}
