/*
Postnkov Aleksey
1 arduino ;
both_wheels script;
constant speed.
resistors 200ohm and 9Kohm;
to gyro just 2 wires, gnd and +5V;
1 big gnd connected with both wheels and arduino;
*/
#include <SoftwareSerial9.h>

#define MOSI 11
#define MISO 12
#define TX MOSI
#define RX MISO
#define TX1 13
#define RX1 14
#define WHEEL1 1
#define WHEEL2 1 //batterywheel
//#define LEDPIN 13
#define JOYSTICK 1

SoftwareSerial9 mySerial(RX,TX);
SoftwareSerial9 mySerial1(RX1,TX1);
int batterywheel = A0;    // select the input pin for the potentiometer
int boardwheel = A1;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
void setup() {
  mySerial.begin(26315);
  mySerial1.begin(26315);
  Serial.begin(115200);
}

char c = ' ';
signed int sp=200;
signed int sp2=100;
int s=255;
int i=1;

void loop() {
  //Serial.println(c);
  if (JOYSTICK){
    sp = (analogRead(boardwheel) - 520)/2;
    sp2= (analogRead(batterywheel)-550)/2;
  }
  //sp = sensorValue - 520;
  //Serial.print("speed ");
  Serial.print(sp);
  Serial.print("  ");
  Serial.println(sp2);
  /*Serial.print(" low byte "); //0110 0100
  Serial.print((sp & 0xFF), BIN);
  Serial.print(" high byte "); // 0000 0000
  Serial.println((sp >> 8) & 0xFF, BIN);
  *///Serial.println((~s)& 0xFF, BIN);
  //Serial.println("_________________________________________");
  
  
  
  //do {
      if (WHEEL1)
        mySerial.write9((~s)& 0xFF); // 100110100
      if (WHEEL2)
        mySerial1.write9((~s)& 0xFF); // 100110100
      if (WHEEL1)
        mySerial.write9((~s)& 0xFF); // 100110100
      if (WHEEL2)
        mySerial1.write9((~s)& 0xFF); // 100110100
      
      if (WHEEL1)
        mySerial.write9(256); // 1 0000 0000
   if (WHEEL2)
        mySerial1.write9(256); // 1 0000 0000
      
   if (WHEEL1)
        mySerial.write9(sp & 0xFF); // 0110 0100*/
   if (WHEEL2)
       mySerial1.write9(sp2 & 0xFF); // 0110 0100*/
   if (WHEEL1)
        mySerial.write9((sp >> 8) & 0xFF); // 0000 0000
   if (WHEEL2)
      mySerial1.write9((sp2 >> 8) & 0xFF); // 0000 0000
      
   if (WHEEL1)
        mySerial.write9(sp & 0xFF);  // 0110 0100
   if (WHEEL2)
       mySerial1.write9(sp2 & 0xFF);  // 0110 0100
   if (WHEEL1) 
        mySerial.write9((sp >> 8) & 0xFF); // 0000 0000
   if (WHEEL2)
       mySerial1.write9((sp2 >> 8) & 0xFF); // 0000 0000
   if (WHEEL1) 
        mySerial.write9(85); //10101010
   if (WHEEL2)
        mySerial1.write9(85); //10101010
  
      //delayMicroseconds(100);
      
  //} while(!Serial.available());
  s=s-1;
  if (s==0) s=255;
  //if (sp>800) i=i*-1;
  //if (sp<200) i=i*-1;
  //c=Serial.read();
}
