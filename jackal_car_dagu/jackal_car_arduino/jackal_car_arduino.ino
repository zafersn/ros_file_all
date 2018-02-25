
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, Tx

#define _pinPwmB1  3 
#define _pinPwmB2  9

#define _pinPwmA2  6  //GERİ
#define _pinPwmA1  5  //İLER


String dizi[3];
void setup() {
    Serial.begin(57600); 
  

  Serial.setTimeout(10);
  
  delay(100);
 mySerial.begin(115200);
  mySerial.setTimeout(10);
  // put your setup code here, to run once:
 /* pinMode(_pinPwmB1,OUTPUT);
  pinMode(_pinPwmB2,OUTPUT);
  pinMode(_pinPwmA2,OUTPUT);
  pinMode(_pinPwmA1,OUTPUT);
  delay(100);*/

    mySerial.println("HELLO");
    
delay(100);

 // delay(100);


  //nh.getHardware()->setBaud(115200);
//  delay(100);

}

void loop() {

if (Serial.available()>0) {
   String gelenDeger = Serial.readString();                      
    //Serial.println(gelenDeger);
    int commaIndex = gelenDeger.indexOf(':');                     
   // int commaIndex2 = gelenDeger.indexOf("!");                  
// Serial.print("bluee");
 // Serial.println(gelenDeger);
   if (commaIndex != (-1 )) {            
      gelenDeger.trim();                                        
     
      dizi[0] = gelenDeger.substring(0, commaIndex);                     
      dizi[1] = gelenDeger.substring(commaIndex + 1);             
     // dizi[2] = gelenDeger.substring(commaIndex2 + 1);                    
      dizi[0].trim();                                                     
      dizi[1].trim();                                                     
      //dizi[2].trim();                                                     

      int X = (dizi[0].toInt());                                                  
      int Y = (dizi[1].toInt());                                        
     // int z = (dizi[2].toInt());  
      mySerial.print("X ");
     mySerial.print(X);
     mySerial.print(" Y");
     mySerial.println(Y);
    yonVer(X,Y);
    }

  
     //mySerial.println(gelenDeger);
   //  delay(1000);
  }
 // Serial1.print(" Y");
 
  
}
void yonVer(int X,int Y)
{
  int a = 0, b = 0,c = 0, d = 0;
  if(X<0)
    b = X *-1;
  else if(X>0)
    a = X;

  if(Y<0)
    c = Y *-1;
  else if(Y>0)
    d = Y;

  yonVer2(a, b,c,d);
}

void yonVer2(int a, int b,int c, int d) {

  analogWrite(_pinPwmB1, a);
  analogWrite(_pinPwmB2, b);

  analogWrite(_pinPwmA1, c);
  analogWrite(_pinPwmA2, d);

}
