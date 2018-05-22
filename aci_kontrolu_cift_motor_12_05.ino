/*
   motor_hareket(motor_numarası,pwm_sinyali_motor_yönü);
 
 motor_numarası
 1 = P2 karta göre sağ
 2 = P1 karta göre sol
 
 motor_yönü
 0 = motorları durdur
 1 = saat yönünün tersine
 2 = saat yönünde
 
 Örnek 
 motor_hareket(1,250,2); sağ motor 250 pwm de saat yönünde döndür
 */
#include <PinChangeInt.h>
#include "PID_v1.h" 

#define en1 6
#define en2 5
#define m11 10
#define m12 9
#define m21 8
#define m22 7
#define senseA A0
#define senseB A1
#define enkoder1A 2
#define enkoder1B A3
#define enkoder2A 3
#define enkoder2B A4

float vin1 = 0.0;
float vin2 = 0.0;
float r1= 0.22;
float akim1=0.0;
float akim2=0.0;

volatile long   darbe=0;
volatile long   darbe2=0;

unsigned long last_time;
long int sayac=0;
double referans= 0;
double referans2= 0;
double sistem_konum;
double sistem_konum2;
double kontrol_sinyali;
double kontrol_sinyali2;

boolean durum=false;
int veri=0;


float girilen_deger[2];

PID DC_hiz(&sistem_konum,&kontrol_sinyali,&referans,150,0.2,0.8,DIRECT);
PID DC_hiz2(&sistem_konum2,&kontrol_sinyali2,&referans2,95,0.2,1,DIRECT);

void setup()
{
  Serial.begin(115200); 
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(m11,OUTPUT);
  pinMode(m12,OUTPUT);
  pinMode(m21,OUTPUT);
  pinMode(m22,OUTPUT);

  pinMode(enkoder1A,INPUT_PULLUP);
  pinMode(enkoder1B,INPUT_PULLUP);
  pinMode(enkoder2A,INPUT_PULLUP);
  pinMode(enkoder2B,INPUT_PULLUP);


  attachPinChangeInterrupt(enkoder1A,encoder_kesme_1a,FALLING); 
  attachPinChangeInterrupt(enkoder1B,encoder_kesme_1b,CHANGE); 
  attachPinChangeInterrupt(enkoder2A,encoder_kesme_2a,CHANGE); 
  attachPinChangeInterrupt(enkoder2B,encoder_kesme_2b,CHANGE); 
  DC_hiz.SetOutputLimits(-255,255); 
  DC_hiz.SetSampleTime(5); 
  DC_hiz.SetMode(AUTOMATIC);
  DC_hiz2.SetOutputLimits(-255,255); 
  DC_hiz2.SetSampleTime(5); 
  DC_hiz2.SetMode(AUTOMATIC);
  last_time=millis();



}
void loop()
{



  float olc1=analogRead(senseA);
  vin1=(olc1*1.1)/1024.0;
  akim1=vin1/r1;

  float olc2=analogRead(senseB);
  vin2=(olc2*1.1)/1024.0;
  akim2=vin2/r1;


  switch (durum)
  {

  case false:
    motor_hareket(1, 1 ,0 );
    motor_hareket(2, 130,1 );
    delay(1000);
    motor_hareket(1, 1 ,0 );
    motor_hareket(2, 130,2 );
    delay(1000);
    motor_hareket(1, 1 ,0 );
    motor_hareket(2, 130,1 );
    delay(1000);
    motor_hareket(1, 1 ,0 );
    darbe2=0;
    darbe=0;
    referans2=45;   
    referans=0;
    darbe2=0;
    darbe=0;
    durum=true;

    break;

  case true:


    if(Serial.available()) 
    {
      for(int i=0;i<2;i++){
        girilen_deger[i]=Serial.parseFloat();
      }

      /* if(referans+2>sistem_konum>referans-2 && referans2+2>sistem_konum2>referans2-2 ){
       
       veri=1;
       
       Serial.println(veri);
       }
       
       
       if(veri==1){*/


      referans=sistem_konum+girilen_deger[0];
      int y=sistem_konum2+girilen_deger[1];
      if(y<5){
        referans2=5;
      }
      else if(y>105){
        referans2=105;
      }
      else{
        referans2=y;
      }
      veri==0;
    }


    //}

    DC_hiz.Compute();
    DC_hiz2.Compute(); 
    //////////

    if(kontrol_sinyali < 0)   
    {
      motor_hareket(1, abs(kontrol_sinyali),1 );
    }
    else                  
    {
      motor_hareket(1, kontrol_sinyali,2 );
    }

    ///////////

    if(kontrol_sinyali2 < 0)   
    {
      motor_hareket(2, abs(kontrol_sinyali2),1 );
    }
    else                  
    {
      motor_hareket(2, kontrol_sinyali2,2 );
    }

    ////////////

    sistem_konum=int(360*((darbe/(4*46*37.0-1))));
    sistem_konum2=int(360*((darbe2/(4*46*10.0-1))));




    if(millis()-last_time>100)
    {

      Serial.print("Motor1-->");    
      Serial.print("Mevcut:");        
      Serial.print(sistem_konum);
      Serial.print("  ");  
      Serial.print("Istenen:");         
      Serial.print(referans);
      Serial.print("  "); 
      Serial.print("PID:");          
      Serial.print(kontrol_sinyali);
      Serial.print("  "); 
      Serial.print("Akim");        
      Serial.println(akim1);

      Serial.print("Motor2-->");    
      Serial.print("Mevcut:");        
      Serial.print(sistem_konum2);
      Serial.print("  ");  
      Serial.print("Istenen:");         
      Serial.print(referans2);
      Serial.print("  "); 
      Serial.print("PID:");          
      Serial.print(kontrol_sinyali2);
      Serial.print("  "); 
      Serial.print("Akim");        
      Serial.println(akim2);


      Serial.flush();
      last_time=millis();
      sayac++;

    }


    break;

  }



}



void encoder_kesme_1a()
{
  if( digitalRead(enkoder1B) == 0 ) {
    if ( digitalRead(enkoder1A) == 0 ) {
      darbe--; 
    } 
    else {
      darbe++;
    }
  }
  else {
    if ( digitalRead(enkoder1A) == 0 ) {
      darbe++; 
    } 
    else {
      darbe--; 
    }
  }
}


void encoder_kesme_1b(){

  if ( digitalRead(enkoder1A) == 0 ) {
    if ( digitalRead(enkoder1B) == 0 ) {
      darbe++; 
    } 
    else {
      darbe--;
    }
  } 
  else {
    if ( digitalRead(enkoder1B) == 0 ) {
      darbe--;
    } 
    else {
      darbe++; 
    }
  }
}
void encoder_kesme_2a()
{
  if( digitalRead(enkoder2B) == 0 ) {
    if ( digitalRead(enkoder2A) == 0 ) {
      darbe2--; 
    } 
    else {
      darbe2++; 
    }
  }
  else {
    if ( digitalRead(enkoder2A) == 0 ) {
      darbe2++; 
    } 
    else {
      darbe2--; 
    }
  }
}

void encoder_kesme_2b()
{ 
  if ( digitalRead(enkoder2A) == 0 ) {
    if ( digitalRead(enkoder2B) == 0 ) {
      darbe2++;
    } 
    else {
      darbe2--; 
    }
  } 
  else {
    if ( digitalRead(enkoder2B) == 0 ) {
      darbe2--; 
    } 
    else {
      darbe2++; 
    }
  }
}

void motor_hareket(int motor, int pwm, int yon )
{
  switch (motor) {
  case 1:

    switch (yon) {
    case 0:
      analogWrite(en1,pwm);
      digitalWrite(m11,0);
      digitalWrite(m12,0);
      break;
    case 1:
      analogWrite(en1,pwm);
      digitalWrite(m11,1);
      digitalWrite(m12,0); 
      break;
    case 2:
      analogWrite(en1,pwm);
      digitalWrite(m11,0);
      digitalWrite(m12,1); 
      break;
    }

    break;
  case 2:
    switch (yon) {
    case 0:
      analogWrite(en2,pwm);
      digitalWrite(m21,0);
      digitalWrite(m22,0);
      break;
    case 1:
      analogWrite(en2,pwm);
      digitalWrite(m21,1);
      digitalWrite(m22,0); 
      break;
    case 2:
      analogWrite(en2,pwm);
      digitalWrite(m21,0);
      digitalWrite(m22,1); 
      break;
    }

    break;
  }
}








