#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <Servo.h>

#define s1 2
#define s2 3
#define s3 4
#define s4 5

#define servo_x1_reset_on_lost_signal false
#define servo_x1_default_value 128

#define servo_y1_reset_on_lost_signal true
#define servo_y1_default_value 60

#define servo_x2_reset_on_lost_signal false
#define servo_x2_default_value 128

#define servo_y2_reset_on_lost_signal false
#define servo_y2_default_value 128


/*unsigned int servo1 = 1000;
unsigned int servo2 = 1333;
unsigned int servo3 = 1666;
unsigned int servo4 = 2000;*/

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

#define COUNT 10 // nomber for statistics

#define DATARATE RF24_2MBPS
//#define DATARATE RF24_1MBPS
//#define DATARATE RF24_250KBPS

const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};
RF24 radio(9, 10);

int _success = 0;
int _fail = 0;
unsigned long _startTime = 0;

unsigned long lastRecived = 0;

typedef struct
{
  byte x1;
  byte y1;
  byte x2;
  byte y2;
}
controlDef;
controlDef TX_Pack;


typedef struct
{
  float batteryVoltage;
}
displayDef;
displayDef RX_Pack;

#define ReferencePin A0
#define VoltagePin A1
#define nf 64
#define voltageInterval 700
int voltageIntervalStatus = 0;
int Volts[nf];
int Refs[nf];
int iFilter = 0;
unsigned long lastVoltageInterval = 0;

int camera = 0;

void recivePack() {
  radio.read(&TX_Pack, sizeof(TX_Pack));

  radio.stopListening();
  radio.write(&RX_Pack, sizeof(RX_Pack));

  radio.startListening();


  camera += (int)(TX_Pack.x1)-128;
  if(camera < 0) camera = 0;
  else if(camera > 169*20) camera = 169*20;

  Serial.println(camera);

}

void setup(void)
{
  Serial.begin(115200);

  printf_begin();

  radio.begin();
  radio.openWritingPipe(pipes [1]);
  radio.openReadingPipe(1, pipes[0]);

  radio.setDataRate( DATARATE ) ;
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setChannel(0x10);
  radio.enableDynamicPayloads() ;
  radio.enableAckPayload();               // not used here
  radio.setRetries(0, 15);                // Smallest time between retries, max no. of retries
  radio.setAutoAck( true ) ;

  radio.printDetails();

  pinMode(VoltagePin, INPUT);

  radio.powerUp();
  radio.startListening();

  
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);

  TX_Pack.x1 = servo_x1_default_value;
  TX_Pack.y1 = servo_y1_default_value;
  TX_Pack.x2 = servo_x2_default_value;
  TX_Pack.y2 = servo_y2_default_value;


  /*pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(s4, OUTPUT);*/
  /*TCCR2A = 0x00;   //Timer operando em modo normal
  TCCR2B = 0x07;   //Prescaler 1:1024
  TIMSK2 = 0x01;   //Habilita interrupção do Timer2*/
} 
/*
ISR(TIMER2_OVF_vect)
{
    TCNT2=0;          // Reinicializa o registrador do Timer2
    //Serial.println(micros());
    digitalWrite(s1, HIGH); //Inverte o estado da saída
    delayMicroseconds(map(servo1, 0, 180, 1000, 2000));
    digitalWrite(s1, LOW); //Inverte o estado da saída
    digitalWrite(s2, HIGH); //Inverte o estado da saída
    delayMicroseconds(map(servo2, 0, 180, 1000, 2000));
    digitalWrite(s2, LOW); //Inverte o estado da saída
    digitalWrite(s3, HIGH); //Inverte o estado da saída
    delayMicroseconds(map(servo3, 0, 180, 1000, 2000));
    digitalWrite(s3, LOW); //Inverte o estado da saída
    digitalWrite(s4, HIGH); //Inverte o estado da saída
    delayMicroseconds(map(servo4, 0, 180, 1000, 2000));
    digitalWrite(s4, LOW); //Inverte o estado da saída
}*/

unsigned long asd = 0;
unsigned int asd2 = 0;
unsigned int asd3 = 0;

void loop(void)
{
  if(voltageIntervalStatus < nf)
  {
    lastVoltageInterval = millis();
    voltageIntervalStatus++;

    iFilter++;
    if(iFilter>=nf) iFilter = 0;
    Volts[iFilter] = analogRead(VoltagePin);
    Refs[iFilter] = analogRead(ReferencePin);
    
    unsigned int somaVolts = 0;
    unsigned int somaRefs = 0;
    for (int j = 0; j < nf; j++)
    {
      somaVolts += Volts[j];
      somaRefs += Refs[j];
    }
    somaVolts /= nf;
    somaRefs /= nf;
    
    if(voltageIntervalStatus == nf)
    {
      RX_Pack.batteryVoltage = map(somaVolts, 0, somaRefs, 0, 1000)/100.00;
      
      /*
      Serial.print(RX_Pack.batteryVoltage);
      Serial.print(" ");
      Serial.print(somaVolts);
      Serial.print(" ");
      Serial.println(somaRefs);*/
    }
  }
  else
  {
    if(millis() - lastVoltageInterval >  voltageInterval)
    {
      voltageIntervalStatus = 0;
    }
  }
  

  // Serial.print(255);
  // Serial.print(" ");
  // Serial.print(0);
  // Serial.print(" ");
  // Serial.print(TX_Pack.x1);
  // Serial.print(" ");
  // Serial.print(TX_Pack.y1);
  // Serial.print(" ");
  // Serial.print(TX_Pack.x2);
  // Serial.print(" ");
  // Serial.println(TX_Pack.y2);


  asd2++;
  unsigned long asd4 = millis();
  if(asd4 - asd > 500)
  {
    //asd3 = asd2;
    asd3 = asd2/((asd4-asd)*0.001);
    asd2 = 0;
    asd = asd4;
    //Serial.println(asd3);
  }
  
  /*servo1 = TX_Pack.x1;
  servo2 = TX_Pack.y1;
  servo3 = TX_Pack.x2;
  servo4 = TX_Pack.y2;*/
  servo1.write(camera/20);
  servo2.write(TX_Pack.y1);
  servo3.write(TX_Pack.x2);
  servo4.write(TX_Pack.y2);

  delay(2);
  
  if (radio.available()) 
  {
    recivePack();
    lastRecived = millis();
  }
  else if (millis() > lastRecived + 500) 
  { 
    
    //Serial.println("conexao perdida");

    #if servo_x1_reset_on_lost_signal == true
      TX_Pack.x1 = servo_x1_default_value;
    #endif
    
    #if servo_y1_reset_on_lost_signal == true
      TX_Pack.y1 = servo_y1_default_value;
    #endif
    
    #if servo_x2_reset_on_lost_signal == true
      TX_Pack.x2 = servo_x2_default_value;
    #endif
    
    #if servo_y2_reset_on_lost_signal == true
      TX_Pack.y2 = servo_y2_default_value;
    #endif
  }
}
