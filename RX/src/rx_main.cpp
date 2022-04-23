#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <Servo.h>


#define MODE_BARCO 1
#define MODE_DRONE 2

#define MODE MODE_DRONE

#if MODE == MODE_BARCO
  #define CAMERA

  // #define servo_x1_default_value 128
  #define servo_y1_default_value 60
  // #define servo_x2_default_value 128
  // #define servo_y2_default_value 128

#elif MODE == MODE_DRONE
  #define SBUS_OUT

  #define servo_x1_default_value 128
  #define servo_y1_default_value 0
  #define servo_x2_default_value 128
  #define servo_y2_default_value 128
#endif




#define s1 2
#define s2 3
#define s3 4
#define s4 5





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
  byte arm_drone;
} TX_Pack_def;
TX_Pack_def TX_Pack;


typedef struct
{
  float batteryVoltage;
}
displayDef;
displayDef RX_Pack;

#define ReferencePin A0
#define VoltagePin A1
#define nf 256
#define voltageInterval 700
int voltageIntervalStatus = 0;
int Volts[nf];
int Refs[nf];
int iFilter = 0;
unsigned long lastVoltageInterval = 0;

#ifdef CAMERA
int camera = 0;
#endif

void recivePack(){

  radio.read(&TX_Pack, sizeof(TX_Pack));

  radio.stopListening();
  radio.write(&RX_Pack, sizeof(RX_Pack));

  radio.startListening();


  #ifdef CAMERA
  camera += (int)(TX_Pack.x1)-128;
  if(camera < 0) camera = 0;
  else if(camera > 169*20) camera = 169*20;
  // Serial.println(camera);
  #endif


}

/*---------- sbus ----------*/
#ifdef SBUS_OUT
#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms


////////////////////////// functions headers //////////////////////////
void recivePack();
void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe);
void sendPacket();
void setup(void);
void loop(void);
///////////////////////////////////////////////////////////////////////

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){
    static int output[SBUS_CHANNEL_NUMBER] = {0};
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;

void sendPacket()
{
  sbusPreparePacket(sbusPacket, rcChannels, false, false);
  Serial.write(sbusPacket, SBUS_PACKET_LENGTH);
}
#endif
/*--------------------------*/

void setup(void)
 {
  #ifndef SBUS_OUT
  Serial.begin(115200);
  #endif

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

  #ifndef SBUS_OUT
  radio.printDetails();
  #endif

  pinMode(VoltagePin, INPUT);

  radio.powerUp();
  radio.startListening();

  
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  
  #ifdef servo_x1_default_value
  TX_Pack.x1 = servo_x1_default_value;
  #endif
  
  #ifdef servo_y1_default_value
  TX_Pack.y1 = servo_y1_default_value;
  #endif
  
  #ifdef servo_x2_default_value
  TX_Pack.x2 = servo_x2_default_value;
  #endif
  
  #ifdef servo_y2_default_value
  TX_Pack.y2 = servo_y2_default_value;
  #endif



#ifdef SBUS_OUT
  /*---------- sbus ----------*/
  for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
      rcChannels[i] = 1500;
  }
  Serial.begin(100000, SERIAL_8E2);
  /*--------------------------*/
#endif

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

void loop(void){
  if(voltageIntervalStatus < nf)
  {
    lastVoltageInterval = millis();
    voltageIntervalStatus++;

    iFilter++;
    if(iFilter>=nf) iFilter = 0;
    Volts[iFilter] = analogRead(VoltagePin);
    Refs[iFilter] = analogRead(ReferencePin);
    
    unsigned long somaVolts = 0;
    unsigned long somaRefs = 0;
    for (int j = 0; j < nf; j++)
    {
      somaVolts += Volts[j];
      somaRefs += Refs[j];
    }
    somaVolts /= nf;
    somaRefs /= nf;
    
    if(voltageIntervalStatus == nf)
    {
      RX_Pack.batteryVoltage = map(somaVolts, 0, somaRefs, 0, 2373)/100.00;
      
      // Serial.print(RX_Pack.batteryVoltage);
      // Serial.print(" ");
      // Serial.print(somaVolts);
      // Serial.print(" ");
      // Serial.println(somaRefs);
    }
  }
  else
  {
    if(millis() - lastVoltageInterval >  voltageInterval)
    {
      voltageIntervalStatus = 0;
    }
  }
  

  //  Serial.print(TX_Pack.x1);
  //  Serial.print(" ");
  //  Serial.print(TX_Pack.y1);
  //  Serial.print(" ");
  //  Serial.print(TX_Pack.x2);
  //  Serial.print(" ");
  //  Serial.print(TX_Pack.y2);
  //  Serial.print(" ");
  //  Serial.print(255);
  //  Serial.print(" ");
  //  Serial.println(0);


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
  
  // servo1 = TX_Pack.x1;
  // servo2 = TX_Pack.y1;
  // servo3 = TX_Pack.x2;
  // servo4 = TX_Pack.y2;
  #ifdef CAMERA
  servo1.write(camera/20);
  #else
  servo1.write(TX_Pack.x1);
  #endif
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
    TX_Pack.arm_drone = 0;
    //Serial.println("conexao perdida");

    #ifdef servo_x1_default_value
      TX_Pack.x1 = servo_x1_default_value;
    #endif
    
    #ifdef servo_y1_default_value
      TX_Pack.y1 = servo_y1_default_value;
    #endif
    
    #ifdef servo_x2_default_value
      TX_Pack.x2 = servo_x2_default_value;
    #endif
    
    #ifdef servo_y2_default_value
      TX_Pack.y2 = servo_y2_default_value;
    #endif
  }

  #ifdef SBUS_OUT
    uint32_t currentMillis = millis();
  
  if (currentMillis > sbusTime) {
    
      rcChannels[0] = map(TX_Pack.x2, 0, 255, 1000, 2000);
      rcChannels[1] = map(TX_Pack.y2, 0, 255, 1000, 2000);
      rcChannels[3] = map(TX_Pack.x1, 0, 255, 1000, 2000);
      rcChannels[2] = map(TX_Pack.y1, 0, 255, 1000, 2000);
      
      rcChannels[4] = map(TX_Pack.arm_drone, 0, 255, 1000, 2000);
      
      sendPacket();
      sbusTime = currentMillis + SBUS_UPDATE_RATE;
  }
  #endif
}
