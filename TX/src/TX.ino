#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <EEPROM.h>
/*@u8g@*/ #include "U8glib.h"

#define COUNT 10 // number for statistics
#define nFilterSignal 10
byte iSignal = 0;
unsigned int signalSum = 0;

#define Wait_Time_Response 50 //number wait a response
#define Wait_Time_Signal 500 //number wait between tries (when lost)

#define DATARATE RF24_2MBPS
//#define DATARATE RF24_1MBPS
//#define DATARATE RF24_250KBPS

/*@u8g@*/ U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);

#define inputX1 A1
#define inputY1 A0
#define inputX2 A2
#define inputY2 A3

//botoes
/*#define botUp 0
#define botDown 3
#define botEnter 1
#define botReturn 2*/
#define botUp 3
#define botDown 2
#define botEnter 1
#define botReturn 0

// voltagem bateria
#define VoltagePin A7
#define nf 64
#define voltageInterval 2000
int voltageIntervalStatus = 0;
int Volts[nf];
int iFilter = 0;
unsigned long lastVoltageInterval = 0;
double internalBattery = 0.00;
double lastInternalBattery = 0.00;

const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};
RF24 radio(9, 10);

int _success = 0;
int _fail = 0;
unsigned long _startTime = 0;

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
  unsigned int x1;
  unsigned int y1;
  unsigned int x2;
  unsigned int y2;
}
uint_joysticks;

uint_joysticks AnalogsBruto;
uint_joysticks Analogs;

// byte analogsBrutoNotHolding = B0000;
bool analogsBrutoNotHolding[4] = {false, false, false, false};
#define histSumTime 100
#define maxDeviationToEndHolding 1 // maximum difference between the held position and the actual position to stop holding the position
#define maxVariationtionToStartHolding 6 // 
unsigned int histSumCount = 0;
unsigned int analogsBrutoSum[4] = {0, 0, 0, 0};
unsigned long analogsBrutoLastSum = 0;

typedef struct
{
  float batteryVoltage;
}
displayDef;
displayDef RX_Pack;

bool aum = 1;
int count1 = 0;
byte ratio = 0;
bool ff1 = 1;
bool ff2 = 0;
unsigned long last_write = 0;

bool b1 = 0;
bool b2 = 0;
bool b3 = 0;
bool b4 = 0;
bool b5 = 0;
bool b6 = 0;
bool b7 = 0;
bool b8 = 0;

byte botoes;

bool menuPrincipal = true;

#define botaoA 2
#define botaoB 3
#define botao1 4
#define botao2 5
#define botao3 6
#define botao4 7
int lastRatio = 0;
double lastVoltage = 0;

byte eixoSelecionado = 0;
byte MaxMin = 0;

bool refreshPoint = true;

unsigned int i = 0; // variavel contadora para timer2

int temp5 = 0; // leitura temporario para ajuste dos potenciometros

String menus[7][5] = 
{
  /*0*/{"Entradas", "Saidas", "Radio", "*", ""},
  /*1*/{"ACB", "*", "", "", ""},
  /*2*/{"Eixo A-X", "Eixo A-Y", "Eixo B-X", "Eixo B-Y", "*"},
  /*3*/{"Maximo", "Minimo", "*", "", ""},
  /*4*/{"A-X  ", "A-Y  ", "B-X  ", "B-Y  ", "*"},
  /*5*/{"Maximo ", "Minimo ", "*", "", ""}, 
  /*6*/{"CAB", "*", "", "", ""}
};

byte itensToMenu[6][4] =
{
  /*0*/{1, 4, 255, 255},
  /*1*/{2, 255, 255, 255},
  /*2*/{3, 3, 3, 3},
  /*3*/{3, 3, 255, 255},
  /*4*/{5, 5, 5, 5},
  /*5*/{6, 6, 255, 255}
};

byte returnMenu[7] =
{
  /*0*/255,
  /*1*/0,
  /*2*/1,
  /*3*/2,
  /*4*/0,
  /*5*/4,
  /*6*/5
};

byte lastMenu = 0;
byte lastItem = 0;
byte jIni = 0;
byte lastJIni = 0;

typedef struct
{
  unsigned int ajMin[4] = {0, 0, 0, 0};
  unsigned int ajMax[4] = {1023, 1023, 1023, 1023};
  unsigned int mid[4][2] = {
    {450, 574},
    {450, 574},
    {450, 574},
    {450, 574}
  };
  byte midEnabled = B1101;
  byte saida[2][4] = {
    {255, 255, 255, 255}, // max //@
    {0, 0, 0, 0}          // min
  };
}
eepromStuff;
eepromStuff e;

void saveToEEPROM()
{
  EEPROM.put(0, e);
}

void loadFromEEPROM()
{
  EEPROM.get(0, e);
}

// unsigned int ajMin[4] = {0, 0, 0, 0};
// unsigned int ajMax[4] = {1023, 1023, 1023, 1023};
// unsigned int ajMid[4] = {512, 512, 512, 512};
// byte ajMidEnabled = B0000;

// byte saida[2][4] = {
//   {255, 255, 255, 255}, // max //@
//   {0, 0, 0, 0}          // min
// };

void readBattery()
{
  if(voltageIntervalStatus < nf)
  {
    lastVoltageInterval = millis();
    voltageIntervalStatus++;

    iFilter++;
    if(iFilter>=nf) iFilter = 0;
    Volts[iFilter] = analogRead(VoltagePin);
    
    unsigned int somaVolts = 0;
    for (int j = 0; j < nf; j++)
    {
      somaVolts += Volts[j];
    }
    somaVolts /= nf;
    
    if(voltageIntervalStatus == nf)
    {
      internalBattery = map(somaVolts, 0, 1023, 0, 443)/100.00;
    }
  }
  else
  {
    if(millis() - lastVoltageInterval >  voltageInterval)
    {
      voltageIntervalStatus = 0;
    }
  }

  
}

void sendPack() {
    unsigned long loop_start = millis();
    if(ff1){
      radio.stopListening();
      if (!radio.write( &TX_Pack, sizeof(TX_Pack) )) last_write = millis();
    } else if(millis() - last_write >= Wait_Time_Signal)ff2 = 1;
    radio.startListening();

    while (!radio.available() && (millis() - loop_start) < Wait_Time_Response&&ff1) {
    }
    if (ff2){
      ff2 = 0;
      unsigned long loop_start2 = millis();
      radio.stopListening();
      if (!radio.write( &TX_Pack, sizeof(TX_Pack) )){}
      radio.startListening();
      bool ff3 = 1;
      while (ff3 && (millis() - loop_start2) < Wait_Time_Response){
        if(radio.available()){
          ff1 = 1;
          ff3 = 0;
          sendPack();
        } else {
          last_write = millis();
        }
      }
    }

    if (millis() - loop_start >= Wait_Time_Response||!ff1) {
      _fail++;
    } else {
      radio.read( &RX_Pack, sizeof(RX_Pack) );
      _success++;
    }

    if (_fail + _success >= COUNT)
    {
      iSignal++;
      // #define nFilterSignal 100
      // byte iSignal = 0;
      // unsigned int signalSum = 0;

      //@sinal
      byte preRatio = (10*_success)/COUNT;
      signalSum += preRatio;
      if(preRatio<1)ff1 = 0;
      _success = 0;
      _fail = 0;
      
      if(iSignal >= 100)
      {
        ratio = signalSum/nFilterSignal;
        iSignal = 0;
        signalSum = 0;

        //Serial.print(signalSum);
        //Serial.print(" ");
        //Serial.println(ratio);
      }
    }
}

ISR(TIMER2_OVF_vect) {
  i++;
  if(i>100)
  {
    i = 0;
    if(menuPrincipal) if(lastVoltage != RX_Pack.batteryVoltage || lastRatio != ratio || lastInternalBattery != internalBattery)
    {
      refreshMenuPrincipal();
      ////Serial.println("refresh yep");
    }/*
    else
    {
      //Serial.println("refresh no");
    }*/
  }
}

void refreshMenuPrincipal()
{
  /*@u8g@*/ u8g.setDefaultForegroundColor();
  /*@u8g@*/ u8g.firstPage();  
  /*@u8g@*/ do {
  /*@u8g@*/   u8g.drawFrame(2, 0, 3, 3); u8g.drawFrame(0, 2, 7, 12); // battery icon 
  /*@u8g@*/   u8g.setPrintPos(11, 13); u8g.print(String(RX_Pack.batteryVoltage) + "v"); // battery voltage
  /*@u8g@*/   
  /*@u8g@*/   u8g.drawFrame(2, 17, 3, 3); u8g.drawFrame(0, 19, 7, 12); // internal battery icon 
  /*@u8g@*/   u8g.setPrintPos(11, 30); u8g.print(String(internalBattery) + "v"); // internal battery voltage
  /*@u8g@*/   
  /*@u8g@*/   // u8g.drawLine(105, 12, 105, 12);
  /*@u8g@*/   // u8g.drawLine(104, 9, 106, 9); u8g.drawLine(103, 10, 103, 10); u8g.drawLine(107, 10, 107, 10);
  /*@u8g@*/   // u8g.drawLine(101, 8, 102, 7); u8g.drawLine(109, 8, 108, 7);   u8g.drawLine(103, 6, 107, 6); 
  /*@u8g@*/   // u8g.drawLine(99, 6, 100, 5); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(101, 4, 102, 4); u8g.drawLine(108, 4, 109, 4);   u8g.drawLine(103, 3, 107, 3); // signal icon //u8g.drawLine(99, 6, 101, 4); u8g.drawLine(109, 4, 111, 6);   u8g.drawLine(102, 3, 108, 3); // signal icon
  /*@u8g@*/   // u8g.setPrintPos(113, 13); u8g.print(ratio); // signal indicator
  /*@u8g@*/   u8g.drawLine(97, 12, 97, 12);
  /*@u8g@*/   u8g.drawLine(96, 9, 98, 9); u8g.drawLine(95, 10, 95, 10); u8g.drawLine(99, 10, 99, 10);
  /*@u8g@*/   u8g.drawLine(93, 8, 94, 7); u8g.drawLine(101, 8, 100, 7);   u8g.drawLine(95, 6, 99, 6); 
  /*@u8g@*/   u8g.drawLine(91, 6, 92, 5); u8g.drawLine(102, 5, 103, 6); u8g.drawLine(102, 5, 103, 6); u8g.drawLine(93, 4, 94, 4); u8g.drawLine(100, 4, 101, 4);   u8g.drawLine(95, 3, 99, 3); // signal icon //u8g.drawLine(99, 6, 101, 4); u8g.drawLine(109, 4, 111, 6);   u8g.drawLine(102, 3, 108, 3); // signal icon
  /*@u8g@*/   u8g.setPrintPos(105, 13); u8g.print(ratio); // signal indicator
  /*@u8g@*/   
  /*@u8g@*/   
  /*@u8g@*/ } while(u8g.nextPage());
  
  lastInternalBattery = internalBattery;
  lastVoltage = RX_Pack.batteryVoltage;
  lastRatio = ratio;
}

#define nFilters 10
unsigned int filters[4][nFilters];
byte filterIndex = 0;

#define tempoMenu3 100

unsigned long lastTempoMenu3 = 0;

void setup()
{
  /*@u8g@*/ u8g.setColorIndex(255);
  /*@u8g@*/ u8g.setFont(u8g_font_unifont);


  /*@u8g@*/ u8g.firstPage();  
  /*@u8g@*/ do {
  /*@u8g@*/   u8g.setPrintPos(25, 32); u8g.print("ligando...");
  /*@u8g@*/ } while(u8g.nextPage());
  
  
  Serial.begin(115200);
  analogReference(EXTERNAL);
  
  for(byte i = 0; i < nFilters; i++)
  {
    for(byte j = 0; j < 4; j++)
    {
      filters[j][i] = 0;
    }
  }

  loadFromEEPROM();

  // printf_begin();
  //printf("TX");

  radio.begin();
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);

  radio.setDataRate( DATARATE ) ;
  radio.setPALevel( RF24_PA_MAX ) ;
  radio.setChannel(0x10);
  radio.enableDynamicPayloads() ;
  radio.enableAckPayload();               // not used here
  radio.setRetries(0, 15);                // Smallest time between retries, max no. of retries
  radio.setAutoAck(true) ;

  // radio.printDetails();

  radio.powerUp();
  radio.startListening();

  pinMode(inputX1, INPUT);
  pinMode(inputY1, INPUT);
  pinMode(inputX2, INPUT);
  pinMode(inputY2, INPUT);
  
  pinMode(botaoA, OUTPUT);
  pinMode(botaoB, OUTPUT);
  pinMode(botao1, INPUT_PULLUP);
  pinMode(botao2, INPUT_PULLUP);
  pinMode(botao3, INPUT_PULLUP);
  pinMode(botao4, INPUT_PULLUP);

  digitalWrite(botaoA, HIGH);
  digitalWrite(botaoB, HIGH);
  
  
  for(int j = 0; j < nf; j++) readBattery();
  
  TCCR2B = 0x07; // Prescaler 1024
  TCNT2 = 0x01; // Enable the timer
  TIMSK2 = 0x01; //Disable Compare Match A interrupt enable
  TIFR2 = 0x01; // Enable Interrupt in TIFR2-Register

  //envia pacotes e atualiza tela no inicio
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  sendPack();
  refreshMenuPrincipal();
}
unsigned long LM = 0;
void loop()
{
  readAnalogs();
  readButtons();
  readBattery();

  if(bitRead(botoes, botEnter)==1) buttonPressed();
  
  ///*Serial.print(botoes, BIN);
  //Serial.print(" ");
  //Serial.print(ratio);
  //Serial.print(" ");
  //Serial.println(RX_Pack.batteryVoltage);*/

  // // prints analog bruto values
  // Serial.print(AnalogsBruto.x1);
  // Serial.print(" ");
  // Serial.print(AnalogsBruto.y1);
  // Serial.print(" ");
  // Serial.print(AnalogsBruto.x2);
  // Serial.print(" ");
  // Serial.print(AnalogsBruto.y2);
  // Serial.println(" 1023 0");

  // // prints analog values
  // Serial.print(Analogs.x1);
  // Serial.print(" ");
  // Serial.print(Analogs.y1);
  // Serial.print(" ");
  // Serial.print(Analogs.x2);
  // Serial.print(" ");
  // Serial.print(Analogs.y2);
  // Serial.println(" 65535 0");
  
  // // prints tx_pack values
  // Serial.print(TX_Pack.x1);
  // Serial.print(" ");
  // Serial.print(TX_Pack.y1);
  // Serial.print(" ");
  // Serial.print(TX_Pack.x2);
  // Serial.print(" ");
  // Serial.print(TX_Pack.y2);
  // Serial.println(" 255 0");


  

  
  LM = micros();
  sendPack();
}

void readButtons()
{
  botoes = 0;
  pinMode(botaoA, OUTPUT);
  pinMode(botaoB, INPUT);
  digitalWrite(botaoA, LOW);
  if(!digitalRead(botao1)) bitSet(botoes, 0);
  if(!digitalRead(botao2)) bitSet(botoes, 1);
  //if(!digitalRead(botao3)) bitSet(botoes, 5);
  //if(!digitalRead(botao4)) bitSet(botoes, 4);
  pinMode(botaoA, INPUT);
  pinMode(botaoB, OUTPUT);
  digitalWrite(botaoB, LOW);
  if(!digitalRead(botao1)) bitSet(botoes, 2);
  if(!digitalRead(botao2)) bitSet(botoes, 3);
  //if(!digitalRead(botao3)) bitSet(botoes, 1);
  //if(!digitalRead(botao4)) bitSet(botoes, 0);
}

byte pag(byte menu, byte item)
{
  if(menu != 1)
  {
    byte nItens = 0;
    while(menus[menu][nItens]!="*")
    {
      nItens++;
    }
    
    if(item==255)item = nItens - 1;
    else if(item>=nItens) item = 0;
  
    if(lastMenu==menu)
    {
      if(item>lastItem)
      {
        if(item>lastJIni+3)
        {
          jIni = item-4;
        }
      }
      else if(item<lastItem)
      {
        if(item<lastJIni)
        {
          jIni = item;
        }
      }
    }
    else
    {
      jIni = 0;
    }

    byte temp3 = 0;
    if(menu==3)
    {
      readAnalogs();
      switch(eixoSelecionado)
      {
        case 1:
          temp3 = Analogs.x1;
          temp5 = AnalogsBruto.x1;
          break;
        case 2:
          temp3 = Analogs.y1;
          temp5 = AnalogsBruto.y1;
          break;
        case 3:
          temp3 = Analogs.x2;
          temp5 = AnalogsBruto.x2;
          break;
        case 4:
          temp3 = Analogs.y2;
          temp5 = AnalogsBruto.y2;
          break;
        default:
          temp3 = 0;
      }
    }
    else if(menu==2)
    {
      eixoSelecionado = item+1;
    }
    
    /*@u8g@*/ u8g.firstPage();  
    /*@u8g@*/ do
    /*@u8g@*/ {
    /*@u8g@*/   //colocar personalizacoes de menu
    /*@u8g@*/   byte i = 0;
    /*@u8g@*/   byte j = jIni;
    /*@u8g@*/   while(i<nItens)
    /*@u8g@*/   {
    /*@u8g@*/     u8g.setDefaultForegroundColor();
    /*@u8g@*/     if(j==item)
    /*@u8g@*/     {
    /*@u8g@*/       u8g.drawBox(0, 13*i, 128, 13);
    /*@u8g@*/       u8g.setDefaultBackgroundColor();
    /*@u8g@*/     }
    /*@u8g@*/     u8g.setPrintPos(0, 13*i+12);
    /*@u8g@*/     u8g.print(menus[menu][j]);
    /*@u8g@*/     if(menu==4)
    /*@u8g@*/     {
    /*@u8g@*/       u8g.print("|");
    /*@u8g@*/       if(e.saida[1][i]<10) u8g.print("  ");
    /*@u8g@*/       else if(e.saida[1][i]<100) u8g.print(" ");
    /*@u8g@*/       u8g.print((String)e.saida[1][i]+" "+(String)e.saida[0][i]);
    /*@u8g@*/       if(e.saida[0][i]<10) u8g.print("  ");
    /*@u8g@*/       else if(e.saida[0][i]<100) u8g.print(" ");
    /*@u8g@*/       u8g.print("|");
    /*@u8g@*/     }
    /*@u8g@*/     else if(menu==5)u8g.print((String)e.saida[i][eixoSelecionado]);
    /*@u8g@*/     i++;
    /*@u8g@*/     j++;
    /*@u8g@*/     if(i> 4) break;
    /*@u8g@*/   }

    /*@u8g@*/   if(menu==3) drawValue1(temp3);
      
    /*@u8g@*/   /*u8g.drawLine(105, 12, 105, 12);
    /*@u8g@*/   u8g.drawLine(104, 9, 106, 9); u8g.drawLine(103, 10, 103, 10); u8g.drawLine(107, 10, 107, 10);
    /*@u8g@*/   u8g.drawLine(101, 8, 102, 7); u8g.drawLine(109, 8, 108, 7);   u8g.drawLine(103, 6, 107, 6); 
    /*@u8g@*/   u8g.drawLine(99, 6, 100, 5); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(101, 4, 102, 4); u8g.drawLine(108, 4, 109, 4);   u8g.drawLine(103, 3, 107, 3); // signal icon //u8g.drawLine(99, 6, 101, 4); u8g.drawLine(109, 4, 111, 6);   u8g.drawLine(102, 3, 108, 3); // signal icon
    /*@u8g@*/   u8g.setPrintPos(113, 13); u8g.print(ratio); // signal indicator*/
      
    /*@u8g@*/ ////Serial.print("A2");
    /*@u8g@*/ } while(u8g.nextPage());

    //if(menu==)
    //Serial.println(menu);
  }
  else if(menu == 1)
  {
    refreshMenuEntradas();
    item = 0;
  }
    
    lastItem = item;
    lastMenu = menu;
    lastJIni = jIni;
    return item;
}

void refreshMenuEntradas()
{
  
  /*@u8g@*/ u8g.firstPage();  
  /*@u8g@*/ do
  /*@u8g@*/ {
  /*@u8g@*/   byte temp1 = map(Analogs.y1, 0, 255, 0, 42);
  /*@u8g@*/   u8g.drawFrame(7, 1, 6, 44);
  /*@u8g@*/   if(temp1!=0) u8g.drawBox(8, 44-temp1, 4, temp1);
  /*@u8g@*/ 
  /*@u8g@*/   byte temp2 = map(Analogs.x1, 0, 255, 0, 42);
  /*@u8g@*/   u8g.drawFrame(13, 45, 44, 6);
  /*@u8g@*/   u8g.drawBox(14, 46, temp2, 4);
  /*@u8g@*/ 
  /*@u8g@*/   u8g.drawBox(13+(temp2==42?41:temp2), 42-(temp1==42?41:temp1), 3, 3);
  /*@u8g@*/   
  /*@u8g@*/ 
  /*@u8g@*/   temp1 = map(Analogs.x2, 0, 255, 0, 42);
  /*@u8g@*/   u8g.drawFrame(69, 45, 44, 6);
  /*@u8g@*/   u8g.drawBox(70+temp1, 46, 42-temp1, 4);
  /*@u8g@*/ 
  /*@u8g@*/   temp2 = map(Analogs.y2, 0, 255, 0, 42);
  /*@u8g@*/   u8g.drawFrame(113, 1, 6, 44);
  /*@u8g@*/   if(temp2!=0) u8g.drawBox(114, 44-temp2, 4, temp2);
  /*@u8g@*/   
  /*@u8g@*/   u8g.drawBox(69+(temp1==42?41:temp1), 42-(temp2==42?41:temp2), 3, 3);
  /*@u8g@*/ 
  /*@u8g@*/   
  /*@u8g@*/   u8g.setPrintPos(6, 64);
  /*@u8g@*/   u8g.print(Analogs.y1);
  /*@u8g@*/   u8g.setPrintPos(34, 64);
  /*@u8g@*/   u8g.print(Analogs.x1);
  /*@u8g@*/   u8g.setPrintPos(68, 64);
  /*@u8g@*/   u8g.print(Analogs.x2);
  /*@u8g@*/   u8g.setPrintPos(96, 64);
  /*@u8g@*/   u8g.print(Analogs.y2);
  /*@u8g@*/   
  /*@u8g@*/ } while(u8g.nextPage());
}

int normalize(int value, int maxV, int minV)
{
  if(maxV < minV)
  {
    int V = maxV;
    maxV = minV;
    minV = V;
  }
  
  if(value > maxV) return maxV;
  else if(value < minV) return minV;
  else return value;
}

// funcao sem compensador de linearidade:
void readAnalogs()
{
  analogRead(0); // it seems that the first analogRead comes with some noise, so, read anything before the actual readings to avoid this noise

  unsigned int now = millis();

  histSumCount++;
  if(now - analogsBrutoLastSum > histSumTime)
  {
    analogsBrutoLastSum = now;

    histSumCount--; // reduces 1 because the last sum was not executed

    for(i = 0; i < 4; i++)
    {
      if(analogsBrutoSum[i]*100/histSumCount <= maxVariationtionToStartHolding)
      {
        analogsBrutoNotHolding[i] = false;
      }

      analogsBrutoSum[i] = 0;
    }

    histSumCount = 0;
  }

  computeFilter(&AnalogsBruto.x1, &Analogs.x1, &TX_Pack.x1, inputX1, 0);
  computeFilter(&AnalogsBruto.y1, &Analogs.y1, &TX_Pack.y1, inputY1, 1);
  computeFilter(&AnalogsBruto.x2, &Analogs.x2, &TX_Pack.x2, inputX2, 2);
  computeFilter(&AnalogsBruto.y2, &Analogs.y2, &TX_Pack.y2, inputY2, 3);

  Serial.println(" |");

//  Serial.print(AnalogsBruto.x1);
//  Serial.print(" ");
//  Serial.print(AnalogsBruto.y1);
//  Serial.print(" ");
//  Serial.print(AnalogsBruto.x2);
//  Serial.print(" ");
//  Serial.print(AnalogsBruto.y2);
//  Serial.println(" 1023 0");

  filterIndex++;
  if(filterIndex >= nFilters) filterIndex = 0;
}

void computeFilter(unsigned int *brutoPointer, unsigned int *analogPointer, byte *txpackPointer, byte readingPin, byte axisNum)
{
  
  filters[axisNum][filterIndex] = analogRead(readingPin);
  unsigned int sumValues = 0;
  for(i = 0; i < nFilters; i++) sumValues += filters[axisNum][i];
  sumValues /= nFilters;

  int deviation = sumValues - *brutoPointer;
  if(deviation < 0) deviation = -deviation;
  analogsBrutoSum[axisNum] += deviation;
  
  if(deviation > maxDeviationToEndHolding || analogsBrutoNotHolding[axisNum] || true)
  {
    analogsBrutoNotHolding[axisNum] = true;
    *brutoPointer = sumValues;    
    *analogPointer = calcAnalogValue(brutoPointer, &axisNum);
    *txpackPointer = map(*analogPointer, 0, 65535, e.saida[1][axisNum], e.saida[0][axisNum]);
  }
  
  // if(axisNum == 1)
  // {
  //   Serial.print(sumValues);
  //   Serial.print(" ");
  //   Serial.println(*brutoPointer);
  // }
}

unsigned int calcAnalogValue(unsigned int *brutoPointer, byte *axisNumPoiner)
{
  unsigned int normalized = normalize(*brutoPointer, e.ajMin[*axisNumPoiner], e.ajMax[*axisNumPoiner]);
  unsigned int final;

  if(*axisNumPoiner <= 1)
  {
    Serial.print(" | ");

    Serial.print(*axisNumPoiner);
    Serial.print(") ");
    Serial.print(bitRead(e.midEnabled, *axisNumPoiner)?"E":"D");
    Serial.print(" ");
    Serial.print(e.mid[*axisNumPoiner][0]);
    Serial.print("~");
    Serial.print(e.mid[*axisNumPoiner][1]);

    Serial.print(" ");
    Serial.print(normalized);
    Serial.print("→");
    Serial.print(final);
  }

  return map(normalized, e.ajMin[*axisNumPoiner], e.ajMax[*axisNumPoiner], 0, 65535);

}

void p(String s, byte c)
{
  unsigned int rep = c - s.length();
  if(rep < 0)
  {
    for(int i=0; i <= c ; i++)
    {
      Serial.print("X");
    }
  }
  else if(rep > 0)
  {
    rep -= 1;
    for(int i=0; i <= rep ; i++)
    {
      Serial.print(" ");
    }
  }
  
  Serial.print(s);
}

void p(int n, byte c) {p((String)n, c);}
void p(unsigned int n, byte c) {p((String)n, c);}
void p(byte n, byte c) {p((String)n, c);}

void buttonPressed()
{
    while(bitRead(botoes, botEnter)==1){readButtons();} //espera soltar botao
    menuPrincipal = false;
    byte menu = 0;
    byte item = 0;
    unsigned long lastButtonPressed = millis();
    //Serial.println("abrindo menu");
    item = pag(menu, item);
    while(true)
    {
      // loop de espera - pressionamento botao
      
      if(lastMenu==6) //se menu for 6
      {
        //Serial.println("Salvando");
        byte valor1 = 0;
        byte valor2 = e.saida[MaxMin][eixoSelecionado];
        if(eixoSelecionado==0) valor1 = TX_Pack.x1;
        else if(eixoSelecionado==1) valor1 = TX_Pack.y1;
        else if(eixoSelecionado==2) valor1 = TX_Pack.x2;
        else if(eixoSelecionado==3) valor1 = TX_Pack.y2;

        //Serial.println("TX pack:");
        //Serial.println("x1 " + (String)TX_Pack.x1);
        //Serial.println("y1 " + (String)TX_Pack.y1);
        //Serial.println("x2 " + (String)TX_Pack.x2);
        //Serial.println("y2 " + (String)TX_Pack.y2);
        
        while(true)
        {
          // loop menu 6 (ajuste de uma e.)
          
          drawValue2(e.saida[MaxMin][eixoSelecionado]);
          
          readButtons();
          if(botoes!=0) if(millis()>lastButtonPressed+50)
          {
            byte f1 = 0;
            if(bitRead(botoes, botUp)==1) f1 = 1;
            else if(bitRead(botoes, botDown)==1) f1 = 2;
            else if(bitRead(botoes, botEnter)==1) f1 = 3;
            else if(bitRead(botoes, botReturn)==1) f1 = 4;
            
            lastButtonPressed = millis();
            bool longMode = true;
            while(millis()<lastButtonPressed+1000)
            {
              readButtons();
              if(botoes==0)
              {
                longMode = false;
                break;
              }
            }

            if(f1 != 0)
            {
              if(f1 == 3 || f1 == 4)
              {
                lastMenu = 5;
                menu = 5;
                item = 0;
                if(f1 == 3) {
                  saveToEEPROM(); // EEPROM.write((eixoSelecionado*2)+MaxMin+16, e.saida[MaxMin][eixoSelecionado]);
                }
                else e.saida[MaxMin][eixoSelecionado] = valor2;
                pag(5, 0);                
                if(eixoSelecionado==0) TX_Pack.x1 = valor1;
                else if(eixoSelecionado==1) TX_Pack.y1 = valor1;
                else if(eixoSelecionado==2) TX_Pack.x2 = valor1;
                else if(eixoSelecionado==3) TX_Pack.y2 = valor1;
                sendPack();
                break;
              }
              
              if(longMode)
              {
                lastButtonPressed = millis();
                while(botoes!=0)
                {
                  if(f1 == 1 || f1 == 2)
                  {
                    if(f1 == 1)
                    {
                      e.saida[MaxMin][eixoSelecionado]+=3;
                      if(e.saida[MaxMin][eixoSelecionado]>255) e.saida[MaxMin][eixoSelecionado] = 0; //@
                      drawValue2(e.saida[MaxMin][eixoSelecionado]);
                    }
                    else
                    {
                      e.saida[MaxMin][eixoSelecionado]-=3;
                      if(e.saida[MaxMin][eixoSelecionado]>255) e.saida[MaxMin][eixoSelecionado] = 255; //@//@
                      drawValue2(e.saida[MaxMin][eixoSelecionado]);
                    }
                    if(eixoSelecionado==0) TX_Pack.x1 = e.saida[MaxMin][eixoSelecionado];
                    else if(eixoSelecionado==1) TX_Pack.y1 = e.saida[MaxMin][eixoSelecionado];
                    else if(eixoSelecionado==2) TX_Pack.x2 = e.saida[MaxMin][eixoSelecionado];
                    else if(eixoSelecionado==3) TX_Pack.y2 = e.saida[MaxMin][eixoSelecionado];
                    sendPack();
                  }
                  readButtons();
                }
              }
              else
              {
                if(f1 == 1)
                {
                  e.saida[MaxMin][eixoSelecionado]++;
                  drawValue2(e.saida[MaxMin][eixoSelecionado]);
                }
                else if(f1 == 2)
                {
                  e.saida[MaxMin][eixoSelecionado]--;
                    if(e.saida[MaxMin][eixoSelecionado]>255) e.saida[MaxMin][eixoSelecionado] = 255; //@//@
                  drawValue2(e.saida[MaxMin][eixoSelecionado]);
                }
              }
            }
            lastButtonPressed = millis();
          }
          
          if(eixoSelecionado==0) TX_Pack.x1 = e.saida[MaxMin][eixoSelecionado];
          else if(eixoSelecionado==1) TX_Pack.y1 = e.saida[MaxMin][eixoSelecionado];
          else if(eixoSelecionado==2) TX_Pack.x2 = e.saida[MaxMin][eixoSelecionado];
          else if(eixoSelecionado==3) TX_Pack.y2 = e.saida[MaxMin][eixoSelecionado];
          sendPack();
        }
      }
      else //se menu for diferente de 6
      {
        
        // se menu for 1 ou 3
        if(lastMenu==1||lastMenu==3) if(lastTempoMenu3 + tempoMenu3 < millis())
        {
          lastTempoMenu3 = millis();
          if(lastMenu==1) refreshMenuEntradas();
          else if(lastMenu==3) pag(3, lastItem);
        }
        readAnalogs();
        readButtons();
        
        if(lastMenu==4||lastMenu==5) //se menu for 4 ou 5
        {
          //Serial.println("m: "+(String)menu+" lm: "+(String)lastMenu);
          sendPack();
        }
        
        if(botoes!=0) if(millis()>lastButtonPressed+50)
        {
          byte f1 = 0;
          //Serial.println("botao pressionado");
          if(bitRead(botoes, botUp)==1)
          {
            item--;
            f1 = 1;
          }
          else if(bitRead(botoes, botDown)==1)
          {
            item++;
            f1 = 1;
          }
          else if(bitRead(botoes, botEnter)==1) f1 = 2;
          else if(bitRead(botoes, botReturn)==1) f1 = 3;
                  
          while(botoes!=0){readButtons();} //espera soltar botao
        
          if(f1==1)
          {
            item = pag(menu, item); //selecionar item
          }
          else if(f1==2) //abrir menu
          {
            if(lastMenu==3)
            {
              if(item == 0)
              {
                e.ajMax[eixoSelecionado-1] = temp5;
                saveToEEPROM(); // EEPROM.write((eixoSelecionado-1)*4+2, (temp5 & 0xFF));
                saveToEEPROM(); // EEPROM.write((eixoSelecionado-1)*4+3, (int) ((temp5 & 0xFF00)>> 8));
              }
              else if(item == 1)
              {
                e.ajMin[eixoSelecionado-1] = temp5;
                saveToEEPROM(); // EEPROM.write((eixoSelecionado-1)*4, (temp5 & 0xFF));
                saveToEEPROM(); // EEPROM.write((eixoSelecionado-1)*4+1, (int) ((temp5 & 0xFF00)>> 8));
              }
            }
            else if(lastMenu==4)
            {
              eixoSelecionado = lastItem;
            }
            else if(lastMenu==5)
            {
              MaxMin = lastItem;
            }

            if(itensToMenu[menu][item]!=255)
            {
              menu = itensToMenu[menu][item];
              item = pag(menu, 0);
            }
          }
          else if(f1==3)
          {
            if(menu == 0)
            {
              refreshMenuPrincipal();
              menuPrincipal = true;
              break;
            }
            else
            {
              menu = returnMenu[menu];
              item = pag(menu, 0);
            }
          }
          lastButtonPressed = millis();
        }
        //else Serial.print("@");
      }
    }
}

void drawValue1(byte valor)
{
  /*@u8g@*/ u8g.setDefaultForegroundColor();
  /*@u8g@*/ u8g.drawFrame(27, 47, 96, 10);
  /*@u8g@*/ u8g.drawBox(28, 48, map(valor, 0, 255, 0, 94), 8);
  
  /*@u8g@*/ u8g.setPrintPos(1, 56);
  /*@u8g@*/ u8g.print(valor);
}

void drawValue2(byte valor)
{
  /*@u8g@*/ u8g.firstPage();  
  /*@u8g@*/ do
  /*@u8g@*/ {
  /*@u8g@*/   u8g.setDefaultForegroundColor();
  /*@u8g@*/   u8g.drawFrame(27, 25, 96, 10);
  /*@u8g@*/   u8g.drawBox(28, 26, map(valor, 0, 255, 0, 94), 8); //@
  /*@u8g@*/   
  /*@u8g@*/   u8g.setPrintPos(1, 34);
  /*@u8g@*/   u8g.print(valor);
  /*@u8g@*/ } while(u8g.nextPage());
}

#define numbOfMappingPoints 5
#define numbOfMappingInputs 4
unsigned int stepSize = 1024/(numbOfMappingPoints-1);

int inputPoints[numbOfMappingInputs][numbOfMappingPoints] = {
  {1023, 908, 585, 204, 65},
  {0, 198, 475, 678, 959},
  {885, 660, 372, 114, 39},
  {0, 105, 402, 627, 199}
};

// int asd[] = {0,1,2};

// unsigned int compensateNonLiniarity(byte p, unsigned int val)
// {
//   for(byte pointIdx = 0; pointIdx < numbOfMappingPoints; pointIdx++)
//   {    
//     int lowerVal;
//     int higherVal;

//     if(inputPoints[p][pointIdx] < inputPoints[p][pointIdx+1])
//     {
//       lowerVal = inputPoints[p][pointIdx];
//       higherVal = inputPoints[p][pointIdx+1];
//     }
//     else
//     {
//       lowerVal = inputPoints[p][pointIdx+1];
//       higherVal = inputPoints[p][pointIdx];
//     }

//     if(val >= lowerVal && val <= higherVal)
//     {
//       return map(val, inputPoints[p][pointIdx], inputPoints[p][pointIdx+1], stepSize*pointIdx, stepSize*(pointIdx+1));
//     }
//   }
// }
