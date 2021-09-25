#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <EEPROM.h>
#include "U8glib.h"

#define COUNT 10 // number for statistics
#define Wait_Time_Response 50 //number wait a response
#define Wait_Time_Signal 500 //number wait between tries (when lost)

#define DATARATE RF24_2MBPS
//#define DATARATE RF24_1MBPS
//#define DATARATE RF24_250KBPS

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);

#define inputX1 A1
#define inputY1 A0
#define inputX2 A3
#define inputY2 A2

//botoes
#define botUp 0
#define botDown 3
#define botEnter 1
#define botReturn 2

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
  float bateryVoltage;
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
  /*1*/{"*", "", "", "", ""},
  /*2*/{"Eixo A-X", "Eixo A-Y", "Eixo B-X", "Eixo B-Y", "*"},
  /*3*/{"Maximo", "Minimo", "*", "", ""},
  /*4*/{"Sinal 1", "Sinal 2", "Sinal 3", "Sinal 4", "*"},
  /*5*/{"Maximo", "Minimo", "*", "", ""},
  /*6*/{"*", "", "", "", ""}
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

unsigned int ajMin[4] = {0, 0, 0, 0};
unsigned int ajMax[4] = {1023, 1023, 1023, 1023};
byte saida[2][4] = {
  {180, 180, 180, 180}, // max
  {0, 0, 0, 0}          // min
};

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
      ratio = (10*_success)/COUNT;
      if(ratio<1)ff1 = 0;
      _success = 0;
      _fail = 0;
    }
}

ISR(TIMER2_OVF_vect) {
  i++;
  if(i>100)
  {
    i = 0;
    if(menuPrincipal) if(lastVoltage != RX_Pack.bateryVoltage || lastRatio != ratio)
    {
      refreshMenuPrincipal();
      //Serial.println("refresh yep");
    }/*
    else
    {
      Serial.println("refresh no");
    }*/
  }
}

void refreshMenuPrincipal()
{
  u8g.firstPage();  
  do {
    
    u8g.drawFrame(2, 0, 3, 3); u8g.drawFrame(0, 2, 7, 12); // batery icon 
    u8g.setPrintPos(11, 13); u8g.print(String(RX_Pack.bateryVoltage) + "v"); //batery voltage
    
    u8g.drawLine(105, 12, 105, 12);
    u8g.drawLine(104, 9, 106, 9); u8g.drawLine(103, 10, 103, 10); u8g.drawLine(107, 10, 107, 10);
    u8g.drawLine(101, 8, 102, 7); u8g.drawLine(109, 8, 108, 7);   u8g.drawLine(103, 6, 107, 6); 
    u8g.drawLine(99, 6, 100, 5); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(101, 4, 102, 4); u8g.drawLine(108, 4, 109, 4);   u8g.drawLine(103, 3, 107, 3); // signal icon //u8g.drawLine(99, 6, 101, 4); u8g.drawLine(109, 4, 111, 6);   u8g.drawLine(102, 3, 108, 3); // signal icon
    u8g.setPrintPos(113, 13); u8g.print(ratio); // signal indicator
    
  } while(u8g.nextPage());
  lastVoltage = RX_Pack.bateryVoltage;
  lastRatio = ratio;
}

#define nFilters 10
byte filters[4][nFilters];
byte filterIndex = 0;

#define tempoMenu3 100

unsigned long lastTempoMenu3 = 0;

void setup(void)
{
  for(byte i = 0; i < nFilters; i++)
  {
    for(byte j = 0; j < 4; j++)
    {
      filters[j][i] = 0;
    }
  }
  
  unsigned int val;
  byte val_1;
  byte val_2;
  for(byte i = 0; i <= 14; i+=4)
  {
    val_1 = EEPROM.read(i);
    val_2 = EEPROM.read(i+1);
    val = (val_1 + (int)(val_2 << 8));
    ajMin[i/4] = val;
    
    val_1 = EEPROM.read(i+2);
    val_2 = EEPROM.read(i+3);
    val = (val_1 + (int)(val_2 << 8));
    ajMax[i/4] = val;
  }
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 2; j++)
    {
      saida[j][i] = EEPROM.read((i*2)+j+16);
    }
  }
  
  Serial.begin(115200);

  printf_begin();
  printf("TX");

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

  radio.printDetails();

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
  
  u8g.setColorIndex(255);
  u8g.setFont(u8g_font_unifont);
  refreshMenuPrincipal();
  
   TCCR2B = 0x07; // Prescaler 1024
   TCNT2 = 0x01; // Enable the timer
   TIMSK2 = 0x01; //Disable Compare Match A interrupt enable
   TIFR2 = 0x01; // Enable Interrupt in TIFR2-Register
} 
unsigned long LM = 0;
void loop()
{
  readAnalogs();

  readButtons();

  if(bitRead(botoes, botEnter)==1) buttonPressed();
  
  /*Serial.print(botoes, BIN);
  Serial.print(" ");
  Serial.print(ratio);
  Serial.print(" ");
  Serial.println(RX_Pack.bateryVoltage);*/
  
  Serial.println(micros() - LM);
  LM = micros();
  sendPack();
}

void readButtons()
{
  botoes = 0;
  pinMode(botaoA, OUTPUT);
  pinMode(botaoB, INPUT);
  digitalWrite(botaoA, LOW);
  if(!digitalRead(botao1)) bitSet(botoes, 7);
  if(!digitalRead(botao2)) bitSet(botoes, 6);
  if(!digitalRead(botao3)) bitSet(botoes, 5);
  if(!digitalRead(botao4)) bitSet(botoes, 4);
  pinMode(botaoA, INPUT);
  pinMode(botaoB, OUTPUT);
  digitalWrite(botaoB, LOW);
  if(!digitalRead(botao1)) bitSet(botoes, 3);
  if(!digitalRead(botao2)) bitSet(botoes, 2);
  if(!digitalRead(botao3)) bitSet(botoes, 1);
  if(!digitalRead(botao4)) bitSet(botoes, 0);
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
          temp3 = TX_Pack.x1;
          temp5 = analogRead(inputX1);
          break;
        case 2:
          temp3 = TX_Pack.y1;
          temp5 = analogRead(inputY1);
          break;
        case 3:
          temp3 = TX_Pack.x2;
          temp5 = analogRead(inputX2);
          break;
        case 4:
          temp3 = TX_Pack.y2;
          temp5 = analogRead(inputY2);
          break;
        default:
          temp3 = 0;
      }
    }
    else if(menu==2)
    {
      eixoSelecionado = item+1;
    }
    
    u8g.firstPage();  
    do
    {
      byte i = 0;
      byte j = jIni;
      while(i<nItens)
      {
        u8g.setDefaultForegroundColor();
        if(j==item)
        {
          u8g.drawBox(0, 13*i, 128, 13);
          u8g.setDefaultBackgroundColor();
        }
        u8g.setPrintPos(0, 13*i+12);
        u8g.print(menus[menu][j]);
        i++;
        j++;
        if(i> 4) break;
      }

      if(menu==3) drawValue1(temp3);
      
      /*u8g.drawLine(105, 12, 105, 12);
      u8g.drawLine(104, 9, 106, 9); u8g.drawLine(103, 10, 103, 10); u8g.drawLine(107, 10, 107, 10);
      u8g.drawLine(101, 8, 102, 7); u8g.drawLine(109, 8, 108, 7);   u8g.drawLine(103, 6, 107, 6); 
      u8g.drawLine(99, 6, 100, 5); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(101, 4, 102, 4); u8g.drawLine(108, 4, 109, 4);   u8g.drawLine(103, 3, 107, 3); // signal icon //u8g.drawLine(99, 6, 101, 4); u8g.drawLine(109, 4, 111, 6);   u8g.drawLine(102, 3, 108, 3); // signal icon
      u8g.setPrintPos(113, 13); u8g.print(ratio); // signal indicator*/
      
    } while(u8g.nextPage());
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
  readAnalogs();
  
  u8g.firstPage();  
  do
  {
    byte temp1 = map(TX_Pack.y1, saida[1][0], saida[0][0], 0, 42);
    u8g.drawFrame(7, 1, 6, 44);
    if(temp1!=0) u8g.drawBox(8, 44-temp1, 4, temp1);

    byte temp2 = map(TX_Pack.x1, saida[1][1], saida[0][1], 0, 42);
    u8g.drawFrame(13, 45, 44, 6);
    u8g.drawBox(14, 46, temp2, 4);

    u8g.drawBox(13+(temp2==42?41:temp2), 42-(temp1==42?41:temp1), 3, 3);
    

    temp1 = map(TX_Pack.x2, saida[1][2], saida[0][2], 0, 42);
    u8g.drawFrame(69, 45, 44, 6);
    u8g.drawBox(70+temp1, 46, 42-temp1, 4);

    temp2 = map(TX_Pack.y2, saida[1][3], saida[0][3], 0, 42);
    u8g.drawFrame(113, 1, 6, 44);
    if(temp2!=0) u8g.drawBox(114, 44-temp2, 4, temp2);
    
    u8g.drawBox(69+(temp1==42?41:temp1), 42-(temp2==42?41:temp2), 3, 3);

    
    u8g.setPrintPos(6, 64);
    u8g.print(TX_Pack.y1);
    u8g.setPrintPos(34, 64);
    u8g.print(TX_Pack.x1);
    u8g.setPrintPos(68, 64);
    u8g.print(TX_Pack.x2);
    u8g.setPrintPos(96, 64);
    u8g.print(TX_Pack.y2);
    
  } while(u8g.nextPage());
}

void readAnalogs()
{
  analogRead(0); // remove mystery first read noise
  byte i;

  unsigned int temp4 = analogRead(inputX1);
  if(temp4 > ajMax[0]) temp4 = ajMax[0];
  if(temp4 < ajMin[0]) temp4 = ajMin[0];
  filters[0][filterIndex] = map(temp4, ajMin[0], ajMax[0], saida[1][0], saida[0][0]);
  temp4 = 0;
  for(i = 0; i < nFilters; i++) temp4 += filters[0][i];
  TX_Pack.x1 = temp4 / nFilters;

  temp4 = analogRead(inputY1);
  if(temp4 > ajMax[1]) temp4 = ajMax[1];
  if(temp4 < ajMin[1]) temp4 = ajMin[1];
  filters[1][filterIndex] = map(temp4, ajMin[1], ajMax[1], saida[1][1], saida[0][1]);
  temp4 = 0;
  for(i = 0; i < nFilters; i++) temp4 += filters[1][i];
  TX_Pack.y1 = temp4 / nFilters;
  
  temp4 = analogRead(inputX2);
  if(temp4 > ajMax[2]) temp4 = ajMax[2];
  if(temp4 < ajMin[2]) temp4 = ajMin[2];
  filters[2][filterIndex] = map(temp4, ajMin[2], ajMax[2], saida[1][2], saida[0][2]);
  temp4 = 0;
  for(i = 0; i < nFilters; i++) temp4 += filters[2][i];
  TX_Pack.x2 = temp4 / nFilters;
  
  temp4 = analogRead(inputY2);
  if(temp4 > ajMax[3]) temp4 = ajMax[3];
  if(temp4 < ajMin[3]) temp4 = ajMin[3];
  filters[3][filterIndex] = map(temp4, ajMin[3], ajMax[3], saida[1][3], saida[0][3]);
  temp4 = 0;
  for(i = 0; i < nFilters; i++) temp4 += filters[3][i];
  TX_Pack.y2 = temp4 / nFilters;

  filterIndex++;
  if(filterIndex >= nFilters) filterIndex = 0;
}

void buttonPressed()
{
    while(bitRead(botoes, botEnter)==1){readButtons();} //espera soltar botao
    menuPrincipal = false;
    byte menu = 0;
    byte item = 0;
    unsigned long lastButtonPressed = millis();
    Serial.println("abrindo menu");
    item = pag(menu, item);
    while(true)
    {
      if(lastMenu!=6)
      {
        if(lastMenu==1||lastMenu==3) if(lastTempoMenu3 + tempoMenu3 < millis())
        {
          lastTempoMenu3 = millis();
          if(lastMenu==1) refreshMenuEntradas();
          else if(lastMenu==3) pag(3, lastItem);
        }
        readAnalogs();
        
        readButtons();
        if(botoes!=0) if(millis()>lastButtonPressed+50)
        {
          byte f1 = 0;
          Serial.println("botao pressionado");
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
                ajMax[eixoSelecionado-1] = temp5;
                EEPROM.write((eixoSelecionado-1)*4+2, (temp5 & 0xFF));
                EEPROM.write((eixoSelecionado-1)*4+3, (int) ((temp5 & 0xFF00)>> 8));
              }
              else if(item == 2)
              {
                ajMin[eixoSelecionado-1] = temp5;
                EEPROM.write((eixoSelecionado-1)*4, (temp5 & 0xFF));
                EEPROM.write((eixoSelecionado-1)*4+1, (int) ((temp5 & 0xFF00)>> 8));
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
            
            if(lastMenu==6) 
            {
              Serial.print("mm: ");
              Serial.print(MaxMin);
              Serial.print("  sinal: ");
              Serial.println(eixoSelecionado);
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
        else Serial.print("@@bouncing@@");
      }
      else
      {
        while(true)
        {
          drawValue2(saida[MaxMin][eixoSelecionado]);
        
          byte valor = 0;
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
                pag(5, 0);
                EEPROM.write((eixoSelecionado*2)+MaxMin+16, saida[MaxMin][eixoSelecionado]);
                break;
              }
              
              if(longMode)
              {
                lastButtonPressed = millis();
                while(botoes!=0)
                {
                  if(f1 == 1)
                  {
                    saida[MaxMin][eixoSelecionado]++;
                    if(saida[MaxMin][eixoSelecionado]>180) saida[MaxMin][eixoSelecionado] = 0;
                    drawValue2(saida[MaxMin][eixoSelecionado]);
                  }
                  else if(f1 == 2)
                  {
                    saida[MaxMin][eixoSelecionado]--;
                    if(saida[MaxMin][eixoSelecionado]>180) saida[MaxMin][eixoSelecionado] = 180;
                    drawValue2(saida[MaxMin][eixoSelecionado]);
                  }
                  readButtons();
                }
              }
              else
              {
                if(f1 == 1)
                {
                  saida[MaxMin][eixoSelecionado]++;
                    if(saida[MaxMin][eixoSelecionado]>180) saida[MaxMin][eixoSelecionado] = 0;
                  drawValue2(saida[MaxMin][eixoSelecionado]);
                }
                else if(f1 == 2)
                {
                  saida[MaxMin][eixoSelecionado]--;
                    if(saida[MaxMin][eixoSelecionado]>180) saida[MaxMin][eixoSelecionado] = 180;
                  drawValue2(saida[MaxMin][eixoSelecionado]);
                }
              }
            }
            lastButtonPressed = millis();
          }
        }
      }
    }
}

void drawValue1(byte valor)
{
  u8g.setDefaultForegroundColor();
  u8g.drawFrame(27, 47, 96, 10);
  u8g.drawBox(28, 48, map(valor, saida[1][eixoSelecionado-1], saida[0][eixoSelecionado-1], 0, 94), 8);
  
  u8g.setPrintPos(1, 56);
  u8g.print(valor);
}

void drawValue2(byte valor)
{
  u8g.firstPage();  
  do
  {
    u8g.setDefaultForegroundColor();
    u8g.drawFrame(27, 25, 96, 10);
    u8g.drawBox(28, 26, map(valor, 0, 180, 0, 94), 8);
    
    u8g.setPrintPos(1, 34);
    u8g.print(valor);
  } while(u8g.nextPage());
}
