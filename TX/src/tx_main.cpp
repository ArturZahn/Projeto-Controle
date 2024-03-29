#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <EEPROM.h>
// /*@u8g@*/ #include "U8glib.h"
// /*@u8g@*/ #include "U8glib.h"
#include "U8glib.h"



#define MODE_BARCO 1
#define MODE_DRONE 2

#define MODE MODE_DRONE
// #define MODE MODE_BARCO

#if MODE == MODE_BARCO
#elif MODE == MODE_DRONE
  #define ARM_DRONE
  #define CAMERA_DRONE
  #define AUX_2_FAIL_SAFE_AND_GPS_RESCUE
#endif

#ifdef CAMERA_DRONE
  #define CAMERA_DRONE_MIN 26
  #define CAMERA_DRONE_MAX 172
  #define CAMERA_DRONE_STEP 5
  #define CAMERA_DRONE_LOOK_DOWN_POSITION 26
  #define CAMERA_DRONE_LOOK_DOWN_DBCLICK_TIME 400 // time in milliseconds used to check a double click
  #define CAMERA_DRONE_SLIDING_SPEED 0.125 // movement speed when pressing the button
#endif


#define COUNT 10 // number for statistics
#define nFilterSignal 100
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

#define minTimeBetweenButtonReads 30
#define longClickTime 200 // time in milliseconds used to check a long click

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

/*---------------------------- Logs and options ----------------------------*/
//tags: logoptions log options

#define logAnalogsRaw false
#define logAnalogs false
#define logAnalogsMappedToByte false
#define logOutput true

#define logLimitAdnMiddleAdjust false
#define limitAdnMiddleAdjustAxis 3

#define logInternalBatteryVoltage false

/*--------------------------------------------------------------*/

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
  #ifdef ARM_DRONE
  byte arm_drone;
  #endif
  #ifdef CAMERA_DRONE
    byte camera_drone_position;
  #endif
  #ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
  byte aux_2_fail_safe_gps_rescue;
  #endif
} TX_Pack_def;
TX_Pack_def TX_Pack;

typedef struct
{
  unsigned int x1;
  unsigned int y1;
  unsigned int x2;
  unsigned int y2;
}
uint_joysticks;

uint_joysticks AnalogsRaw;
uint_joysticks Analogs;
#define AnalogsMaxValue 65535
#define AnalogsMiddleValue 32896

// byte analogsRawNotHolding = B0000;
bool analogsRawNotHolding[4] = {false, false, false, false};
#define histSumTime 100
#define maxDeviationToEndHolding 1 // maximum difference between the held position and the actual position to stop holding the position
#define maxVariationtionToStartHolding 6 // 
unsigned int histSumCount = 0;
unsigned int analogsRawSum[4] = {0, 0, 0, 0};
unsigned long analogsRawLastSum = 0;

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

unsigned int it2 = 0; // variavel contadora para timer2

int temp5 = 0; // leitura temporario para ajuste dos potenciometros

String menus[9][5] = 
{
  /*0*/{"Entradas", "Saidas", "Fail Safe", "GPS Rescue", "*"},
  /*1*/{"ACB", "*", "", "", ""},
  /*2*/{"Eixo A-X", "Eixo A-Y", "Eixo B-X", "Eixo B-Y", "*"},
  /*3*/{"Maximo", "Minimo", "*", "", ""},
  /*4*/{"A-X  ", "A-Y  ", "B-X  ", "B-Y  ", "*"},
  /*5*/{"Maximo ", "Minimo ", "*", "", ""}, 
  /*6*/{"CAB", "*", "", "", ""},
};

byte itensToMenu[6][4] =
{
  /*0*/{1, 4, 7, 8},
  /*1*/{2, 255, 255, 255},
  /*2*/{3, 3, 3, 3},
  /*3*/{3, 3, 255, 255},
  /*4*/{5, 5, 5, 5},
  /*5*/{6, 6, 255, 255}
};

byte returnMenu[9] =
{
  /*0*/255,
  /*1*/0,
  /*2*/1,
  /*3*/2,
  /*4*/0,
  /*5*/4,
  /*6*/5,
  /*7*/0,
  /*8*/0,
};

byte lastMenu = 0;
byte lastItem = 0;
byte jIni = 0;
byte lastJIni = 0;

typedef struct
{
  // unsigned int ajMin[4] = {0, 0, 0, 0};
  // unsigned int ajMax[4] = {1023, 1023, 1023, 1023};
  unsigned int ajMin[4] = {2, 878, 950, 18};
  unsigned int ajMax[4] = {968, 25, 71, 846};
  // unsigned int mid[4][2];
  unsigned int mid[4][2] = {
    // {512, 512},
    // {512, 512},
    // {512, 512},
    // {512, 512}

    {444, 513}, //normal
    {512, 512}, //invertido (nao calibrado)
    {553, 488}, //invertido
    {395, 427}  //normal
  };
  byte midEnabled = B1101;
  byte saida[2][4] = {
    {255, 255, 255, 255}, // max //@
    {0, 0, 0, 0}          // min
  };
}
eepromStuff;
eepromStuff e;


//////////////////////// function headers ////////////////////////
void saveToEEPROM();
void loadFromEEPROM();
bool ensureMiddleAdjustIsRight();
void readBattery();
void sendPack();
void refreshMenuPrincipal();
void refreshMenuPrincipal(bool forceUpdate);
void setup();
void loop();
void readButtons();
byte pag(byte menu, byte item);
void refreshMenuEntradas();
int normalize(int value, int maxV, int minV);
void readAnalogs();
void computeFilter(unsigned int *rawPointer, unsigned int *analogPointer, byte *txpackPointer, byte readingPin, byte axisNum);
unsigned int calcAnalogValue(unsigned int *rawPointer, byte *axisNumPoiner);
void openMainMenu();
void drawValue1(unsigned int valor);
void drawValue2(byte valor);
byte mapAnalogToByte(unsigned int val);
//////////////////////////////////////////////////////////////////


void saveToEEPROM()
{
  // ensureMiddleAdjustIsRight();
  // EEPROM.put(0, e);
}

void loadFromEEPROM()
{
  // EEPROM.get(0, e);
  // if(ensureMiddleAdjustIsRight()) EEPROM.put(0, e);
}

bool ensureMiddleAdjustIsRight()
{
  bool anythingChanged = false;
  for(byte i = 0; i < 4; i++)
  {
    if((e.ajMin[i] > e.ajMax[i]) != (e.mid[i][0] > e.mid[i][1])) // if the ajMin/ajMax is in crescent/decrescent order and the mid adjust is different
    {
      // so change mid adjust order to make it order equals ajMin/ajMax order
      unsigned int tempVal = e.mid[i][0];
      e.mid[i][0] = e.mid[i][1];
      e.mid[i][1] = tempVal;
      anythingChanged = true;
    }
  }

  return anythingChanged;
}

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

      //@sinal
      byte preRatio = (10*_success)/COUNT;
      signalSum += preRatio;
      if(preRatio<1)ff1 = 0;

      _success = 0;
      _fail = 0;
      
      if(iSignal >= nFilterSignal)
      {
        ratio = signalSum*10/nFilterSignal;
        iSignal = 0;
        signalSum = 0;
      }
    }
}

ISR(TIMER2_OVF_vect) {
  it2++;
  if(it2>100)
  {
    it2 = 0;
    if(menuPrincipal)
    {
      refreshMenuPrincipal();
    }
  }
}

void refreshMenuPrincipal()
{
  refreshMenuPrincipal(false);
}
void refreshMenuPrincipal(bool forceUpdate)
{
  // Serial.print("refreshMenuPrincipal ");
  // Serial.println(forceUpdate);
  // if there isn't new values and it dont want to force the update, do not update screen
  if((lastVoltage == RX_Pack.batteryVoltage) && (lastRatio == ratio) && (lastInternalBattery == internalBattery) && !forceUpdate) 
  {
    return;
  }

  /*@u8g@*/ u8g.setDefaultForegroundColor();
  /*@u8g@*/ u8g.firstPage();  
  /*@u8g@*/ do {
  /*@u8g@*/   u8g.drawFrame(2, 0, 3, 3); u8g.drawFrame(0, 2, 7, 12); // battery icon 
  /*@u8g@*/   u8g.setPrintPos(11, 13); u8g.print(String(RX_Pack.batteryVoltage) + "v"); // battery voltage
  /*@u8g@*/   
  /*@u8g@*/   u8g.drawFrame(2, 17, 3, 3); u8g.drawFrame(0, 19, 7, 12); // internal battery icon 
  /*@u8g@*/   u8g.setPrintPos(11, 30); u8g.print(String(internalBattery) + "v"); // internal battery voltage
  /*@u8g@*/   
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


#ifdef CAMERA_DRONE

unsigned long camera_drone_last_click_look_down = 0;
byte camera_drone_initial_sliding_position;
unsigned long camera_drone_last_position;
bool camera_drone_is_looking_down = false;


void togleCameraDroneLookDown()
{
  camera_drone_is_looking_down = !camera_drone_is_looking_down;

  if(camera_drone_is_looking_down)
  {
    // if changing to the look down position
    camera_drone_last_position = TX_Pack.camera_drone_position;
    TX_Pack.camera_drone_position = CAMERA_DRONE_LOOK_DOWN_POSITION;
  }
  else
  {
    // if returning from the look down position
    TX_Pack.camera_drone_position = camera_drone_last_position;
  }
}

#endif

#ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
byte fail_safe_gps_rescue = 0;

void updateFailSafeAndGpsRescue()
{
  TX_Pack.aux_2_fail_safe_gps_rescue = fail_safe_gps_rescue*64 + 32;
}

void enterFailSafeMode()
{
  bitSet(fail_safe_gps_rescue, 0);
  updateFailSafeAndGpsRescue();
}
void leaveFailSafeMode()
{
  bitClear(fail_safe_gps_rescue, 0);
  updateFailSafeAndGpsRescue();
}

void enterGpsRescueMode()
{
  bitSet(fail_safe_gps_rescue, 1);
  updateFailSafeAndGpsRescue();
}
void leaveGpsRescueMode()
{
  bitClear(fail_safe_gps_rescue, 1);
  updateFailSafeAndGpsRescue();
}

#endif

void setup()
{

  Serial.begin(115200);

  /*@u8g@*/ u8g.setColorIndex(255);
  /*@u8g@*/ u8g.setFont(u8g_font_unifont);


  /*@u8g@*/ u8g.firstPage();  
  /*@u8g@*/ do {
  /*@u8g@*/   u8g.setPrintPos(25, 32); u8g.print("ligando...");
  /*@u8g@*/ } while(u8g.nextPage());
  
  
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

  #ifdef ARM_DRONE
  TX_Pack.arm_drone = 0;
  #endif
  
  #ifdef CAMERA_DRONE
    TX_Pack.camera_drone_position = CAMERA_DRONE_MAX;
    camera_drone_last_position = TX_Pack.camera_drone_position;
  #endif

  #ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
  updateFailSafeAndGpsRescue();
  #endif

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
  refreshMenuPrincipal(true);
}
unsigned long LM = 0;
byte lastBotoes = 0;
bool startedHolding = false;
bool lastIsLongClick = false;
unsigned long startedHoldingTime = 0;
bool isHoldingTwo = false;
unsigned long holdTwoStartTime = 0;
void loop()
{
  readAnalogs();
  readButtons();
  readBattery();
  
  int buttunsPressendCount = 0;
  if(bitRead(botoes, botUp)==1) buttunsPressendCount++;
  if(bitRead(botoes, botDown)==1) buttunsPressendCount++;
  if(bitRead(botoes, botEnter)==1) buttunsPressendCount++;
  if(bitRead(botoes, botReturn)==1) buttunsPressendCount++;

  if(startedHolding)
  {
    if(lastBotoes != botoes)
    {
      startedHolding = false;
    }
  }

  
  // Serial.print(botoes, BIN);
  // Serial.print(" - ");

  // Serial.print("\n");

  // dont do anything if more than one button is pressed
  if(buttunsPressendCount == 1)
  {
    ////////////// ↓ click down and long click detection ↓ //////////////

    // those next three variable can be used to control trings
    bool isClickDown = false;           // if true, indicates that the button was just pressed
    bool isLongClick = false;           // if true, indicates that the buttos was pressed for some time
    bool isStartOfLongClick = false;    // if true, indicates that the button is entering the "long click" state
    unsigned long longClickElapsedTime; // if isLongClick is true, indicates the elapsed time since entered in the "long click" state, else indicates the elapsed time since the isClickDown was set to true.

    if(lastBotoes != botoes)
    {
      isClickDown = true;
      startedHolding = true;
      startedHoldingTime = millis();
    }

    if(startedHolding)
    {
      longClickElapsedTime = millis()-startedHoldingTime;
      if(longClickElapsedTime > longClickTime)
      {
        longClickElapsedTime -= longClickTime;
        isLongClick = true;
        if(isLongClick != lastIsLongClick)
        {
          isStartOfLongClick = true;
        }
      }
    }
    // Serial.print(" | ");
    // Serial.print(isClickDown?"DOWN":"    ");
    // Serial.print(" | ");
    // Serial.print(isStartOfLongClick?"START":"     ");
    // Serial.print(" | ");
    // Serial.print(isLongClick?"LONG":"NORM");
    // Serial.print(" | ");
    // Serial.print(longClickElapsedTime);
    // Serial.print(" | ");
    ////////////// ↑ click down and long click detection ↑ //////////////

    #ifdef CAMERA_DRONE
      if(isClickDown) // if the button was just pressed
      {
        if(bitRead(botoes, botUp)==1)
        {
          if(!camera_drone_is_looking_down)
          {
            if(TX_Pack.camera_drone_position+CAMERA_DRONE_STEP > CAMERA_DRONE_MAX) TX_Pack.camera_drone_position = CAMERA_DRONE_MAX;
            else TX_Pack.camera_drone_position += CAMERA_DRONE_STEP;

            // camera_drone_press_down_start = millis();
          }
          else togleCameraDroneLookDown();
        }
        else if(bitRead(botoes, botDown)==1)
        {
          if(millis() - camera_drone_last_click_look_down < CAMERA_DRONE_LOOK_DOWN_DBCLICK_TIME)
          {
            // if its a double click

            // undo the last step
            if(TX_Pack.camera_drone_position+CAMERA_DRONE_STEP <= CAMERA_DRONE_MAX) TX_Pack.camera_drone_position += CAMERA_DRONE_STEP;

            togleCameraDroneLookDown();
          }
          else
          {
            // if its a single click
            if(!camera_drone_is_looking_down)
            {
              if(TX_Pack.camera_drone_position-CAMERA_DRONE_STEP < CAMERA_DRONE_MIN) TX_Pack.camera_drone_position = CAMERA_DRONE_MIN;
              else TX_Pack.camera_drone_position -= CAMERA_DRONE_STEP;
            }
          }

          camera_drone_last_click_look_down = millis();
        }

        #ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
          if(bitRead(botoes, botReturn)==1 || bitRead(botoes, botEnter)==1)
          {
            leaveFailSafeMode();
            leaveGpsRescueMode();
          }
        #endif
        
        #ifdef ARM_DRONE
        if(bitRead(botoes, botEnter)==1)
        {
          TX_Pack.arm_drone = 0;
        }
        else if(bitRead(botoes, botReturn)==1)
        {
          TX_Pack.arm_drone = 192;
        }
        #endif

      }
      if(isLongClick)
      {
        if(!camera_drone_is_looking_down && (bitRead(botoes, botUp)==1 || bitRead(botoes, botDown)==1))
        {
          if(isStartOfLongClick) camera_drone_initial_sliding_position = TX_Pack.camera_drone_position;

          // this variable will be used to calculate the camera position
          // here it stores the deviation from original value
          int position_calculation_var = CAMERA_DRONE_SLIDING_SPEED*longClickElapsedTime;

          // flip the direction if depending on which button is pressed
          if(bitRead(botoes, botDown)==1) position_calculation_var *= -1;
          
          // calculate the position
          position_calculation_var = camera_drone_initial_sliding_position+position_calculation_var;

          // check if the position is on the defined limits
          if(position_calculation_var < CAMERA_DRONE_MIN) position_calculation_var = CAMERA_DRONE_MIN;
          else if(position_calculation_var > CAMERA_DRONE_MAX) position_calculation_var = CAMERA_DRONE_MAX;
          
          // write the final value to the pack
          TX_Pack.camera_drone_position = position_calculation_var;
        }
      }

    #endif

    lastIsLongClick = isLongClick;
  }

  if(buttunsPressendCount == 2)
  {
    if(isHoldingTwo)
    {
      if(millis()-holdTwoStartTime > 1000) // if hold any two buttons for 1s
      {

        if(bitRead(botoes, botUp)==1 && bitRead(botoes, botDown)==1)
        {
          openMainMenu();
        }
      }
    }
    else
    {
      isHoldingTwo = true;
      holdTwoStartTime = millis();
    }
  }
  else
  {
    isHoldingTwo = false;
  }

  lastBotoes = botoes;


  
  ///*Serial.print(botoes, BIN);
  //Serial.print(" ");
  //Serial.print(ratio);
  //Serial.print(" ");
  //Serial.println(RX_Pack.batteryVoltage);*/

  #if logAnalogsRaw
  // prints analog raw values
  Serial.print(AnalogsRaw.x1);
  Serial.print(" ");
  Serial.print(AnalogsRaw.y1);
  Serial.print(" ");
  Serial.print(AnalogsRaw.x2);
  Serial.print(" ");
  Serial.print(AnalogsRaw.y2);
  Serial.println(" 1023 0");
  #endif

  #if logAnalogs
  // prints analog values
  Serial.print(Analogs.x1);
  Serial.print(" ");
  Serial.print(Analogs.y1);
  Serial.print(" ");
  Serial.print(Analogs.x2);
  Serial.print(" ");
  Serial.print(Analogs.y2);
  Serial.println(" "+(String)AnalogsMaxValue+" 0");
  #endif

  #if logAnalogsMappedToByte
  // prints analog values
  Serial.print(mapAnalogToByte(Analogs.x1));
  Serial.print(" ");
  Serial.print(mapAnalogToByte(Analogs.y1));
  Serial.print(" ");
  Serial.print(mapAnalogToByte(Analogs.x2));
  Serial.print(" ");
  Serial.print(mapAnalogToByte(Analogs.y2));
  Serial.println(" 255 0");
  #endif
  
  #if logOutput
  // prints tx_pack values
  Serial.print(TX_Pack.x1);
  Serial.print(" ");
  Serial.print(TX_Pack.y1);
  Serial.print(" ");
  Serial.print(TX_Pack.x2);
  Serial.print(" ");
  Serial.print(TX_Pack.y2);
  #ifdef CAMERA_DRONE
    Serial.print(" ");
    Serial.print(TX_Pack.camera_drone_position);
  #endif
  #ifdef ARM_DRONE
    Serial.print(" ");
    Serial.print(TX_Pack.arm_drone);
  #endif
  #ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
    Serial.print(" ");
    Serial.print(TX_Pack.aux_2_fail_safe_gps_rescue);
  #endif
  Serial.println(" 255 0");
  #endif


  #if logInternalBatteryVoltage
  Serial.println(internalBattery);
  #endif

  

  
  LM = micros();
  sendPack();
}

unsigned long lastButtonReadTime = 0;
void readButtons()
{
  if(millis()-lastButtonReadTime < minTimeBetweenButtonReads) return;

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
  
  lastButtonReadTime = millis();
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

    unsigned int temp3 = 0;
    if(menu==3)
    {
      for(int i = 0; i < nFilters; i++)
      {
        readAnalogs();
      }
      switch(eixoSelecionado)
      {
        case 1:
          temp3 = Analogs.x1;
          temp5 = AnalogsRaw.x1;
          break;
        case 2:
          temp3 = Analogs.y1;
          temp5 = AnalogsRaw.y1;
          break;
        case 3:
          temp3 = Analogs.x2;
          temp5 = AnalogsRaw.x2;
          break;
        case 4:
          temp3 = Analogs.y2;
          temp5 = AnalogsRaw.y2;
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
      
    // sinal fora do lugar (na pagina errada)
    // /*@u8g@*/   /*u8g.drawLine(105, 12, 105, 12);
    // /*@u8g@*/   u8g.drawLine(104, 9, 106, 9); u8g.drawLine(103, 10, 103, 10); u8g.drawLine(107, 10, 107, 10);
    // /*@u8g@*/   u8g.drawLine(101, 8, 102, 7); u8g.drawLine(109, 8, 108, 7);   u8g.drawLine(103, 6, 107, 6); 
    // /*@u8g@*/   u8g.drawLine(99, 6, 100, 5); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(110, 5, 111, 6); u8g.drawLine(101, 4, 102, 4); u8g.drawLine(108, 4, 109, 4);   u8g.drawLine(103, 3, 107, 3); // signal icon //u8g.drawLine(99, 6, 101, 4); u8g.drawLine(109, 4, 111, 6);   u8g.drawLine(102, 3, 108, 3); // signal icon
    // /*@u8g@*/   u8g.setPrintPos(113, 13); u8g.print(ratio); // signal indicator*/
      
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
  /*@u8g@*/   byte temp1 = map(Analogs.y1, 0, AnalogsMaxValue, 0, 42);
  /*@u8g@*/   u8g.drawFrame(7, 1, 6, 44);
  /*@u8g@*/   if(temp1!=0) u8g.drawBox(8, 44-temp1, 4, temp1);
  /*@u8g@*/ 
  /*@u8g@*/   byte temp2 = map(Analogs.x1, 0, AnalogsMaxValue, 0, 42);
  /*@u8g@*/   u8g.drawFrame(13, 45, 44, 6);
  /*@u8g@*/   u8g.drawBox(14, 46, temp2, 4);
  /*@u8g@*/ 
  /*@u8g@*/   u8g.drawBox(13+(temp2==42?41:temp2), 42-(temp1==42?41:temp1), 3, 3);
  /*@u8g@*/   
  /*@u8g@*/ 
  /*@u8g@*/   temp1 = map(Analogs.x2, 0, AnalogsMaxValue, 0, 42);
  /*@u8g@*/   u8g.drawFrame(69, 45, 44, 6);
  /*@u8g@*/   u8g.drawBox(70+temp1, 46, 42-temp1, 4);
  /*@u8g@*/ 
  /*@u8g@*/   temp2 = map(Analogs.y2, 0, AnalogsMaxValue, 0, 42);
  /*@u8g@*/   u8g.drawFrame(113, 1, 6, 44);
  /*@u8g@*/   if(temp2!=0) u8g.drawBox(114, 44-temp2, 4, temp2);
  /*@u8g@*/   
  /*@u8g@*/   u8g.drawBox(69+(temp1==42?41:temp1), 42-(temp2==42?41:temp2), 3, 3);
  /*@u8g@*/ 
  /*@u8g@*/   
  /*@u8g@*/   u8g.setPrintPos(6, 64);
  /*@u8g@*/   u8g.print(mapAnalogToByte(Analogs.y1));
  /*@u8g@*/   u8g.setPrintPos(34, 64);
  /*@u8g@*/   u8g.print(mapAnalogToByte(Analogs.x1));
  /*@u8g@*/   u8g.setPrintPos(68, 64);
  /*@u8g@*/   u8g.print(mapAnalogToByte(Analogs.x2));
  /*@u8g@*/   u8g.setPrintPos(96, 64);
  /*@u8g@*/   u8g.print(mapAnalogToByte(Analogs.y2));
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
  if(now - analogsRawLastSum > histSumTime)
  {
    analogsRawLastSum = now;

    histSumCount--; // reduces 1 because the last sum was not executed

    for(byte i = 0; i < 4; i++)
    {
      if(analogsRawSum[i]*100/histSumCount <= maxVariationtionToStartHolding)
      {
        analogsRawNotHolding[i] = false;
      }

      analogsRawSum[i] = 0;
    }

    histSumCount = 0;
  }

  computeFilter(&AnalogsRaw.x1, &Analogs.x1, &TX_Pack.x1, inputX1, 0);
  computeFilter(&AnalogsRaw.y1, &Analogs.y1, &TX_Pack.y1, inputY1, 1);
  computeFilter(&AnalogsRaw.x2, &Analogs.x2, &TX_Pack.x2, inputX2, 2);
  computeFilter(&AnalogsRaw.y2, &Analogs.y2, &TX_Pack.y2, inputY2, 3);
  // Serial.println();

  #if logLimitAdnMiddleAdjust
  Serial.println(" |");
  #endif

//  Serial.print(AnalogsRaw.x1);
//  Serial.print(" ");
//  Serial.print(AnalogsRaw.y1);
//  Serial.print(" ");
//  Serial.print(AnalogsRaw.x2);
//  Serial.print(" ");
//  Serial.print(AnalogsRaw.y2);
//  Serial.println(" 1023 0");

  filterIndex++;
  if(filterIndex >= nFilters) filterIndex = 0;
}

void computeFilter(unsigned int *rawPointer, unsigned int *analogPointer, byte *txpackPointer, byte readingPin, byte axisNum)
{
  
  filters[axisNum][filterIndex] = analogRead(readingPin);
  unsigned int sumValues = 0;
  for(byte i = 0; i < nFilters; i++) sumValues += filters[axisNum][i];
  sumValues /= nFilters;

  int deviation = sumValues - *rawPointer;
  if(deviation < 0) deviation = -deviation;
  analogsRawSum[axisNum] += deviation;
  
  #if !logLimitAdnMiddleAdjust
  if(deviation > maxDeviationToEndHolding || analogsRawNotHolding[axisNum])
  {
  #endif
    analogsRawNotHolding[axisNum] = true;
    *rawPointer = sumValues;    
    *analogPointer = calcAnalogValue(rawPointer, &axisNum);
    *txpackPointer = map(*analogPointer, 0, AnalogsMaxValue, e.saida[1][axisNum], e.saida[0][axisNum]);
  #if !logLimitAdnMiddleAdjust
  }
  #endif

  // Serial.print(" ");
  // Serial.print(*txpackPointer);
}

// #define inBetween(number, value1, value2) ((number <= value1 && number >= value2)||((number >= value1 && number <= value2)))?true:false
#define inBetween(number, value1, value2) (number <= value1 && number >= value2)?(true):(number >= value1 && number <= value2)
// #define inBetweenCres(number, value1, value2) (number <= value1 && number >= value2)

unsigned int calcAnalogValue(unsigned int *rawPointer, byte *axisNumPoiner)
{
  unsigned int normalized = normalize(*rawPointer, e.ajMin[*axisNumPoiner], e.ajMax[*axisNumPoiner]);

    #if logLimitAdnMiddleAdjust
    #ifdef limitAdnMiddleAdjustAxis
    if(*axisNumPoiner == limitAdnMiddleAdjustAxis)
    {
    #endif

    Serial.print(" | ");

    Serial.print(*axisNumPoiner);
    Serial.print(") ");
    Serial.print(bitRead(e.midEnabled, *axisNumPoiner)?"E":"D");
    Serial.print(" ");

    Serial.print(e.ajMin[*axisNumPoiner]);
    Serial.print("-");
    Serial.print(e.mid[*axisNumPoiner][0]);
    Serial.print("~");
    Serial.print(e.mid[*axisNumPoiner][1]);
    Serial.print("-");
    Serial.print(e.ajMax[*axisNumPoiner]);

    Serial.print(" ");
    Serial.print(*rawPointer);
    Serial.print("→");
    #ifdef limitAdnMiddleAdjustAxis
    }
    #endif
    #endif

  if(bitRead(e.midEnabled, *axisNumPoiner)) // if the middle position is enabled
  {
    if(inBetween(normalized, e.ajMin[*axisNumPoiner], e.mid[*axisNumPoiner][0]))
    {
      #if logLimitAdnMiddleAdjust
      #ifdef limitAdnMiddleAdjustAxis
      if(*axisNumPoiner == limitAdnMiddleAdjustAxis)
      {
      #endif
        Serial.print(map(normalized, e.ajMin[*axisNumPoiner], e.mid[*axisNumPoiner][0], 0, AnalogsMiddleValue));
        Serial.print(" ←");
      #ifdef limitAdnMiddleAdjustAxis
      }
      #endif
      #endif

      return map(normalized, e.ajMin[*axisNumPoiner], e.mid[*axisNumPoiner][0], 0, AnalogsMiddleValue);
    }
    else if(inBetween(normalized, e.ajMax[*axisNumPoiner], e.mid[*axisNumPoiner][1]))
    {
      #if logLimitAdnMiddleAdjust
      #ifdef limitAdnMiddleAdjustAxis
      if(*axisNumPoiner == limitAdnMiddleAdjustAxis)
      {
      #endif
        Serial.print(map(normalized, e.mid[*axisNumPoiner][1], e.ajMax[*axisNumPoiner], AnalogsMiddleValue, AnalogsMaxValue));
        Serial.print(" →");
      #ifdef limitAdnMiddleAdjustAxis
      }
      #endif
      #endif

      return map(normalized, e.mid[*axisNumPoiner][1], e.ajMax[*axisNumPoiner], AnalogsMiddleValue, AnalogsMaxValue);
    }
    else
    {
      #if logLimitAdnMiddleAdjust
      #ifdef limitAdnMiddleAdjustAxis
      if(*axisNumPoiner == limitAdnMiddleAdjustAxis)
      {
      #endif
        Serial.print(AnalogsMiddleValue);
        Serial.print(" °");
      #ifdef limitAdnMiddleAdjustAxis
      }
      #endif
      #endif

      return AnalogsMiddleValue;
    }
  }
  else
  {
    #if logLimitAdnMiddleAdjust
    #ifdef limitAdnMiddleAdjustAxis
    if(*axisNumPoiner == limitAdnMiddleAdjustAxis)
    {
    #endif
      Serial.print(map(normalized, e.ajMin[*axisNumPoiner], e.ajMax[*axisNumPoiner], 0, AnalogsMaxValue));
      Serial.print(" X");
    #ifdef limitAdnMiddleAdjustAxis
    }
    #endif
    #endif

    return map(normalized, e.ajMin[*axisNumPoiner], e.ajMax[*axisNumPoiner], 0, AnalogsMaxValue);
  }
}

void openMainMenu()
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
        
        for(int i = 0; i < nFilters; i++)
        {
          readAnalogs();
        }

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

            #ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
            if(menu == 0 && item == 2)
            {
              enterFailSafeMode();
            }
            else if(menu == 0 && item == 3)
            {
              enterGpsRescueMode();
            }
            else
            {
            #endif
              if(itensToMenu[menu][item]!=255)
              {
                menu = itensToMenu[menu][item];
                item = pag(menu, 0);
              }
            #ifdef AUX_2_FAIL_SAFE_AND_GPS_RESCUE
            }
            #endif

          }
          else if(f1==3)
          {
            if(menu == 0)
            {
              refreshMenuPrincipal(true);
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

void drawValue1(unsigned int valor)
{
  /*@u8g@*/ u8g.setDefaultForegroundColor();
  /*@u8g@*/ u8g.drawFrame(27, 47, 96, 10);
  /*@u8g@*/ u8g.drawBox(28, 48, map(valor, 0, AnalogsMaxValue, 0, 94), 8);
  
  /*@u8g@*/ u8g.setPrintPos(1, 56);
  /*@u8g@*/ u8g.print(mapAnalogToByte(valor));
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

byte mapAnalogToByte(unsigned int val)
{
  return map(val, 0, AnalogsMaxValue, 0, 255);
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
