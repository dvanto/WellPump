#define FF(a) (a)

//#define NO_WDT_WatchDog
//#include <WatchDog.h>               //  https://github.com/nadavmatalon/WatchDog

//#define NO_WDT_LowPower
#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power
#include <avr/wdt.h>

// #define DISABLE_LOGGING
#include <ArduinoLog.h>             // https://github.com/thijse/Arduino-Log/
typedef const __FlashStringHelper* fchar;

#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
#include "Chrono.h"

/*
  LiquidCrystal Library - Autoscroll

  Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
  library works with all LCD displays that are compatible with the
  Hitachi HD44780 driver. There are many of them out there, and you
  can usually tell them by the 16-pin interface.

  This sketch demonstrates the use of the autoscroll()
  and noAutoscroll() functions to make new text scroll or not.

  The circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)

  Library originally added 18 Apr 2008
  by David A. Mellis
  library modified 5 Jul 2009
  by Limor Fried (http://www.ladyada.net)
  example added 9 Jul 2009
  by Tom Igoe
  modified 22 Nov 2010
  by Tom Igoe
  modified 7 Nov 2016
  by Arturo Guadalupi

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/LiquidCrystalAutoscroll

*/

// include the library code:
#include <LiquidCrystal.h>

//***********************************************************************************************************************************************************

#define SW_LOW_MARK         5
#define SW_HIGH_MARK        6
#define SW_LEAKAGE          7

#define SW_MOTOR            8
#define SW_VALVE            9

#define LOWMARK_TIMEOUT     10 // s
#define HIGHMARK_TIMEOUT    250 // s = 250 сек = ~200 литров 

void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_Leakage(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);

DebounceEvent sw_LowMark   = DebounceEvent(SW_LOW_MARK,   btn_LowMark,  BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent sw_HighMark  = DebounceEvent(SW_HIGH_MARK,  btn_HighMark, BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
DebounceEvent sw_Leakage   = DebounceEvent(SW_LEAKAGE,    btn_Leakage,  BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
//DebounceEvent button = DebounceEvent(BUTTON_PIN, BUTTON_SWITCH | BUTTON_SET_PULLUP);

Chrono TankTimout(Chrono::SECONDS);
Chrono FlowTimout(Chrono::SECONDS);

volatile uint8_t f_Leakage = 0;
volatile uint8_t f_LowMark = 0;
volatile uint8_t f_HighMark = 0;

//******************************************************************************************

void PumpOff()
{
  Log.notice(F(CR "Pump OFF - " ));

  noInterrupts();
  digitalWrite(SW_VALVE, LOW);
  digitalWrite(SW_MOTOR, LOW);
  interrupts();

  Log.notice(F( "done" CR));
  FlowTimout.stop();
  TankTimout.stop();
}

void PumpOn()
{
  Log.notice(F(CR "Pump ON - " ));

  digitalWrite(SW_VALVE, HIGH);
  digitalWrite(SW_MOTOR, HIGH);

  Log.notice(F( "done" CR));
  FlowTimout.restart();
  TankTimout.restart();
}

volatile uint8_t f_WDT = 0;

typedef enum : byte {
  POWEROFF = 0,
  ABORT    = 1
} ShutdownMode;

void Shutdown(fchar s1, fchar s2, ShutdownMode mode = POWEROFF)
{
  Log.fatal( s1 );
  PumpOff();
#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;
  Log.fatal( s2 );
  if (mode == POWEROFF)
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  else
    // сделать блокировку до полного отключения
    LowPower.powerStandby(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void Shutdown(const char* s1, const char* s2, ShutdownMode mode = POWEROFF)
{
  Log.fatal( s1 );
  PumpOff();
#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;
  Log.fatal( s2 );
  if (mode == POWEROFF)
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  else
    // сделать блокировку до полного отключения
    LowPower.powerStandby(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void A_Watchdog()
{
  wdt_disable();
  f_WDT = 1;
  PumpOff();
}

//***********************************************************************************************************************************************************

void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.notice(F(CR "LowMark: %i" CR), f_LowMark);
  f_LowMark = 0;

  if (event == EVENT_PRESSED) {
    if (sw_HighMark.pressed()) {
#ifdef WatchDog_h
      WatchDog::start();
#endif
      PumpOn();
    } else {
      Shutdown(
        F("INVALID sensors' status!" CR "Emergency STOP!!!"),
        F("Emergency STOP on SENSORS ALERT!!!"));
    }
  }

}

void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.notice(F(CR "HighMark: %i" CR), f_HighMark);
  f_HighMark = 1;

  //  must be pressed before
  if (event == EVENT_RELEASED) {
    Shutdown(
      F("Полный бак!"),
      F("Успешная остановка! :)" ));
    //    PumpOff();
    //    WatchDog::stop();
    //    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
}

void btn_Leakage(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.notice(F(CR "LEAKAGE: %i" CR), f_Leakage);

  // any change is dangerous!!!
  Shutdown(
    F("Leakage detected!!!" CR "Emergency STOP!!!"),
    F("Emergency STOP on LEAKAGE!!!" ));
}

//***********************************************************************************************************************************************************

#define PIN_LEAKAGE         PINB0
#define PIN_HIGHMARK        PINB1
#define PIN_LOWMARK         PINB2

volatile uint8_t portBhistory = 0xFF;     // default is high because the pull-up
volatile uint8_t portChistory = 0xFF;
volatile uint8_t portDhistory = 0xFF;

ISR(PCINT0_vect)
{
  uint8_t changedbits;

  //PCIMSK1
  changedbits = PINB ^ portBhistory;
  portBhistory = PINB;

  changedbits = PINC ^ portChistory;
  portChistory = PINC;

  changedbits = PIND ^ portDhistory;
  portDhistory = PIND;

  if (changedbits & (1 << PIN_LEAKAGE))
  {
    f_Leakage = 1;
  }

  if (changedbits & (1 << PIN_HIGHMARK))
  {
    f_HighMark = 1;
  }

  if (changedbits & (1 << PIN_LOWMARK))
  {
    f_LowMark = 1;
  }

}

//ISR(INT1_vect)
//{
//
//}

//volatile uint8_t  = 0;
ISR(WDT_vect) {
#ifdef WatchDog_h
  if (Wdog1.active())   {
    if (WatchDog::ovfCounter < WatchDog::ovfTop) {
      WatchDog::ovfCounter++;
    } else {
      WatchDog::ovfCounter = 0;
      Wdog1.isrCallback();
    }
  } else
#endif
    wdt_disable();
  f_WDT = 1;
}

//***********************************************************************************************************************************************************


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.print(F("In the begining...") );
  Serial.println();

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Log.notice(F(CR "******************************************" CR));                     // Info string with Newline
  //Log.notice(  "***          Logging example                " CR);                       // Info string in flash memory
  //Log.notice(F("******************* ")); Log.notice("*********************** " CR);      // two info strings without newline

#ifdef WatchDog_h
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif

  pinMode(SW_MOTOR, OUTPUT);
  pinMode(SW_VALVE, OUTPUT);
  PumpOff();

}

//***********************************************************************************************************************************************************

void loop() {

  if (f_WDT) {
    Shutdown(
      F("Watchdog alert!" CR "Emergency STOP!!!"),
      F("Emergency STOP on WATCHDOG ALERT!!!"));
  }

  if (FlowTimout.hasPassed(LOWMARK_TIMEOUT)) {
    Shutdown(
      F("FLOW alert!" CR "Emergency STOP!!!"),
      F("Check water valves!!!")
    );
  }

  if (TankTimout.hasPassed(HIGHMARK_TIMEOUT)) {
    Shutdown(
      F("TANK is FULL alert!" CR "Emergency STOP!!!"),
      F("Check TANK and SENSORS!!!"));
  }

  // put your main code here, to run repeatedly:
  sw_LowMark.loop();
  sw_Leakage.loop();
  sw_HighMark.loop();

  if (sw_LowMark.pressed() && !sw_HighMark.pressed()) {
    Shutdown(
        F("INVALID sensors' status! (in loop)" CR "Emergency STOP!!!"),
        F("Emergency STOP on SENSORS ALERT!!!"));
  }


  LowPower.idle(SLEEP_500MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  /*
    if (sw_LowMark.loop() != EVENT_NONE) {
      Serial.print(F("Switched - loop: "));  Serial.println( (sw_LowMark.getEventLength() / 1000) );
    }
    if (sw_Leakage.loop() != EVENT_NONE) {
      Serial.print(F("Switched - loop: "));  Serial.println( (sw_LowMark.getEventLength() / 1000) );
    }
    if (sw_HighMark.loop() != EVENT_NONE) {
      Serial.print(F("Switched - loop: "));  Serial.println( (sw_LowMark.getEventLength() / 1000) );
    }
  */
}
