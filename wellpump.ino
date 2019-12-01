/*
   Управление колодезным насосом
   
   Закачка воды в бак с двумя датчиками - нижний уровень и верхний
   Ограничение по времени заполнения бака
   Переход в аварийный режим при "непредсказуемом" поведении датчиков
   Вывод диагностической информации на LCD 1602 (16 символов, 2 строки)

   Рабочий режим
   | 1234567890123456 |
   | ABRLHO 000S 0000
   | STA msg456789012

   Доп режм - вкл по кнопке, возврат к рабочему режиму через 10 сек
   | 1234567890123456 |
   | T  0000s  000000
   | Cnt 000 Leak 000

*/

//#define FF(a) (a)

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )

//#define DEBUG

//#define NO_WDT_WatchDog
//#include <WatchDog.h>               //  https://github.com/nadavmatalon/WatchDog

//#define NO_WDT_LowPower

#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power
//#include <ArduinoLowPower.h>          // https://www.arduino.cc/en/Reference/ArduinoLowPower
#include <avr/wdt.h>

// #define DISABLE_LOGGING
#include <ArduinoLog.h>             // https://github.com/thijse/Arduino-Log/
typedef const __FlashStringHelper* fchar;
//const char LOG_AS[] PROGMEM =

fchar tmp_str = NULL;
//#define FF(a)       ( tmp_str = F(a))
#define FF(a)       F(a)

#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
#include "Chrono.h"


// include the library code:
#include <LiquidCrystal_PCF8574.h>
//#include <LiquidCrystal_I2C_OLED.h>

#define  LCD_PORT 0x27
LiquidCrystal_PCF8574 lcd(LCD_PORT);


//***********************************************************************************************************************************************************

#define PIN_OVERFLOW        PD5
#define PIN_HIGHMARK        PD6
#define PIN_LOWMARK         PD7

#define SW_LOW_MARK         7
#define SW_HIGH_MARK        6
#define SW_OVERFLOW         5

#define SW_MOTOR            8
#define SW_VALVE            9

#define LOWMARK_TIMEOUT     10 // s
#define HIGHMARK_TIMEOUT    427 // s = 0,468л/с 200 литров 
#define LCD_TIMEOUT         20 // s
#define LCD_STAT_TIME       10 // s

typedef enum : byte { POWEROFF = 0,  ABORT = 1} t_ShutdownMode;
typedef enum : byte { PUMP_OFF = 0,  PUMP_ABORT = 1, PUMP_ON = 2} t_PumpStatus;

void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void Shutdown(fchar s1, fchar s2, t_ShutdownMode mode);
void Shutdown(const char*  s1, const char*  s2, t_ShutdownMode mode);

//******************************************************************************************
//  EVENT_NONE EVENT_CHANGED EVENT_PRESSED EVENT_RELEASED
DebounceEvent sw_Overflow   = DebounceEvent(SW_OVERFLOW,    btn_Overflow,  BUTTON_PUSHBUTTON );
DebounceEvent sw_LowMark   = DebounceEvent(SW_LOW_MARK,   btn_LowMark,  BUTTON_PUSHBUTTON );
DebounceEvent sw_HighMark  = DebounceEvent(SW_HIGH_MARK,  btn_HighMark, BUTTON_PUSHBUTTON );

//BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
//DebounceEvent button = DebounceEvent(BUTTON_PIN, BUTTON_SWITCH | BUTTON_SET_PULLUP);

Chrono TankTimeout(Chrono::SECONDS);
Chrono FlowTimeout(Chrono::SECONDS);
Chrono LCD_Timeout(Chrono::SECONDS);

volatile uint8_t f_Overflow = 0;
volatile uint8_t f_LowMark = 0;
volatile uint8_t f_HighMark = 0;

volatile uint8_t f_WDT = 0;

t_PumpStatus pump 		= PUMP_OFF;

// statistic
int 		 pump_cnt			= 0;
long 		 pump_total_time	= 0;
int 		 pump_last_time		= 0;

//******************************************************************************************
//#define LOG(func, s) { Log.func( s ); lcd.print( s ); }
//#define LOG1(func, s, a1) { Log.func( s, a1 ); lcd.print( a1 ); }
//#define LOG2(func, s, a1, a2) { Log.func( s, a1, a2 ); lcd.print( a1 ); lcd.print( a2 ); }
//#define LOG3(func, s, a1, a2, a3) { Log.func( s, a1, a2, a3 ); lcd.print( a1 ); lcd.print( a2 ); lcd.print( a3 );  }

void lcd_on(bool timeout = false)
{
  lcd.setBacklight(255);
  lcd.clear();

  if (timeout)
    LCD_Timeout.restart();

}

inline char sw(bool f)
{
  return f ? 'F' : '_';
}

void lcd_status()
{
  char *s;
  lcd.setCursor(13, 0);
  switch (pump)
  {
    case PUMP_OFF:
      s = "OFF";
      break;
    case PUMP_ABORT:
      s = "ABR";
      break;
    case PUMP_ON:
      s = "ON!";
      break;
    default:
      s = "INV";  // INVALID
  }
  lcd.print( s );
  lcd.setCursor(13, 1);
  lcd.print( sw(sw_Overflow.pressed()) );
  lcd.print( sw(sw_HighMark.pressed()) );
  lcd.print( sw(sw_LowMark.pressed())  );
}

void lcd_statistic()
{
  lcd_on();

  lcd.print(FF("Ready! Cnt: "));
  lcd.print(pump_cnt);

  lcd.setCursor(0, 1);
  lcd.print(FF("LT: "));
  lcd.print(pump_last_time);
  lcd.print(FF("TT: "));
  lcd.print(pump_total_time);
}

void PumpOff()
{
  Log.notice( FF("Pump OFF - " ) );

  //noInterrupts();
  digitalWrite(SW_VALVE, LOW);
  digitalWrite(SW_MOTOR, LOW);
  //interrupts();

  // stat
  pump_total_time += pump_last_time = TankTimeout.elapsed();

  Log.notice( FF( "done: %i seconds" CR), pump_last_time );

  lcd_on( true );
  lcd.print ( FF("Pump OFF: " ) ); lcd.print ( pump_last_time ); lcd.print ( FF("s"));
  // LCD_Timeout.restart(); в lcd_on

  FlowTimeout.stop();
  TankTimeout.stop();

  pump = PUMP_OFF;
}



// включает двигатель, если все нормально и запускает таймеры переполнения и нижнего датчика
void PumpOn()
{

  if (pump == PUMP_ABORT)
  {
    Shutdown(
      FF("Pump ABORT mode!" ),
      FF("Start failed!!!" ), ABORT);
    return;
  }

  Log.notice( FF("Pump ON - " ) );

  // stat
  pump_cnt++;

  digitalWrite(SW_VALVE, HIGH);
  digitalWrite(SW_MOTOR, HIGH);

  Log.notice( FF( "done" CR) );

  lcd_on();
  lcd.print( FF("Pump ON!!!" ));

  FlowTimeout.restart();
  TankTimeout.restart();
  pump = PUMP_ON;
}

// уснуть - до следующего поплавка (POWEROFF) или навсегда
void _shutdown(t_ShutdownMode mode = POWEROFF)
{

#ifdef DEBUG
  return;
#endif

  if (mode == POWEROFF)
  {
    delay(LCD_TIMEOUT * 1000);
    lcd.setBacklight(0);
  }
  else {
    // сделать блокировку до полного отключения
    PCICR = 0;
    PCMSK2 = 0;

    pump = PUMP_ABORT;

    // чтобы байтики через serial прошли
    delay(1000);

    // не просыпаться никогда
    noInterrupts();
  }

#ifdef LowPower_h
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#endif
}

// печатает текст и останавливает устройство
void Shutdown(fchar s1, fchar s2, t_ShutdownMode mode = POWEROFF)
{
  lcd_on();

  lcd.print( s1 );
  Log.fatal( "%S " CR, s1 );

  PumpOff();
#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;

  lcd.setCursor( 0, 1);
  lcd.print( s2 );
  Log.fatal( "%S " CR, s2 );


  _shutdown(mode);
}

/*
  void Shutdown(const char* s1, const char* s2, t_ShutdownMode mode = POWEROFF)
  {
  Log.fatal( s1 );
  PumpOff();
  #ifdef WatchDog_h
  WatchDog::stop();
  #endif
  f_WDT = 0;
  Log.fatal( s2 );

  _shutdown(mode);
  }
  /**/

void A_Watchdog()
{
  wdt_disable();
  //f_WDT = 1;
  SETFLAG(f_WDT);
  //PumpOff();
}

//***********************************************************************************************************************************************************

void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.verbose( FF(CR "LowMark: %i %x" CR), f_LowMark, event);
  //f_LowMark = 0;

  switch (event)
  {
    case EVENT_PRESSED:
      if (sw_HighMark.pressed()) {
#ifdef WatchDog_h
        WatchDog::start();
#endif
        PumpOn();
      } else {
        Shutdown(
          FF("INVALID sensors!"),
          FF("Emergency STOP!"), ABORT);
      }
      break;
    case EVENT_RELEASED:
      FlowTimeout.stop();
      break;
    default:
      Shutdown(
        FF("Lowmark FATAL!" ),
        FF("Emergency STOP!.." ), ABORT);
  }

}

void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.verbose(FF(CR "HighMark: %i %x" CR), f_HighMark, event);
  //f_HighMark = 0;

  //  must be pressed before
  switch (event)
  {
    case EVENT_PRESSED:
      // вода начала кончаться
      break;
    case EVENT_RELEASED:
      if (pump != PUMP_ON)
      {
        Shutdown(
          FF("Highmark ERROR!"),
          FF("Check sensor!"), ABORT);
      }
      else {
        PumpOff();

        // Shutdown(
        // FF("FULL TANK!" ),
        // FF("Success! :)" ));
      }
      break;
    default:
      Shutdown(
        FF("Highmark FATAL!" ),
        FF("Emergency STOP!.." ), ABORT);
  }
}

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.verbose(FF(CR "OVERFLOW: %i %x" CR), f_Overflow, event);

  // any change is dangerous!!!
  Shutdown(
    FF("OVERFLOW!!!" ),
    FF("Emergency STOP!"),
    ABORT);
}

//***********************************************************************************************************************************************************

volatile uint8_t portBhistory = 0x00;     // default is high because the pull-up
volatile uint8_t portChistory = 0x00;
volatile uint8_t portDhistory = 0x00;     // не pool-up
volatile long t = 0;

ISR(PCINT2_vect)
{
  t = millis();
  uint8_t changedbits;

  //PCIMSK2
  changedbits = PIND ^ portDhistory;
  portDhistory = PIND;

  if (changedbits & (1 << PIN_OVERFLOW))
  {
    SETFLAG(f_Overflow);
  }

  if (changedbits & (1 << PIN_HIGHMARK))
  {
    SETFLAG(f_HighMark);
  }

  if (changedbits & (1 << PIN_LOWMARK))
  {
    SETFLAG(f_LowMark);
  }

}

ISR(PCINT0_vect)
{
  t = millis();
  uint8_t changedbits;

  //PCIMSK0
  changedbits = PINB ^ portBhistory;
  portBhistory = PIND;

  //  if (changedbits & (1 << PIN_OVERFLOW))
  //  {
  //    SETFLAG(f_Overflow);
  //  }
  //
  //  if (changedbits & (1 << PIN_HIGHMARK))
  //  {
  //    SETFLAG(f_HighMark);
  //  }
  //
  //  if (changedbits & (1 << PIN_LOWMARK))
  //  {
  //    SETFLAG(f_LowMark);
  //  }

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
  //if (++f_WDT == 0) ++f_WDT;
  SETFLAG(f_WDT);
}

//***********************************************************************************************************************************************************


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.print(FF("In the begining...") );
  Serial.println();

  lcd.begin(16, 2);
  lcd_on();
  lcd.print(FF("Starting..."));
  //  lcd.setCursor(0,1);
  //  lcd.print("Starting...");

  // LOG_LEVEL_VERBOSE
  // LOG_LEVEL_TRACE
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Log.notice( FF(CR "******************************************" CR) );                     // Info string with Newline

#ifdef WatchDog_h
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif

  pinMode(SW_MOTOR, OUTPUT);
  pinMode(SW_VALVE, OUTPUT);
  PumpOff();

  lcd.setCursor(0, 1);
  lcd.print( FF("READY!!!") );

  //PORTD &= 0b11100000;
  // разрешение прерываний INT0 и INT1
  //  EIMSK  =  (1<<INT0)  | (1<<INT1);
  // настройка срабатывания прерываний на любому изменению
  //EICRA  =  (0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);
  // разрешение прерываний с портов B (PCINT[7:0]) и D (PCINT[23:16]), и запрет с порта C (PCINT[14:8])
  PCICR  |= (1 << PCIE2) | (0 << PCIE1) | (0 << PCIE0);
  // маскирование всех ног, кроме PB0 и PD7 - по одной на PCINT0 и PCINT2
  //PCMSK0 |= (0 << PCINT7)  | (0 << PCINT6)  | (0 << PCINT5)  | (0 << PCINT4)  | (0 << PCINT3)  | (0 << PCINT2)  | (0 << PCINT1)  | (1 << PCINT0);
  //PCMSK1 |=                (0 << PCINT14) | (0 << PCINT13) | (0 << PCINT12) | (0 << PCINT11) | (0 << PCINT10) | (0 << PCINT9)  | (0 << PCINT8);
  PCMSK2 |= (1 << PCINT23) | (1 << PCINT22) | (1 << PCINT21) | (0 << PCINT20) | (0 << PCINT19) | (0 << PCINT18) | (0 << PCINT17) | (0 << PCINT16);

}

//***********************************************************************************************************************************************************
uint8_t l_cnt = 0;
#define CNT_DIV 3

void loop() {

#ifdef LowPower_h
  if (pump == PUMP_OFF)
  {
    LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  }
  else
#endif
    delay(10);


#ifdef LowPower_h
  //LowPower.idle(.....
  if ( ( ( (++l_cnt) & 0b11111 ) == 0b10000 ) )
#else
  //delay(10);
  if ( ( ( (++l_cnt) & 0b111111 ) == 0b100000 ) )
#endif
  {
    unsigned long e1 =   FlowTimeout.elapsed();
    unsigned long e2 =   TankTimeout.elapsed();
    //Log.trace( FF("t = %l f_WDT %T, f_Overflow %T, f_LowMark %T, f_HighMark %T, portBhistory %x, portChistory %x, portDhistory %x" CR),
    Log.trace( FF("l_cnt = %x, t = %l f_WDT %i, f_Overflow %i, f_LowMark %i, f_HighMark %i, portBhistory %x, portChistory %x, portDhistory %x, FlowTimeout = %l, TankTimeout = %l" CR),
               l_cnt, t, f_WDT, f_Overflow, f_LowMark, f_HighMark,
               portBhistory, portChistory, portDhistory, e1, e2);
  }


  if (0 & f_WDT) {
    Shutdown(
      FF("WATCHDOG ALERT!" ),
      FF("Emergency STOP!" ),
      ABORT);
  }


  if (LCD_Timeout.hasPassed(LCD_TIMEOUT)) {
    lcd.setBacklight(0);
    _shutdown();
  }
  else if (LCD_Timeout.hasPassed(LCD_STAT_TIME))
  {
    lcd_statistic();
  }

  if (FlowTimeout.hasPassed(LOWMARK_TIMEOUT)) {
    Shutdown(
      FF("NO FLOW alert!" ),
      FF("Chk WATER VALVES" ),
      ABORT);
  }

  if (TankTimeout.hasPassed(HIGHMARK_TIMEOUT)) {
    TankTimeout.stop();
    Shutdown(
      FF("TANK OVERFLOW!" ),
      FF("Chk TANK & SENS!" ),
      ABORT);
  }

  // put your main code here, to run repeatedly:

  sw_Overflow.loop();
  sw_HighMark.loop();
  sw_LowMark.loop();

  lcd_status();

  if (sw_LowMark.pressed() && !sw_HighMark.pressed()) {
    Shutdown(
      FF("INVALID sensors!" ),
      FF("Emergency STOP!" ),
      ABORT);
  }

}
