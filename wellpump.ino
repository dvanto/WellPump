/*
   Управление колодезным насосом

   Закачка воды в бак с двумя датчиками - нижний уровень и верхний
   Ограничение по времени заполнения бака
   Переход в аварийный режим при "непредсказуемом" поведении датчиков
   Вывод диагностической информации на LCD 1602 (16 символов, 2 строки)

   Рабочий режим
   | 1234567890123456 |
   | ABRLHO 000s 0000
   | STA msg456789012

   Доп режм - вкл по кнопке, возврат к рабочему режиму через 10 сек
   mode  0
   | 1234567890123456 |
   | T  0000s  000000
   | Cnt 000"Tank Timout 000

*/

//#define FF(a) (a)

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )  // если увеличение дало 0, то увеличиваем еще раз

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


#include <LiquidCrystal_PCF8574.h>
//#include <LiquidCrystal_I2C_OLED.h>

#define  LCD_PORT 0x27
LiquidCrystal_PCF8574 lcd(LCD_PORT);


//*******************************************************************************************

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
#define SPEED_LPS           0.468 // liters per second
#define LCD_TIMEOUT         20 // s
#define LCD_STAT_TIME       10 // s
#define STAT_DELAY			 100 // ms


//******************************************************************************************
//  EVENT_NONE EVENT_CHANGED EVENT_PRESSED EVENT_RELEASED
//  BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
//  DebounceEvent button = DebounceEvent(BUTTON_PIN, BUTTON_SWITCH | BUTTON_SET_PULLUP);

volatile uint8_t f_Overflow = 0;
volatile uint8_t f_LowMark = 0;
volatile uint8_t f_HighMark = 0;

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);

DebounceEvent sw_Overflow  = DebounceEvent(SW_OVERFLOW,   btn_Overflow, BUTTON_PUSHBUTTON );
DebounceEvent sw_LowMark   = DebounceEvent(SW_LOW_MARK,   btn_LowMark,  BUTTON_PUSHBUTTON );
DebounceEvent sw_HighMark  = DebounceEvent(SW_HIGH_MARK,  btn_HighMark, BUTTON_PUSHBUTTON );

Chrono TankTimeout(Chrono::SECONDS);
Chrono FlowTimeout(Chrono::SECONDS);
Chrono LCD_Timeout(Chrono::SECONDS);
Chrono StatDelay(Chrono::MILLIS);


volatile uint8_t f_WDT = 0;

//******************************************************************************************
typedef enum : byte { POWEROFF = 0,  ABORT = 1} t_ShutdownMode;

void Shutdown(fchar s1, //fchar s2,
              t_ShutdownMode mode);
void Shutdown(const char*  s1, //const char*  s2,
              t_ShutdownMode mode);

void log_internal_state();
							


//******************************************************************************************
typedef enum : byte { PUMP_OFF = 0,  PUMP_ABORT = 1, PUMP_ON = 2} t_PumpStatus;

t_PumpStatus pump 		= PUMP_OFF;

// statistic
int 		fv_pump_cnt			      = 0;
int     fv_pump_owfl          = 0;
long 		fv_pump_total_time    = 0;
long    fv_pump_total_volume  = 0;
int 		pump_last_time        = 0;
int     pump_last_volume      = 0;

//******************************************************************************************
//#define LOG(func, s) { Log.func( s ); lcd.print( s ); }
//#define LOG1(func, s, a1) { Log.func( s, a1 ); lcd.print( a1 ); }
//#define LOG2(func, s, a1, a2) { Log.func( s, a1, a2 ); lcd.print( a1 ); lcd.print( a2 ); }
//#define LOG3(func, s, a1, a2, a3) { Log.func( s, a1, a2, a3 ); lcd.print( a1 ); lcd.print( a2 ); lcd.print( a3 );  }

void lcd_on(bool timeout = false)
{
  lcd.setBacklight(255);
  lcd.clear();

	Log.verbose("lcd_on %d restart = %c" CR, __LINE__, timeout?'R':'-' );
	
  if (timeout)
    LCD_Timeout.restart();

}


char* sprintTime4(char* s, unsigned long v)
{
  if (v < 1000)
    sprintf(s, "%03lus", v);            // 000s
  else if (v < 100*60)
    sprintf(s, "%03lum", v/60);         // 000m
  else if (v < (unsigned long)10*60*60)
    sprintf(s, "%3.1fh", (float)v/3600);// 0.0h
  else
    sprintf(s, "%03luh", v/3600);       // 000h

  return s;
}


char* sprintTime5(char* s, unsigned long v)
{
  if (v < 10000)
    sprintf(s, "%04lus", v);            // 0000s
  else if (v < 100*60)
    sprintf(s, "%4.1fm", (float)v/60);  // 00.0m
  else if (v < (unsigned long)10*60*60)
    sprintf(s, "%4.2fh", (float)v/3600);// 0.00h
  else if (v < (unsigned long)100*60*60)
    sprintf(s, "%4.1fh", (float)v/3600);// 00.0h
  else
    sprintf(s, "%04luh", v/3600);       // 0000h

  return s;
}


inline char sw(bool f, char c ='F') 
{
  return f ? c : '_';
}


char status_msg[]="INV\0OFF\0ABR\0ON!\0INV";

void lcd_status()
{
  char *s, c1, c2, c3;
  char s_time[5];
  char s_vol[5];

  lcd.setCursor(0, 0);
  switch (pump)
  {
    case PUMP_OFF:
      s = status_msg+4;
      break;
    case PUMP_ABORT:
      s = status_msg+8;  // ABORT
      break;
    case PUMP_ON:
      s = status_msg+12;
      break;
    default:
      s = status_msg;  // INVALID
  }
  lcd.print( s );
  lcd.print( c1 = sw(sw_Overflow.pressed(), 'O') );
  lcd.print( c2 = sw(sw_HighMark.pressed(), 'F') );
  lcd.print( c3 = sw( sw_LowMark.pressed(), 'E') );
  lcd.print( ' ' );

  lcd.print(sprintTime4(s_time, pump_last_time));
  lcd.print(' ');
  sprintf(s_vol, "%04u", pump_last_volume);
  lcd.print(s_vol);

  Log.notice("%s %c%c%c %s %s" CR, s, c1, c2, c3, s_time, s_vol);

  lcd.setCursor(0, 1);
}


void lcd_statistic(uint8_t mode=0)
{
  switch (mode)
  {
    case 0:
    default:  // пока так
      lcd.setCursor(0, 0);
      lcd.print("T  ");
      lcd.print(fv_pump_total_time);
      lcd.print("  ");
      lcd.print(fv_pump_total_volume);

      lcd.setCursor(0, 1);
      lcd.print("Cnt ");
      lcd.print(fv_pump_cnt);
      lcd.print("Ovfl ");
      lcd.print(fv_pump_owfl);
      break;
  }

  //lcd_on(true);
}

//******************************************************************************************

void PumpOff()
{
  Log.notice( FF("Pump OFF - " ) );

  //noInterrupts();
  digitalWrite(SW_VALVE, LOW);
  digitalWrite(SW_MOTOR, LOW);
  //interrupts();

  // stat
  fv_pump_total_time += pump_last_time = TankTimeout.elapsed();
  pump_last_volume = SPEED_LPS * pump_last_time;
  fv_pump_total_volume += pump_last_volume;

  Log.notice( FF( "done: %i seconds" CR), pump_last_time );

  lcd_status();

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
      //  1234567890123456
      FF("Start failed!!!" ),
      ABORT);
    return;
  }

  Log.notice( FF("Pump ON - " ) );

  // stat
  fv_pump_cnt++;

  digitalWrite(SW_VALVE, HIGH);
  digitalWrite(SW_MOTOR, HIGH);

  Log.notice( FF( "done" CR) );

  lcd_status();

  FlowTimeout.restart();
  TankTimeout.restart();
  StatDelay.restart();
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
    // FULL Shutdown !!!
    // сделать блокировку до полного отключения
    Log.verbose("shutdown %d" CR, __LINE__);
    PCICR = 0;
    PCMSK2 = 0;

    pump = PUMP_ABORT;

    // чтобы байтики через serial прошли
    delay(100);

    // не просыпаться никогда
		
		Log.verbose("powerDown %d" CR, __LINE__);
		delay(100);
		
    noInterrupts(); ///???
		LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    Log.verbose("wakeup %d" CR, __LINE__);
  }

#ifdef LowPower_h
  Log.verbose("powerDown %d" CR, __LINE__);
  delay(100);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  Log.verbose("wakeup %d" CR, __LINE__);
#endif
}


// печатает текст и останавливает устройство
void Shutdown(fchar s1, t_ShutdownMode mode = POWEROFF)
{
	
	PumpOff();
	
	if (mode != POWEROFF)
	{
		pump = PUMP_ABORT;
	}
	
  lcd_on(pump != PUMP_ABORT);
  lcd_status();
  lcd.print( s1 );

  Log.fatal( "%S " CR, s1 );

#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;

  _shutdown(mode);
}


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
          //  1234567890123456
          FF("INVALID sensors!"),
          ABORT);
      }
      break;
    case EVENT_RELEASED:
      FlowTimeout.stop();
      break;
    default:
      Shutdown(
        //  1234567890123456
        FF("LM FTL! Chk SENS" ),
        ABORT);
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
          //  1234567890123456
          FF("HM ERR! Chk SENS"),
          ABORT);
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
        //  1234567890123456
        FF("HM FTL! Chk SENS"),
        ABORT);
  }
}

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
  Log.verbose(FF(CR "OVERFLOW: %i %x" CR), f_Overflow, event);

  fv_pump_owfl++;

  // any change is dangerous!!!
  Shutdown(
    //  1234567890123456
    FF("OVRFLW DETECTED!" ),
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


void log_internal_state(int l_cnt)
{
    unsigned long e1 =   FlowTimeout.elapsed();
    unsigned long e2 =   TankTimeout.elapsed();
		unsigned long e3 =   LCD_Timeout.elapsed();
		unsigned long e4 =   StatDelay.elapsed();
						
    //Log.trace( FF("t = %l f_WDT %T, f_Overflow %T, f_LowMark %T, f_HighMark %T, portBhistory %x, portChistory %x, portDhistory %x" CR),
    Log.trace( FF("l_cnt = %x, t = %l f_WDT %i, f_Overflow %i, f_LowMark %i, f_HighMark %i, " CR
									"        portBhistory %x, portChistory %x, portDhistory %x, " CR
									"        FlowTimeout = %c %l, TankTimeout = %c %l, LCD_Timeout = %c %l, StatDelay = %c %l" CR),
               l_cnt, t, f_WDT, f_Overflow, f_LowMark, f_HighMark,
               portBhistory, portChistory, portDhistory, 
							 sw( FlowTimeout.isRunning(), 'R') , e1, sw( TankTimeout.isRunning(), 'R'), e2, 
							 sw( LCD_Timeout.isRunning(), 'R'), e3, sw( StatDelay.isRunning(), 'R'), e4
		);
}


//***********************************************************************************************************************************************************


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.print(FF("In the begining...") );
  Serial.println();

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd_on(true);
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

  lcd_status();
  lcd.print( FF("READY!!!") );
	StatDelay.start();

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
    Log.verbose("Sleep %d" CR, __LINE__);
		delay(100);
		LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
    Log.verbose("wakeup %d" CR, __LINE__);
  }
#endif

	l_cnt++;

  if ( StatDelay.hasPassed(STAT_DELAY, true) )
  {
		log_internal_state(l_cnt);
					
		if (pump == PUMP_ON) 
		{
			lcd_status();
		}
  }


  if (0 & f_WDT) 
  {
    Shutdown(
      //  1234567890123456
      FF("WATCHDOG ALERT!" ),
      ABORT);
  }


  if (LCD_Timeout.hasPassed(LCD_TIMEOUT)) 
  {
		// lcd_off:
    LCD_Timeout.stop();
    lcd.setBacklight(0);
		
    Log.verbose("Shutdown %d" CR, __LINE__);
    _shutdown();
    Log.verbose("wakeup %d" CR, __LINE__);
  }
  else if (LCD_Timeout.hasPassed(LCD_STAT_TIME))
  {
    lcd_statistic();
  }

  if (FlowTimeout.hasPassed(LOWMARK_TIMEOUT)) 
  {
    Shutdown(
      //  1234567890123456
      FF("NO FLOW! Chk WTR" ),
      ABORT);
  }

  if (TankTimeout.hasPassed(HIGHMARK_TIMEOUT)) 
  {
    Shutdown(
      //  1234567890123456
      FF("Tank Timout!" ),
      ABORT);
  }

  // put your main code here, to run repeatedly:

  sw_Overflow.loop();
  sw_HighMark.loop();
  sw_LowMark.loop();

  if (sw_LowMark.pressed() && !sw_HighMark.pressed()) 
  {
    Shutdown(
      //  1234567890123456
      FF("Sensors error!" ),
      ABORT);
  }

}
