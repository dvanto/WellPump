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
   | T 0000s L 000000
   | C 00000 OF 00000

*/

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )  // если увеличение дало 0, то увеличиваем еще раз

//#define DEBUG

//#define NO_WDT_WatchDog
//#include <WatchDog.h>               //  https://github.com/nadavmatalon/WatchDog

//#define NO_WDT_LowPower

#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power
#include <EEPROM.h>
// #include <avr/wdt.h>

// #define DISABLE_LOGGING
// * 0 - LOG_LEVEL_SILENT     no output 
// * 1 - LOG_LEVEL_FATAL      fatal errors 
// * 2 - LOG_LEVEL_ERROR      all errors  
// * 3 - LOG_LEVEL_WARNING    errors, and warnings 
// * 4 - LOG_LEVEL_NOTICE     errors, warnings and notices 
// * 5 - LOG_LEVEL_TRACE      errors, warnings, notices & traces 
// * 6 - LOG_LEVEL_VERBOSE    all 
#define LOG_LEVEL			LOG_LEVEL_TRACE
#include <ArduinoLog.h>             // https://github.com/thijse/Arduino-Log/
#include <avr/pgmspace.h>

typedef const __FlashStringHelper* fchar;
//const char LOG_AS[] PROGMEM =

//#define FF(a) 			(a)
#define FF(a)       F(a)

// const char STR_D_CR[] PROGMEM  = ("%s (line:%d)" CR);
fchar STR_D_CR;


#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
#include "Chrono.h"


#include <LiquidCrystal_PCF8574.h>
//#include <LiquidCrystal_I2C_OLED.h>

#define	LCD_PORT 						0x27
#define LCD_WIDTH						16
LiquidCrystal_PCF8574 			lcd(LCD_PORT);


//*******************************************************************************************

#define PIN_OVERFLOW        PD5
#define PIN_HIGHMARK        PD6
#define PIN_LOWMARK         PD7

#define SW_OVERFLOW         5
#define SW_HIGH_MARK        6
#define SW_LOW_MARK         7

#define SW_MOTOR            8
#define SW_VALVE            9

/* 
#define LOWMARK_TIMEOUT     10 // s
#define HIGHMARK_TIMEOUT    427 // s = 0,468л/с 200 литров
#define SPEED_LPS           0.468 // liters per second
#define LCD_TIMEOUT         (LCD_STAT_TIME+10) // s
#define LCD_STAT_TIME       10 // s
#define STAT_DELAY			 		500 // ms
 */
 
#define LOWMARK_TIMEOUT     3 // s
#define HIGHMARK_TIMEOUT    30 // s = 0,468л/с 200 литров
#define SPEED_LPS           0.468 // liters per second
#define LCD_TIMEOUT         (LCD_STAT_TIME+5) // s
#define LCD_STAT_TIME       5 // s
#define STAT_DELAY			 		500 // ms


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

void _P_ShutdownMode() {} // просто отметка в редакторе

//******************************************************************************************
typedef enum : char { POWEROFF = 0,  
											ABORT = -127,
											ERR_OVERFLOW,
											ERR_WATHCDOG,
											
											ERR_NO_FLOW,
											ERR_TANK_TIMEOUT,
											
											ERR_START_ABORTED_PUMP,
											ERR_INVALID_SENSORS,
											ERR_INV_LOWMARK_STATUS,
											ERR_INV_HIGHMARK_STATUS,
											ERR_HIGHMARK_UNATTENDED_RELEASE,
											ERR_LAST
											
										} t_ShutdownMode;
						
// ErrorNames[t_ShutdownMode-ABORT]			
																	// 1234567890123456
const  char ErrorMsg_1[] PROGMEM 	= "FATAL ABORT!";
const  char ErrorMsg_2[] PROGMEM 	= "OVRFLW DETECTED!";
const  char ErrorMsg_3[] PROGMEM 	= "WATCHDOG ALERT!";
const  char ErrorMsg_4[] PROGMEM 	= "NO FLOW! Chk WTR";
const  char ErrorMsg_5[] PROGMEM 	= "Tank Timout!";
const  char ErrorMsg_6[] PROGMEM 	= "Start failed!!!";
const  char ErrorMsg_7[] PROGMEM 	= "INVALID sensors!";
const  char ErrorMsg_8[] PROGMEM 	= "LM FTL! Chk SENS";
const  char ErrorMsg_9[] PROGMEM 	= "HM FTL! Chk SENS";
const  char ErrorMsg_10[] PROGMEM = "HM ERR! Chk SENS";

				
const  char* const  ErrorMsg[] PROGMEM = {
										ErrorMsg_1, ErrorMsg_2, ErrorMsg_3, ErrorMsg_4, ErrorMsg_5,
										ErrorMsg_6, ErrorMsg_7, ErrorMsg_8, ErrorMsg_9, ErrorMsg_10
};


void Shutdown(t_ShutdownMode mode);
void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, const char* s1 ); //, const char*  s2,


void log_internal_state();
							


//******************************************************************************************
typedef enum : byte { PUMP_OFF = 0,  PUMP_ABORT = 1, PUMP_ON = 2, PUMP_STANBDY = 3} t_PumpStatus;

t_PumpStatus pump 		= PUMP_OFF;

// statistic
struct t_flash_var_pump {
				int 		cnt;
				int     owfl;
				long 		total_time;
				long    total_volume;
								};
								
t_flash_var_pump	fv_pump = {0,0,0,0};
byte 		fv_errors[6];

int 		pump_last_time        = 0;
int     pump_last_volume      = 0;


//******************************************************************************************
//#define LOG(func, s) { Log.func( s ); lcd.print( s ); }
//#define LOG1(func, s, a1) { Log.func( s, a1 ); lcd.print( a1 ); }
//#define LOG2(func, s, a1, a2) { Log.func( s, a1, a2 ); lcd.print( a1 ); lcd.print( a2 ); }
//#define LOG3(func, s, a1, a2, a3) { Log.func( s, a1, a2, a3 ); lcd.print( a1 ); lcd.print( a2 ); lcd.print( a3 );  }


inline char sw(bool f, char c ='F') 
{
  return f ? c : '_';
}

#define FLASH_SIGNATURE	0xADB0
// #define _wflash(fv, l, e_offset)  for(char i=0, *s = (char*)&fv; i < sizeof(fv); EEPROM[e_offset++] = s[i++])
// #define _rflash(fv, l, e_offset)  for(char i=0, *s = (char*)&fv; i < sizeof(fv); s[i++] = EEPROM[e_offset++])


void write_flash(uint16_t sign = FLASH_SIGNATURE);
void read_flash()
{
	int e_offset = 0;
	uint16_t sign;
	
	EEPROM.get( e_offset, sign );
	Log.notice( STR_D_CR, ("read_flash") , sign);
	e_offset += sizeof(sign);
	if ( sign == FLASH_SIGNATURE )
	{
		EEPROM.get( e_offset, fv_pump );
		e_offset += sizeof(fv_pump);
		EEPROM.get( e_offset, fv_errors );
	}
	else 
		write_flash();
}

	
#define EEPROM_update(e_offset, v)		EEPROM.update( e_offset, v ); e_offset += sizeof(v);

void write_flash(uint16_t sign)// = FLASH_SIGNATURE)
{
	int e_offset = 0;
	
	Log.notice( STR_D_CR, ("write_flash"), sign);;
	
	EEPROM.update( e_offset, sign );
	e_offset += sizeof(sign);
	
	// EEPROM.update( e_offset, fv_pump );
	// e_offset += sizeof(fv_pump);
	
	EEPROM_update( e_offset, fv_pump.cnt );
	EEPROM_update( e_offset, fv_pump.owfl );
	EEPROM_update( e_offset, fv_pump.total_time );
	EEPROM_update( e_offset, fv_pump.total_volume );
	
	for(byte i=0; i<sizeof(fv_errors); ) 
		EEPROM.update( e_offset++, fv_errors[i++] );
	
	delay(5); // чтобы точно записалось
}


void lcd_on(bool timeout = false)
{
  lcd.setBacklight(255);
  lcd.clear();

// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
	Log.trace(FF("lcd_on %d restart = %c" CR), __LINE__, timeout?'R':'-' );
// #endif
	
  if (timeout)
    LCD_Timeout.restart();
	else
		LCD_Timeout.stop();

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


char status_msg[]="INV\0OFF\0ABR\0ON!\0INV";

void lcd_status()
{
  char *s, c1, c2, c3;
  char s_time[5];
  char s_vol[5];
	char buf[LCD_WIDTH+1];

	// update stat
	pump_last_time = TankTimeout.elapsed();
  pump_last_volume = SPEED_LPS * pump_last_time;
	
  lcd.setCursor(0, 0);
  switch (pump)
  {
    case PUMP_OFF:
		case PUMP_STANBDY:
      s = status_msg+4;
      break;
    case PUMP_ABORT:
      s = status_msg+8;  // Abort
      break;
    case PUMP_ON:
      s = status_msg+12;
      break;
    default:
      s = status_msg;  // INVALID
  }
	
	sprintf(buf, "%s%c%c%c %s %04u", 
					s, 
					c1 = sw(sw_Overflow.pressed(), 'O'),
					c2 = sw(sw_HighMark.pressed(), 'F'),
					c3 = sw( sw_LowMark.pressed(), 'E'),
					sprintTime4(s_time, pump_last_time),
					pump_last_volume
					);
	lcd.print(buf);
	Log.notice( STR_D_CR, buf, __LINE__);
	
  // lcd.print( s );
  // lcd.print( c1 = sw(sw_Overflow.pressed(), 'O') );
  // lcd.print( c2 = sw(sw_HighMark.pressed(), 'F') );
  // lcd.print( c3 = sw( sw_LowMark.pressed(), 'E') );
  // lcd.print( ' ' );

  // lcd.print(sprintTime4(s_time, pump_last_time));
  // lcd.print(' ');
  // sprintf(s_vol, "%04u", pump_last_volume);
  // lcd.print(s_vol);

  // Log.notice("%s %c%c%c %s %s" CR, s, c1, c2, c3, s_time, s_vol);

  lcd.setCursor(0, 1);
}

void sprintErrors(char* buf)
{
		// char buf[sizeof(fv_errors)*2+2];
		
		for(char i=0, *s=buf; i<sizeof(fv_errors); s+=sprintf(s, "%02X", fv_errors[i++]));
			
		Log.notice( STR_D_CR, buf, __LINE__);
		
}

void lcd_statistic(uint8_t mode=0)
{
												 //12    34567890123456
	char buf[LCD_WIDTH+1] = "T ";//00000 V 000000";
	char buf_err[sizeof(fv_errors)*2+2];
	
  switch (mode)
  {
    case 0:
    default:  // пока так
			
      lcd.setCursor(0, 0);
			sprintTime5(buf+2, fv_pump.total_time);
			sprintf(buf+7, " V %06d", fv_pump.total_volume);
      lcd.print(buf);
			Log.notice( STR_D_CR, buf, __LINE__);

      lcd.setCursor(0, 1);
			sprintErrors(buf_err);
			sprintf( buf, "%03dE%s", fv_pump.cnt, buf_err); //fv_pump.owfl);
			lcd.print(buf);
 			Log.notice( STR_D_CR, buf, __LINE__);
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

  pump = PUMP_OFF;
	
	lcd_on(true);
  lcd_status();
	
  // stat
	Log.notice( STR_D_CR, ("PumpOff"), __LINE__);
	lcd_statistic();
	delay(5000);
	
  fv_pump.total_time += pump_last_time;
  fv_pump.total_volume += pump_last_volume;
	write_flash();
	
	// Log.notice( STR_D_CR, "PumpOff", __LINE__);
	
	lcd_statistic();
	delay(5000);
	
  Log.notice( FF( "done: %i seconds" CR), pump_last_time );

  FlowTimeout.stop();
  TankTimeout.stop();
  
}


// включает двигатель, если все нормально и запускает таймеры переполнения и нижнего датчика
void PumpOn()
{

  if (pump == PUMP_ABORT)
  {
    Shutdown(
      //  1234567890123456
      // FF("Start failed!!!" ),
      ERR_START_ABORTED_PUMP);
    return;
  }

  Log.notice( FF("Pump ON - " ) );

  // stat
  fv_pump.cnt++;

  digitalWrite(SW_VALVE, HIGH);
  digitalWrite(SW_MOTOR, HIGH);

  Log.notice( FF( "done" CR) );

  FlowTimeout.restart();
  TankTimeout.restart();
  StatDelay.restart();
	
  pump = PUMP_ON;

	lcd_on();
  lcd_status();
}


// уснуть - до следующего поплавка (POWEROFF) или навсегда
void _shutdown(t_ShutdownMode mode = POWEROFF)
{

#ifdef DEBUG
  return;
#endif

	// log_internal_state(-1);

  if (mode == POWEROFF)
  {
		// выключить свет и все
    lcd.setBacklight(0);
  }
  else {
    // FULL Shutdown !!!
    pump = PUMP_ABORT;

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
    Log.verbose(FF("shutdown Abort %d" CR), __LINE__);
#endif

    // чтобы байтики через serial прошли
    delay(100);

    // не просыпаться никогда
    // сделать блокировку до полного отключения
		PCICR = 0;
    PCMSK2 = 0;
		
		// Log.verbose(FF("powerDown %d" CR), __LINE__);
		// delay(100);
		
		// LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    // Log.verbose(FF("wakeup %d" CR), __LINE__);
  }

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.verbose(FF("powerDown %d" CR), __LINE__);
#endif

  delay(100);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif
}

										
// печатает текст и останавливает устройство
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL)

void Shutdown(t_ShutdownMode mode)
{
	char buf[20];
	
	if (mode != POWEROFF && mode > ERR_LAST) mode = ABORT;
	if (mode != POWEROFF)
	{
		byte a;
		byte shift = mode - ABORT;
		byte &err = fv_errors[shift>>1];
		Log.notice( FF("shift %d" CR), shift);
		
		if (shift & 1) // байты задом наперед
			err+=((err&0x0f)<0x0f)?0x01:0;
		else
			err+=((err&0xf0)<0xf0)?0x10:0;
		
		pump = PUMP_ABORT;
		
		sprintErrors(buf);
		Log.notice( FF("Shutdown mode %d %d %x %d %s (%d)" CR), mode, shift, err, sizeof(fv_errors), buf, __LINE__);

	}
	
	// if (mode != POWEROFF)
	// {
		// pump = PUMP_ABORT;
	// }
	
	PumpOff();
	
  // lcd_on(pump != PUMP_ABORT);
	lcd_on();
  lcd_status();
	
	strcpy_P(buf, (char*)pgm_read_word(&(ErrorMsg[mode-ABORT])));
	
  lcd.print( buf );
  Log.fatal( STR_D_CR, buf, __LINE__);

#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;

  _shutdown(mode);
}

/* 
void A_Watchdog()
{
  wdt_disable();
  //f_WDT = 1;
  SETFLAG(f_WDT);
  //PumpOff();
}
 */
 
//***********************************************************************************************************************************************************

void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF(CR "LowMark: %i %x" CR), f_LowMark, event);
// #endif	
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
          // FF("INVALID sensors!"),
          ERR_INVALID_SENSORS);
      }
      break;
    case EVENT_RELEASED:
      FlowTimeout.stop();
      break;
    default:
      Shutdown(
        //  1234567890123456
        // FF("LM FTL! Chk SENS" ),
        ERR_INV_LOWMARK_STATUS);
  }

}

void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF(CR "HighMark: %i %x" CR), f_HighMark, event);
// #endif 
	//f_HighMark = 0;

  //  must be pressed before
  switch (event)
  {
    case EVENT_PRESSED:
      // вода начала кончаться
			// игнорим
			lcd_on(true);
			lcd_status();
			lcd.print("waiting...."); 
			pump = PUMP_STANBDY; // типа выключить экран, но текст не обновлять
      break;
    case EVENT_RELEASED:
      if (pump != PUMP_ON)
      {
        Shutdown(
          //  1234567890123456
          // FF("HM ERR! Chk SENS"),
          ERR_HIGHMARK_UNATTENDED_RELEASE);
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
        // FF("HM FTL! Chk SENS"),
        ERR_INV_HIGHMARK_STATUS);
  }
}

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
// #if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace(FF(CR "OVERFLOW: %i %x" CR), f_Overflow, event);
// #endif

  fv_pump.owfl++;

  // any change is dangerous!!!
  Shutdown(
    //  1234567890123456
    // FF("OVRFLW DETECTED!" ),
    ERR_OVERFLOW);
}

//***********************************************************************************************************************************************************

volatile uint8_t portBhistory = 0x00;     // default is high because the pull-up
volatile uint8_t portChistory = 0x00;
volatile uint8_t portDhistory = 0x00;     // не pool-up
volatile long t = 0;

ISR(PCINT2_vect)
{
  // t = millis();
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
/* 
ISR(PCINT0_vect)
{
  t = millis();
  uint8_t changedbits;

  // PCIMSK0
  changedbits = PINB ^ portBhistory;
  portBhistory = PINB;

   // if (changedbits & (1 << PIN_OVERFLOW))
   // {
     // SETFLAG(f_Overflow);
   // }
  
   // if (changedbits & (1 << PIN_HIGHMARK))
   // {
     // SETFLAG(f_HighMark);
   // }
  
   // if (changedbits & (1 << PIN_LOWMARK))
   // {
     // SETFLAG(f_LowMark);
   // }

}
 */
 
 /* 
ISR(INT1_vect)
{

}
 */

#ifdef WatchDog_h
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
#endif

void log_internal_state(int l_cnt)
{
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
    unsigned long e1 =   FlowTimeout.elapsed();
    unsigned long e2 =   TankTimeout.elapsed();
		unsigned long e3 =   LCD_Timeout.elapsed();
		unsigned long e4 =   StatDelay.elapsed();
						
    Log.trace( FF("l_cnt = %d, t = %l f_WDT %i, f_Overflow %i, f_LowMark %i, f_HighMark %i, " CR
									"        portBhistory %X, portChistory %X, portDhistory %X, " CR
									"        FlowTimeout = %c %l, TankTimeout = %c %l, LCD_Timeout = %c %l, StatDelay = %c %l" CR),
               l_cnt, t, f_WDT, f_Overflow, f_LowMark, f_HighMark,
               portBhistory, portChistory, portDhistory, 
							 sw( FlowTimeout.isRunning(), 'R') , e1, sw( TankTimeout.isRunning(), 'R'), e2, 
							 sw( LCD_Timeout.isRunning(), 'R'), e3, sw( StatDelay.isRunning(), 'R'), e4
		);
#endif
}


//***********************************************************************************************************************************************************


void setup() 
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println(FF(CR "In the begining...") );

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd_on(true);
  lcd.print(FF("Starting..."));
  //  lcd.setCursor(0,1);
  //  lcd.print("Starting...");

	STR_D_CR = FF("%s (line:%d)" CR);
  Log.begin(LOG_LEVEL, &Serial);
  Log.notice( FF(CR "******************************************" CR) );                     // Info string with Newline

#ifdef WatchDog_h
	Log.notice( FF("WatchDog::init(A_Watchdog, OVF_8000MS, STOP);" CR) );
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif
	
	Log.notice( STR_D_CR, ("test setup"), __LINE__);
	for (char i=0; i<sizeof(fv_errors); fv_errors[i++]=i);
	lcd_statistic();
	delay(5000);
	
	// write_flash(0); // только один раз для платы
	read_flash();
	lcd_statistic();
	Log.notice( STR_D_CR, ("test setup"), __LINE__);

  pinMode(SW_MOTOR, OUTPUT);
  pinMode(SW_VALVE, OUTPUT);
	// PumpOn();
	// delay(500);
  PumpOff();

  lcd_status();
  lcd.print( FF("READY!!!") );
	StatDelay.start();

	PORTD |= (1 << PORTD5) | (1 << PORTD6) | (1 << PORTD6);
  
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
  if ( (pump == PUMP_OFF || pump == PUMP_STANBDY) && 
			!LCD_Timeout.isRunning() &&
			!StatDelay.isRunning() &&
			!TankTimeout.isRunning() &&
			!FlowTimeout.isRunning()
		  )
  {
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
    Log.verbose(FF("Sleep %d" CR), __LINE__);
#endif		
		delay(10);
		LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
    Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif		
  }
#else
	delay(100);
#endif

	l_cnt++;

  if ( StatDelay.hasPassed(STAT_DELAY, true) )
  {
		log_internal_state(l_cnt);
					
		if (pump == PUMP_ON) 
		{
			lcd_status();
			// дописать текущий тайминг
		}
  }


  if (0 & f_WDT) 
  {
    Shutdown(
      //  1234567890123456
      // FF("WATCHDOG ALERT!" ),
      ERR_WATHCDOG);
  }



	if (LCD_Timeout.hasPassed(LCD_TIMEOUT)) 
	{
		if (pump == PUMP_OFF)
		{
			// lcd_off:
			lcd.clear();
			lcd_status();
			lcd.print("READY! cnt="); lcd.print(fv_pump.cnt);
			pump = PUMP_STANBDY;		
		}
		
		LCD_Timeout.stop();
		LCD_Timeout.add( -LCD_Timeout.elapsed() );
		lcd.setBacklight(0);
			
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
		Log.verbose(FF("Shutdown %d" CR), __LINE__);
#endif	
		_shutdown();
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
		Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif
	}
	else if ((pump == PUMP_OFF) && LCD_Timeout.hasPassed(LCD_STAT_TIME))
	{
		lcd_statistic();
	}


  if (FlowTimeout.hasPassed(LOWMARK_TIMEOUT)) 
  {
    Shutdown(
      //  1234567890123456
      // FF("NO FLOW! Chk WTR" ),
      ERR_NO_FLOW);
  }

  if (TankTimeout.hasPassed(HIGHMARK_TIMEOUT)) 
  {
    Shutdown(
      //  1234567890123456
      // FF("Tank Timout!" ),
      ERR_TANK_TIMEOUT);
  }

  // put your main code here, to run repeatedly:

  sw_Overflow.loop();
  sw_HighMark.loop();
  sw_LowMark.loop();

  if (sw_LowMark.pressed() && !sw_HighMark.pressed()) 
  {
    Shutdown(
      //  1234567890123456
      // FF("Sensors error!" ),
      ERR_INVALID_SENSORS);
  }

}
