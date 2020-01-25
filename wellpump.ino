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

// #define DEBUG_WAKEUPS
#define DISABLE_LOGGING

#include <limits.h>
#include <WString.h>
// #include <avr/wdt.h>
#include <LowPower.h>               //  https://github.com/rocketscream/Low-Power
#include <EEPROM.h>
#include <ArduinoLog.h>             // https://github.com/thijse/Arduino-Log/
#include <avr/pgmspace.h>
#include <DebounceEvent.h>          // https://github.com/xoseperez/debounceevent
#include "Chrono.h"
#include <LiquidCrystal_PCF8574.h>
//#include <LiquidCrystal_I2C_OLED.h>


void _P_defines() {} // просто отметка в редакторе

// * 0 - LOG_LEVEL_SILENT     no output 
// * 1 - LOG_LEVEL_FATAL      fatal errors 
// * 2 - LOG_LEVEL_ERROR      all errors  
// * 3 - LOG_LEVEL_WARNING    errors, and warnings 
// * 4 - LOG_LEVEL_NOTICE     errors, warnings and notices 
// * 5 - LOG_LEVEL_TRACE      errors, warnings, notices & traces 
// * 6 - LOG_LEVEL_VERBOSE    all 
#define LOG_LEVEL			LOG_LEVEL_TRACE

#define SETFLAG(x) ( ++(x)?(x):(++(x)) )  // если увеличение дало 0, то увеличиваем еще раз

typedef unsigned long	t_time;
/* для работы со сторками из ПЗУ */
typedef __FlashStringHelper* fchar;
typedef const __FlashStringHelper* fchar_;
//const char LOG_AS[] PROGMEM =

//#define FF(a) 			(a)
#define FF(a)       F(a)
// #define FPSTR(a)		(a)

// https://arduino-esp8266.readthedocs.io/en/latest/PROGMEM.html 
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))


/* настройки экранчика */
#define	LCD_PORT 						0x27
#define LCD_WIDTH						16
LiquidCrystal_PCF8574 			lcd(LCD_PORT);


//*******************************************************************************************

#define PIN_OVERFLOW        PD5
#define PIN_HIGHMARK        PD6
#define PIN_LOWMARK         PD7
#define PIN_STATUS					PB2

// sensors' pins
#define SNS_OVERFLOW				5
#define SNS_HIGH_MARK				6
#define SNS_LOW_MARK				7
#define SNS_STATUS					10

// switches' pins
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
 
#define LOWMARK_TIMEOUT     10 // s
#define HIGHMARK_TIMEOUT    427 // s = 0,468л/с 200 литров
#define SPEED_LPS           0.468 // liters per second
#define LCD_TIMEOUT         (LCD_STAT_TIME+10) // s
#define LCD_STAT_TIME       7 // s
#define STAT_DELAY			 		250 // ms
#define RESET_TIMEOUT				8000 // ms


//******************************************************************************************
//  EVENT_NONE EVENT_CHANGED EVENT_PRESSED EVENT_RELEASED
//  BUTTON_PUSHBUTTON | BUTTON_DEFAULT_HIGH | BUTTON_SET_PULLUP);
//  DebounceEvent button = DebounceEvent(BUTTON_PIN, BUTTON_SWITCH | BUTTON_SET_PULLUP);

uint16_t loop_cnt = 0;
uint8_t  wakeupmode = 0;

#define CNT_DIV 3
byte v_StatMode = 0;


volatile uint8_t f_Overflow	= 0;
volatile uint8_t f_LowMark	= 0;
volatile uint8_t f_HighMark	= 0;
volatile uint8_t f_Status		= 0;

void btn_Overflow(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_LowMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_HighMark(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);
void btn_Status(uint8_t pin, uint8_t event, uint8_t count, uint16_t length);

DebounceEvent sw_Overflow		= DebounceEvent(SNS_OVERFLOW,	btn_Overflow,	BUTTON_PUSHBUTTON );
DebounceEvent sw_LowMark		= DebounceEvent(SNS_LOW_MARK,	btn_LowMark,	BUTTON_PUSHBUTTON );
DebounceEvent sw_HighMark		= DebounceEvent(SNS_HIGH_MARK,	btn_HighMark,	BUTTON_PUSHBUTTON );
DebounceEvent sw_Stat				= DebounceEvent(SNS_STATUS,		btn_Status,		BUTTON_PUSHBUTTON );

Chrono TankTimeout(Chrono::SECONDS);
Chrono FlowTimeout(Chrono::SECONDS);
Chrono LCD_Timeout(Chrono::SECONDS);
Chrono StatDelay(Chrono::MILLIS);


volatile uint8_t f_WDT = 0;

void _P_ShutdownMode() {} // просто отметка в редакторе

//******************************************************************************************
typedef enum : char { POWEROFF = 0,
											STANDBY,
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
											ERR_FLASH_RESET,
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
const  char ErrorMsg_11[] PROGMEM = "FLASH ERASED!"; //"Press RESET btn!";
const  char ErrorMsg_0[] PROGMEM  = "READY! cnt=";

				
const  char* const  ErrorMsg[] PROGMEM = {
										ErrorMsg_0,
										ErrorMsg_1, ErrorMsg_2, ErrorMsg_3, ErrorMsg_4, ErrorMsg_5,
										ErrorMsg_6, ErrorMsg_7, ErrorMsg_8, ErrorMsg_9, ErrorMsg_10, 
										ErrorMsg_11
};


//******************************************************************************************
typedef enum : byte { PUMP_OFF = 0,  PUMP_ABORT = 1, PUMP_ON = 2, PUMP_STANBDY = 3} t_PumpStatus;

t_PumpStatus pump 		= PUMP_OFF;

// statistic
struct t_flash_var_pump {
				uint16_t				sign;
				unsigned int 		pwr;
				unsigned int 		cnt;
				unsigned int 		owfl;
				t_time				 	total_time;
				unsigned long		total_volume;
				byte						errors[ERR_LAST-ABORT];
			};
								
t_flash_var_pump				fv_pump 							= {0, 0, 0, 0, 0, 0};
// t_flash_var_pump				fv_saved;


unsigned 								pump_last_time        = 0;
unsigned								pump_last_volume      = 0;

const  char PROGMEM	STR_sd_CR[]		= ("%s (line:%d)" CR);
const  char PROGMEM	STR_SXd_CR[]	= ("%S %X (line:%d)" CR);

char 										buf[LCD_WIDTH+1];
char										buf2[LCD_WIDTH+2];

//******************************************************************************************

void Shutdown(t_ShutdownMode mode);
void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode, fchar s1); //,fchar s2,);
// void Shutdown(t_ShutdownMode mode = POWEROFF, const char* s1 ); //, const char*  s2,

/* void log_internal_state(); */

//******************************************************************************************

inline char sw(bool f, char c ='F') 
{
  return f ? c : '_';
}

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

#define FLASH_SIGNATURE	0xADB0
#define FLASH_OFFSET		0x20*6


void print_flash(int l)
{
	sprintErrors(buf);
	Log.warning(FF("flash(%d): %x %x %x %x err=|%s|" CR), l,
		(int)fv_pump.cnt, (int)fv_pump.owfl, (int)fv_pump.total_time, (int)fv_pump.total_volume, buf);
		// fv_pump.errors[0], fv_pump.errors[1], fv_pump.errors[2], fv_pump.errors[3], fv_pump.errors[4], fv_pump.errors[5],
		// fv_pump.errors[6], fv_pump.errors[7], fv_pump.errors[8], fv_pump.errors[9]);
	
}

#define EEPROM_f(f, e_offset, v)		{ EEPROM.f( e_offset, v ); e_offset += sizeof(v); }

void write_flash(uint16_t sign = FLASH_SIGNATURE);
bool read_flash()
{
	int e_offset = FLASH_OFFSET;
	char *e_buf = (char*)&fv_pump;
		
	EEPROM_f( get, e_offset, e_buf[0] );
	EEPROM_f( get, e_offset, e_buf[1] );
	// Log.notice(STR_SXd_CR, FF("read_flash: sign = "), fv_pump.sign, __LINE__);
	// Serial.println( fv_pump.sign );
	// Serial.println( e_offset );
	
	if ( fv_pump.sign == FLASH_SIGNATURE )
	{
		for (size_t i=2; i<sizeof(fv_pump); i++)
				EEPROM_f( get, e_offset, e_buf[i] );
		
		Log.notice(FF("read_flash done %d %d" CR), FLASH_OFFSET, e_offset);
		
		print_flash(__LINE__);
		return true;
	}
	else 
	{
		// Log.notice(STR_SXd_CR, FF(""), 0, __LINE__);
		for (size_t i=0; i<sizeof(fv_pump.errors); fv_pump.errors[i++]=0);
		print_flash(__LINE__);
		write_flash();
		return false;
	}
	
}

#define EEPROM_W_FUNC	update
void write_flash(uint16_t sign)
{
	int e_offset = FLASH_OFFSET;
	// t_EEPROM_sign &s = (t_EEPROM_sign&)sign;
	char *e_buf = (char*)&fv_pump;
	
	// Serial.println( sign );
	// Log.warning(STR_SXd_CR, FF("write flash: free mem = "), freeRam(), __LINE__);
	
	fv_pump.sign = sign;
	for (size_t i=0; i<sizeof(fv_pump); i++)
				EEPROM_f( EEPROM_W_FUNC, e_offset, e_buf[i] );
 
	Log.notice(FF("write_flash done %d %d" CR), FLASH_OFFSET, e_offset);
	print_flash(__LINE__);
	
}

void reset_flash()
{
		for (size_t i=0; i<sizeof(fv_pump); i++)
				EEPROM.write ( FLASH_OFFSET + i, (byte)0 );
}



void lcd_on(bool timeout = false)
{
  lcd.setBacklight(255);
  lcd.clear();

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
	Log.trace(FF("lcd_on %d restart = %c" CR), __LINE__, timeout?'R':'-' );
#endif
	
  if (timeout)
    LCD_Timeout.restart();
	else
		LCD_Timeout.stop();

}


char* sprintTime4(char* s, unsigned long v)
{
  if (v < 900)
    sprintf(s, ("%03us"), (unsigned)  v);            	// 000s
  else if (v < 6000)
    sprintf(s, ("%03um"), (unsigned) (v/60) );				// 000m
/*   else if (v < 600)
    sprintf(s, ("%c:%03um"), (unsigned) (v/60)+'0', (unsigned) (v%60) );         // 0:00	*/
 else if (v < 36000 )
	{
		v /= 360;
    sprintf(s, ("%u.%ch"), (unsigned) v/10, (char) (v%10) + '0' );		// 0.0h
	}
  else
    sprintf(s, ("%03uh"), (unsigned) (v/3600) );			// 000h

  return s;
}


char* sprintTime5(char* s, unsigned long v)
{
	/* 
// Скетч использует 13770 байт (96%) памяти устройства. Всего доступно 14336 байт.
// Глобальные переменные используют 797 байт (77%) динамической памяти, оставляя 227 байт для локальных переменных. Максимум: 1024 байт.
	if (v < 600)
    sprintf(s, ("%04us"),  (unsigned) v);       // 0000s
  else if ( (v/=6) < 900 )  // < 6000
    sprintf(s, ("%02u.%cm"),  (unsigned) v/10,  (char) (v%10) + '0'); //00.1m
  else if ( (v/=6) < 1000 )	
		sprintf(s, ("%u.%02dh"),  (unsigned) v/100, (unsigned) v%100 );// 0.00h
  else if ( (v/=10) < 1000 )
    sprintf(s, ("%02u.%ch"), 	(unsigned) v/10,  (char) (v%10) + '0' );// 00.0h
  else
    sprintf(s, ("%04uh"),  		(unsigned) v/10 );    // 0000h
 */

// Скетч использует 13756 байт (95%) памяти устройства. Всего доступно 14336 байт.
// Глобальные переменные используют 797 байт (77%) динамической памяти, оставляя 227 байт для локальных переменных. Максимум: 1024 байт.
  if (v < 900)
    sprintf(s, ("%04us"),  (unsigned) v);       // 0000s
  /* else if ( v < 5400 )  //   900 сек
	{
		v /= 6;
    sprintf(s, ("%02u.%cm"),  (unsigned) v/10,  (char) (v%10) + '0'); //00.1m
  }
	else if ( v < 36000 )		// 10 часов
	{
		v /= 36;
		sprintf(s, ("%u.%02dh"),  (unsigned) v/100, (unsigned) v%100 );// 0.00h
  } */
	else 
	if ( v < 216000 )  // 60*60*60
  {
		v /= 60;
		sprintf(s, ("%02u:%02uh"), 	(unsigned) v/60,  (unsigned) (v%60) );// 00.0h
  }
	else if ( v < 360000 )   // 100 часов 
  {
		v /= 360;
		sprintf(s, ("%02u.%ch"), 	(unsigned) v/10,  (char) (v%10) + '0' );// 00.0h
  }	

	else
  {
		sprintf(s, ("%04uh"),  		(unsigned) v/3600 );       // 0000h
	} 

	
  return s;
}

void lcd_print(char buf[], unsigned int l)
{
	lcd.print(buf);
	Log.notice( FPSTR(STR_sd_CR), buf, l);
}

void lcd_print_P(const char* buf, unsigned int l)
{
	lcd.print( FPSTR(buf) );
	Log.notice( FPSTR(STR_SXd_CR), FPSTR(buf), 0, l);
}


char status_msg[]="INV\0OFF\0ABR\0ON!\0INV";

void lcd_status()
{
  char *s, c1, c2, c3;
  char s_time[5];
  // char s_vol[5];
	// char buf[LCD_WIDTH+1];

	// update stat
	if ( TankTimeout.isRunning() )
	{
		pump_last_time = TankTimeout.elapsed();
		pump_last_volume = SPEED_LPS * pump_last_time;
	}
		
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
				#ifdef DEBUG_WAKEUPS
					loop_cnt
				#else
					pump_last_volume
				#endif
					);
			
	#ifdef DEBUG_WAKEUPS			
	buf[6] = 'A'-1 + wakeupmode; 	// отладка
	#endif
	
	lcd_print(buf, __LINE__);
	// Log.notice( STR_sd_CR, buf, );
	
  lcd.setCursor(0, 1);
}


int sprintErrors(char* buf)
{		
		char *s = buf;
		for(char i=0; i<sizeof(fv_pump.errors); s+=sprintf(s, "%X", fv_pump.errors[i++]));
		for(char* t = buf; *t; t++) 
			if (*t=='0') *t=' ';
		
		return s-buf;
}

/* 
before:
Скетч использует 10844 байт (75%) памяти устройства. Всего доступно 14336 байт.
Глобальные переменные используют 765 байт (74%) динамической памяти, оставляя 259 байт для локальных переменных. Максимум: 1024 байт.

after:
Скетч использует 11468 байт (79%) памяти устройства. Всего доступно 14336 байт.
Глобальные переменные используют 780 байт (76%) динамической памяти, оставляя 244 байт для локальных переменных. Максимум: 1024 байт.
 */
 
void lcd_statistic(uint8_t mode=0)
{
														//12    34567890123456
	// char buf[LCD_WIDTH+5] = "T ";//00000 V 000000";
	int sh;
	size_t i;
	char* b1;
	char* b2;	
	uint16_t p1;
	uint16_t p2;

  lcd.setCursor(0, 0);	
	
  switch (mode)
  {
		// case 8:
		default:  // пока так
			mode = v_StatMode = 0;
    case 0:
			*buf = 'T';
			*(buf+1)=' ';
			sprintTime5(buf+2, fv_pump.total_time);
	#ifdef DEBUG_WAKEUPS
			sprintf(buf+7, " V %06d", loop_cnt);
	#else
			sprintf(buf+7, " V %06d", fv_pump.total_volume);
	#endif					
			sh = sprintf( buf2, "%03dE", fv_pump.cnt);
			// залить пробелами строку до конца экрана
			for( i=sh+sprintErrors( buf2 + sh ); i<=LCD_WIDTH; buf2[i++]=' '); //fv_pump.owfl);
			buf2[LCD_WIDTH] = 0;
			
			// lcd_print(buf, __LINE__);
			// Log.notice( FF("stat (%d): sh %i buf %s" CR), __LINE__, sh, buf);
			break;
		case 1:
			b1 = PSTR("TTL time: ");
			b2 = PSTR("TTL volume: ");
			p1 = fv_pump.total_time;
			p2 = fv_pump.total_volume;
			// total time
			// total liters
			break;
		case 2:
			b1 = PSTR("Power ups: ");
			b2 = PSTR("Pumps ON:  ");
			p1 = fv_pump.pwr;
			p2 = fv_pump.cnt;
			// power 
			// fills	
			break;
		case 3:
			b1 = PSTR("Overflow:  ");
			b2 = PSTR("Fatal ERR: ");
			p1 = fv_pump.owfl;
			p2 = fv_pump.errors[ABORT-ABORT];
			// overflow
			// fatal
			break;
		case 4:
			b1 = PSTR("No FLOW:   ");
			b2 = PSTR("Tank ABR:  ");
			p1 = fv_pump.errors[ABORT-ERR_NO_FLOW];
			p2 = fv_pump.errors[ABORT-ERR_TANK_TIMEOUT];
			// NO FLOW
			// Tank Timout
			break;
		case 5:	
			b2 = PSTR("WATCHDOG:  ");
			b1 = PSTR("Start ABR: ");
			p2 = fv_pump.errors[ABORT-ERR_WATHCDOG];
			p1 = fv_pump.errors[ABORT-ERR_START_ABORTED_PUMP];
			// WATCHDOG	
			// Start failed
			break;
		case 6:	
			b1 = PSTR("INV sens:  ");
			b2 = PSTR("HM&LM err: ");
			p1 = fv_pump.errors[ABORT-ERR_INVALID_SENSORS];
			p2 = fv_pump.errors[ABORT-ERR_HIGHMARK_UNATTENDED_RELEASE];
			// INVALID sensors!";
			// HM ERR
			break;
		case 7:	
			b1 = PSTR("LM errors: ");
			b2 = PSTR("HM errors: ");
			p1 = fv_pump.errors[ABORT-ERR_INV_LOWMARK_STATUS];
			p2 = fv_pump.errors[ABORT-ERR_INV_HIGHMARK_STATUS];
// const  char ErrorMsg_8[] PROGMEM 	= "LM FTL! Chk SENS";
// const  char ErrorMsg_9[] PROGMEM 	= "HM FTL! Chk SENS";
			break;
  }
      
	if ( mode > 0 )
	{
		size_t l;
		l = strlcpy_P(buf, b1, sizeof(buf));
		if ( mode == 1 )
			sprintTime5(buf + l, p1);
		else	
			sprintf(buf + l, "%d", p1);
		
		l = strlcpy_P(buf2, b2, sizeof(buf2));
		sprintf(buf2 + l, "%d", p2);
	}
	
	lcd.setCursor(0, 0);		
	lcd_print(buf, __LINE__+mode*1000);
  lcd.setCursor(0, 1);
	lcd_print(buf2, __LINE__+mode*1000);
	
  //lcd_on(true);
}

//******************************************************************************************

void PumpOff()
{
  Log.notice( FF("Pump OFF - done: %i seconds" CR), pump_last_time );

  //noInterrupts();
  digitalWrite(SW_VALVE, LOW);
  digitalWrite(SW_MOTOR, LOW);
  //interrupts();

  pump = PUMP_OFF;
	
	lcd_on(true);
  lcd_status();
	
  // stat
		
  fv_pump.total_time += pump_last_time;
  fv_pump.total_volume += pump_last_volume;
	// write_flash();

  // Log.notice( FF( "done: %i seconds" CR), pump_last_time );
	
	// read_flash();
	

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

  Log.notice( FF("Pump ON" ) );

  // stat
  fv_pump.cnt++;

  digitalWrite(SW_VALVE, HIGH);
  digitalWrite(SW_MOTOR, HIGH);

  // Log.notice( FF( "done" CR) );

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

    // не просыпаться никогда
    // сделать блокировку до полного отключения
		PCICR = 0;
    PCMSK2 = 0;
	
  }

#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.verbose(FF("powerDown %d (%d)" CR), mode, __LINE__);
#endif

  delay(100); // чтобы байтики через serial прошли ...перед глубоким сном
	
	#ifdef DEBUG_WAKEUPS
	wakeupmode = 1;
	#endif
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
	
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif
}

										
// печатает текст и останавливает устройство
// void Shutdown(t_ShutdownMode mode = POWEROFF, fchar s1 = NULL)

void Shutdown(t_ShutdownMode mode)
{
	// char buf[20] = "empty";
	size_t	l;
	
	if ( !(mode == POWEROFF || mode == STANDBY) && mode > ERR_LAST) mode = ABORT;
	if (mode < POWEROFF)
	{
		byte &err = fv_pump.errors[mode - ABORT];
		err += (err^0x0f)?1:0;
		
		pump = PUMP_ABORT;
	}


	if (mode == STANDBY)
	{
		pump = PUMP_STANBDY;
		mode = POWEROFF;
	}
	else
	{
		PumpOff();
	}
	
	write_flash();
		
	// lcd_on(pump != PUMP_ABORT);
	v_StatMode = 0;
	lcd_on();
	lcd_status();
	
	l = strlcpy_P(buf, (char*)pgm_read_word(&(ErrorMsg[(mode==POWEROFF)?0:mode-ABORT+1])), sizeof(buf));
	if (mode == POWEROFF) 
		sprintf(buf + l, "%d", fv_pump.cnt);
	
	lcd_print( buf, __LINE__);
	// Log.fatal( STR_sd_CR, buf, __LINE__);
	
#ifdef WatchDog_h
  WatchDog::stop();
#endif
  f_WDT = 0;

  _shutdown(mode);
}

void _reset()
{
	// lcd_on();
	// lcd_print( strcpy_P(buf, FF("FLASH ERASED!") ), __LINE__);
	// lcd.setCursor(0, 1);
	// lcd_print( strcpy_P(buf, FF("Press RESET btn!") ), __LINE__);
	delay(100);
	
	Shutdown(ERR_FLASH_RESET);
	// wdt_disable();
	// wdt_enable(WDTO_8S);
	// while(1);
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
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF(CR "LowMark: %i %x" CR), f_LowMark, event);
#endif	
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
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF(CR "HighMark: %i %x" CR), f_HighMark, event);
#endif 
	//f_HighMark = 0;

  //  must be pressed before
  switch (event)
  {
    case EVENT_PRESSED:
      // вода начала кончаться
			// игнорим
			lcd_on(true);
			lcd_status();
			lcd.print(FF("waiting....")); 
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
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace(FF(CR "OVERFLOW: %i %x" CR), f_Overflow, event);
#endif

  fv_pump.owfl++;

  // any change is dangerous!!!
  Shutdown(
    //  1234567890123456
    // FF("OVRFLW DETECTED!" ),
    ERR_OVERFLOW);
}

void btn_Status(uint8_t pin, uint8_t event, uint8_t count, uint16_t length)
{
	static t_time ev_time = 0;
	// if (pump != PUMP_ON && event == EVENT_PRESSED)
	// {
		// lcd_on(true);
		// lcd_statistic(++v_StatMode);
	// }
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
  Log.trace( FF("btn_Status: %i %x" CR), v_StatMode, event);

	// Serial.print(FF("btn_Status "));
	Serial.println(ev_time);
#endif 
	
	if (pump == PUMP_ON) return;
	switch ( event ) 
	{
		case EVENT_PRESSED:
			ev_time = millis();
			lcd_on(true);
			lcd_statistic(++v_StatMode);
			break;
		case EVENT_RELEASED:
			{
				t_time ev_len, now = millis();
				
				// Serial.println(now);
				
				if ( now < ev_time ) 
					ev_len = ULONG_MAX - ev_time + now;
				else
					ev_len = now - ev_time;
				
				if ( ev_len > RESET_TIMEOUT )
				{
					// Serial.println(ev_len);
					// Serial.println(RESET_TIMEOUT);
					reset_flash();
					_reset();
				}
				
				break;
			}
	}
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

ISR(PCINT0_vect)
{
  t = millis();
  uint8_t changedbits;

  // PCIMSK0
  changedbits = PINB ^ portBhistory;
  portBhistory = PINB;

   if (changedbits & (1 << PIN_STATUS))
   {
     SETFLAG(f_Status);
   }
  
}

 
 /* 
ISR(INT1_vect)
{

}
 */

/* #ifdef WatchDog_h
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
 */
 
/* void log_internal_state(int loop_cnt)
{
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)
    unsigned long e1 =   FlowTimeout.elapsed();
    unsigned long e2 =   TankTimeout.elapsed();
		unsigned long e3 =   LCD_Timeout.elapsed();
		unsigned long e4 =   StatDelay.elapsed();
						
    Log.trace( FF("loop_cnt = %d, t = %l f_WDT %i, f_Overflow %i, f_LowMark %i, f_HighMark %i, " CR
									"        portBhistory %X, portChistory %X, portDhistory %X, " CR
									"        FlowTimeout = %c %l, TankTimeout = %c %l, LCD_Timeout = %c %l, StatDelay = %c %l" CR),
               loop_cnt, t, f_WDT, f_Overflow, f_LowMark, f_HighMark,
               portBhistory, portChistory, portDhistory, 
							 sw( FlowTimeout.isRunning(), 'R') , e1, sw( TankTimeout.isRunning(), 'R'), e2, 
							 sw( LCD_Timeout.isRunning(), 'R'), e3, sw( StatDelay.isRunning(), 'R'), e4
		);
#endif
} */


//***********************************************************************************************************************************************************


void setup() 
{
#ifndef DISABLE_LOGGING
  Serial.begin(115200);
  while (!Serial);
  // Serial.println(FF(CR "In the begining...") );
#endif

  // lcd.init();
  lcd.begin(16, 2);
  lcd.noAutoscroll();
  lcd_on(true);
  Log.begin(LOG_LEVEL, &Serial);
  // Log.notice( FF(CR "******************************************" CR) );                     // Info string with Newline

  lcd_print_P(PSTR("Starting..."), __LINE__);
  //  lcd.setCursor(0,1);
  //  lcd.print("Starting...");
	

#ifdef WatchDog_h
	Log.notice( FF("WatchDog::init(A_Watchdog, OVF_8000MS, STOP);" CR) );
  WatchDog::init(A_Watchdog, OVF_8000MS, STOP);
#endif
	
	// write_flash(0); // только один раз для платы
	read_flash();
	fv_pump.pwr++;

  pinMode(SW_MOTOR, OUTPUT);
  pinMode(SW_VALVE, OUTPUT);
	
  PumpOff();

  lcd_status();
  lcd.print( FF("READY! pwr=" ) );
	lcd.print( fv_pump.pwr );
	StatDelay.start();

  // разрешение прерываний INT0 и INT1
  //  EIMSK  =  (1<<INT0)  | (1<<INT1);
	
  // настройка срабатывания прерываний на любому изменению
  //EICRA  =  (0<<ISC11) | (1<<ISC10) | (0<<ISC01) | (1<<ISC00);
	
	PORTD |= (1 << PORTD5) | (1 << PORTD6) | (1 << PORTD6);
  PORTB |= (1 << PORTB2);
	
  // разрешение прерываний с портов B (PCINT[7:0]) и D (PCINT[23:16]), и запрет с порта C (PCINT[14:8])
  PCICR  |= (1 << PCIE2) | (0 << PCIE1) | (1 << PCIE0);
	
  // маскирование всех ног, кроме PB0 и PD7 - по одной на PCINT0 и PCINT2
  PCMSK0 |= (0 << PCINT7)  | (0 << PCINT6)  | (0 << PCINT5)  | (0 << PCINT4)  | (0 << PCINT3)  | (1 << PCINT2)  | (0 << PCINT1)  | (0 << PCINT0);
  //PCMSK1 |=                (0 << PCINT14) | (0 << PCINT13) | (0 << PCINT12) | (0 << PCINT11) | (0 << PCINT10) | (0 << PCINT9)  | (0 << PCINT8);
  PCMSK2 |= (1 << PCINT23) | (1 << PCINT22) | (1 << PCINT21) | (0 << PCINT20) | (0 << PCINT19) | (0 << PCINT18) | (0 << PCINT17) | (0 << PCINT16);

}

//***********************************************************************************************************************************************************

void loop() 
{
/*
 	char s_time[20];
	pump_last_time += 13;
  pump_last_volume = SPEED_LPS * pump_last_time;
	
	fv_pump.cnt++;
	fv_pump.total_time += 37;
	
	sprintf(buf, "%s %04u", 
					sprintTime4(s_time, pump_last_time),
					pump_last_volume
					);
	// lcd.print(buf);
	Log.notice( FPSTR(STR_sd_CR), buf, __LINE__);
	
	lcd_statistic();
	delay(100);
 */
	
 
#ifdef LowPower_h
	// дремать если мотор выключен
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
		delay(10);	// чтобы байтики через serial прошли ...перед сном
		#ifdef DEBUG_WAKEUPS
		wakeupmode = 2;
		#endif
		LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_ON, TWI_OFF);
    //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
#if (LOG_LEVEL == LOG_LEVEL_VERBOSE)	
    Log.verbose(FF("wakeup %d" CR), __LINE__);
#endif
		#ifdef DEBUG_WAKEUPS
		wakeupmode = 3;
		#endif
  }
#else
	delay(100); // если нет LowPower
#endif

	loop_cnt++;

  if ( StatDelay.hasPassed(STAT_DELAY, true) )
  {
		// log_internal_state(loop_cnt);
					
		if (pump == PUMP_ON) 
		{
			lcd_status();
			// дописать текущий тайминг
		}
  }


  // if (0 & f_WDT) 
  // {
    // Shutdown(
      // //  1234567890123456
      // // FF("WATCHDOG ALERT!" ),
      // ERR_WATHCDOG);
  // }



	if (LCD_Timeout.hasPassed(LCD_TIMEOUT)) 
	{
		LCD_Timeout.stop();
		LCD_Timeout.add( -LCD_Timeout.elapsed() );

		Shutdown(STANDBY);
		#ifdef DEBUG_WAKEUPS
		wakeupmode = 4;
		lcd_statistic();
		#endif
	}
	else if ((pump == PUMP_OFF) && LCD_Timeout.hasPassed(LCD_STAT_TIME))
	{
		pump = PUMP_STANBDY;
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
	sw_Stat.loop();

  if (sw_LowMark.pressed() && !sw_HighMark.pressed()) 
  {
    Shutdown(
      //  1234567890123456
      // FF("Sensors error!" ),
      ERR_INVALID_SENSORS);
  }

}
