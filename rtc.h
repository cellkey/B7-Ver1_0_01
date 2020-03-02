/*
 * rtc.h
 *
 *  Created on: Feb 24, 2013
 *      Author: boaz
 */

#ifndef RTC_H_
#define RTC_H_

#define RTC_ADDRESS  0xD0   //PCF8523


unsigned char rtc_read_bcd(unsigned char addr);
void rtc_write_bcd(unsigned char addr,unsigned char data);
void rtc_set_time(unsigned char hour,unsigned char min,unsigned char sec);
void rtc_set_date(unsigned char date,unsigned char month,unsigned year);
void rtc_stop(void);
void rtc_start(void);
//void RTCwrite(unsigned char start_reg, unsigned char buf_len, unsigned char *buf);
//void RTCread(unsigned char start_reg, unsigned char buf_len, unsigned char *buf);
//void RTCSetTime(void);
void RTCSetDate(void);
void RTCgetTime(void);
void RTCgetDate(void);
unsigned char RTCGetDOW(void);
unsigned int GetCurrentMinutsTime(void);
void GetTimeFromMinutsTime(eeprom unsigned int *min_time, unsigned char *hour, unsigned char *min);
void rtc_set_timeAll(char *clockBuf);
void RTC_Setup(void);
void set_rtc_alarm(unsigned char day,unsigned char hour,unsigned char min, unsigned char DWeek );
void SetAlarmTiming(unsigned int NewVal);

#endif /* RTC_H_ */
