/****************************************************************************
* Title                 :   Time library
* Filename              :   time.h
* Author                :   RL
* Origin Date           :   08/25/2015
* Notes                 :   None
*****************************************************************************/
/**************************CHANGE LIST **************************************
*
*    Date    Software Version    Initials   Description
*  08/25/15    1.01              RL         Platform independent version
*
*****************************************************************************/
#ifndef _TIME_H
#define _TIME_H
 
/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
 
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
 
#define Time_secInMn    60                      // seconds per minute
#define Time_secInH     (Time_secInMn * 60)     // seconds per hour
#define Time_secIn24h   (Time_secInH * 24)      // seconds per day
 
 
/******************************************************************************
Configuration Constants
*******************************************************************************/
 
/******************************************************************************
* Macros
*******************************************************************************/
 
/******************************************************************************
* Typedefs
*******************************************************************************/
typedef struct
{
    unsigned char   ss ;    
    unsigned char   mn ;    
    unsigned char   hh ;    
    unsigned char   md ;    
    unsigned char   wd ;    
    unsigned char   mo ;    
    unsigned int    yy ;    
} TimeStruct;
 
/******************************************************************************
* Variables
*******************************************************************************/
extern  long            Time_jd1970 ;   // 01/01/1970 julian day number
 
/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
 
/*
* public functions
*/
long Time_dateToEpoch(TimeStruct *ts) ;
long Time_dateDiff(TimeStruct *t1, TimeStruct *t2);
void Time_epochToDate(long e, TimeStruct *ts) ;
 
#ifdef __cplusplus
} // extern "C"
#endif
 
#endif /*TIME_H_*/
 
/*** End of File **************************************************************/
