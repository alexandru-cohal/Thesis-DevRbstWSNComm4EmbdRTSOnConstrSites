#ifndef _GNSS_DEFS_H
#define _GNSS_DEFS_H

#include <stdint.h>
#include "time.h"

#define MAX_FIELDS 19
#define MAX_FIELD_SIZE 10
#define BUFFER_MAX 80

typedef enum
{
    UNKNOWN = 0,
    NORTH,
    SOUTH,
    EAST,
    WEST
} azmuth_t;

typedef struct
{
    uint8_t degrees; 
    double minutes;  
    azmuth_t azmuth; 
} location_t;

typedef struct
{
    uint8_t hour;   
    uint8_t minute; 
    uint8_t second; 
    uint8_t ms;     
} utc_time_t;

typedef enum
{
    INVALID,            
    GPS_FIX,            
    DGPS_FIX,           
    PPS_FIX,            
    REAL_TIME_KINEMATIC,
    FLOAT_RTK,          
    ESTIMATED,          
    MANUAL_MODE,        
    SIMULATION_MODE     
} fix_t;


typedef struct
{
    utc_time_t *fix_time;  
    location_t *lat;       
    location_t *lon;       
    fix_t fix;             
    uint8_t num_sats : 4;  
    float horizontal;      
    double altitude;       
    double height;         
    uint16_t last_update;  
    uint16_t station_id;   
} gga_t;


typedef enum
{
    LORAN_UNKNOWN,
    LORAN_ACTIVE,
    LORAN_VOID
} ACTIVE_t;

typedef struct
{
    location_t *lat;    
    location_t *lon;    
    utc_time_t *fix_time;
    ACTIVE_t active;    
} gll_t;

typedef enum
{
    GSA_UNKNOWN = 0,
    GSA_NO_FIX,     
    GSA_2D_FIX,     
    GSA_3D_FIX,     
    GSA_AUTO_MODE,
    GSA_MANUAL_MODE
} GSA_MODE_t;

typedef struct
{
    GSA_MODE_t mode;    
    GSA_MODE_t fix;
    uint8_t sats[12];   
    float pdop;         
    float hdop;         
    float vdop;         
} gsa_t;

typedef struct
{
    uint8_t num_sentences : 2; 
    uint8_t sentence : 2;      
    uint8_t num_sats : 6;      
    struct
    {
        uint8_t sat_prn_num;   
        uint8_t elevation;     
        uint8_t azimuth;       
        uint8_t snr;           
    } sat_info[4];
} gsv_t;

typedef enum
{
    RMC_UKNOWN,
    RMC_ACTIVE,
    RMC_VOID,
    RMC_AUTONOMOUS,
    RMC_DIFFERENTIAL,
    RMC_NOT_VALID
} GNSS_STATUS_t;

typedef struct
{
    utc_time_t *fix_time; 
    GNSS_STATUS_t status; 
    location_t *lat;    
    location_t *lon;    
    double speed;       
    double track;       
    TimeStruct *date;   
    struct
    {
        double mag_variation; 
        azmuth_t azmuth;
    } magnetic;         
    GNSS_STATUS_t mode;

} rmc_t;

typedef struct
{
    double track;    
    double mag_track;
    double speed_knots; 
    double speed_km; 
} vtg_t;


#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
} // extern "C"
#endif

#endif /*GNSS_DEFS_H_*/

/*** End of File **************************************************************/
