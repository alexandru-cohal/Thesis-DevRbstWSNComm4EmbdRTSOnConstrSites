#ifndef GNSS_PARSER_H_
#define GNSS_PARSER_H_
 
#include <stdint.h>
#include "time.h"
#include "gnss_defs.h"
 
#ifdef __cplusplus
extern "C" {
#endif

void gnss_put( char input );
 
bool gnss_parse();
/*
 * Converts longitude or latitude from Degrees, Minutes format to
 * Degrees, Decimal degrees format, which is also commonly used
*/
double minutes_to_degrees ( uint8_t degrees, double minutes);
 
 
/***************** Common ***************/
location_t* gnss_current_lon( void );
 
location_t* gnss_current_lat( void );
 
TimeStruct* gnss_current_time( void );
 
utc_time_t* gnss_current_fix( void );
 
/****************** GGA ******************/
fix_t gnss_gga_fix_quality( void );
 
uint8_t gnss_gga_satcount( void );
 
float gnss_gga_hor_dilution( void );
 
double gnss_gga_altitude( void );
 
double gnss_gga_msl( void );

uint16_t gnss_gga_lastDGPS_update( void );
 
uint16_t gnss_gga_DGPS_stationID( void );
 
/***************** GLL ******************/
ACTIVE_t gnss_gll_active( void );
 
/***************** GSA ******************/
GSA_MODE_t gnss_gsa_mode( void );
 
GSA_MODE_t gnss_gsa_fix_type( void );
 
uint8_t *gnss_gsa_sat_prn( void );
 
float gnss_gsa_precision_dilution( void );
 
float gnss_gsa_horizontal_dilution( void );
 
float gnss_gsa_vertical_dilution( void );
 
/***************** GSV *****************/
// TODO: Must combine sat info and clear
/***************** RMC *****************/
GNSS_STATUS_t gnss_rmc_status( void );
 
double gnss_rmc_speed( void );
 
double gnss_rmc_track( void );
 
double gnss_rmc_mag_var( void );

azmuth_t gnss_rmc_direction( void );
 
GNSS_STATUS_t gnss_rmc_mode( void );
 
/**************** VTG *****************/
double gnss_vtg_track( void );
 
double gnss_vtg_mag( void );

double gnss_vtg_speedknt( void );
 
double gnss_vtg_speedkm( void );
 
#ifdef __cplusplus
} // extern "C"
#endif

#endif /*GNSS_PARSER_H_*/
 
/*** End of File **************************************************************/
