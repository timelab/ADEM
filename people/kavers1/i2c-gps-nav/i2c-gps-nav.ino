/*******************************************************************************************************************************
 * I2CGPS  - Inteligent GPS and NAV module for MultiWii by EOSBandi
 * V2.2   
 *
 * This program implements position hold and navigational functions for MultiWii by offloading caclulations and gps parsing 
 * into a secondary arduino processor connected to the MultiWii via i2c.
 * Once activated it outputs desired banking in a lat/lon coordinate system, which can be easily rotated into the copter's frame of reference.
 *
 * Navigation and Position hold routines and PI/PID libraries are based on code and ideas from the Arducopter team:
 * Jason Short,Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen
 * Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 * Status blink code from Guru_florida
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version. see <http://www.gnu.org/licenses/>
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
***********************************************************************************************************************************/

#define VERSION 22                                                         //Software version for cross checking


#include "WireMW.h"
#include "PIDCtrl.h"
#include "PICtrl.h"
#include "LeadFilter.h"


#include "registers.h"                                                    //Register definitions
#include "config.h"

#ifdef __AVR__
  #include <avr/power.h>
#endif


#ifdef NEOPIXEL_FEEDBACK
#include <Adafruit_NeoPixel.h>
#endif

int state_r = 0;
int state_g = 0;
int state_b = 0;

int state2_r = 0;
int state2_g = 0;
int state2_b = 0;

#define REG_MAP_SIZE       sizeof(i2c_dataset)       //size of register map
#define MAX_SENT_BYTES     0x0C                      //maximum amount of data that I could receive from a master device (register, plus 11 byte waypoint data)

#define LAT  0
#define LON  1

///////////////////////////////////////////////////////////////////////////////////////////////////
// Which command I got from the host ? in GPSMode variable
#define GPSMODE_NONAV 0        // no navigation
#define GPSMODE_HOLD  1        // pos hold
#define GPSMODE_WP    2        // wp navigation
///////////////////////////////////////////////////////////////////////////////////////////////////
// Navigation state machine states (nav_mode)
#define NAV_MODE_NONE              0
#define NAV_MODE_POSHOLD           1
#define NAV_MODE_WP                2
// Convert deg*100 to radians
#define RADX100                    0.000174532925  
//Blink feedback, by guru_florida
#define BLINK_INTERVAL  90


// Set up gps lag
#if defined(UBLOX)
  #define GPS_LAG 0.5f			//UBLOX GPS has a smaller lag than MTK and other
#else 
  #define GPS_LAG 1.0f				//We assumes that MTK GPS has a 1 sec lag
#endif  

#if defined(INIT_MTK_GPS)

 #define MTK_SET_BINARY          "$PGCMD,16,0,0,0,0,0*6A\r\n"
 #define MTK_SET_NMEA            "$PGCMD,16,1,1,1,1,1*6B\r\n"
 #define MTK_SET_NMEA_SENTENCES  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
 #define MTK_OUTPUT_4HZ          "$PMTK220,250*29\r\n"
 #define MTK_OUTPUT_5HZ          "$PMTK220,200*2C\r\n"
 #define MTK_OUTPUT_10HZ         "$PMTK220,100*2F\r\n"
 #define MTK_NAVTHRES_OFF        "$PMTK397,0*23\r\n" // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s  
 #define SBAS_ON                 "$PMTK313,1*2E\r\n"
 #define WAAS_ON                 "$PMTK301,2*2E\r\n"
 #define SBAS_TEST_MODE			 "$PMTK319,0*25\r\n"	//Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)

#endif


typedef struct {
  uint8_t    new_data:1;
  uint8_t    gps2dfix:1;
  uint8_t    gps3dfix:1;
  uint8_t    wp_reached:1;
  uint8_t    numsats:4;
} STATUS_REGISTER;

typedef struct {
  uint8_t command:4;
  uint8_t     wp:4;
} COMMAND_REGISTER;


typedef struct {
 uint8_t     active_wp:4;
 uint8_t     pervious_wp:4;
} WAYPOINT_REGISTER;

typedef struct {
  long      lat;            //degree*10 000 000
  long      lon;            //degree*10 000 000
} GPS_COORDINATES;

typedef struct {
  GPS_COORDINATES position;    //GPS coordinate of the waypoint
  uint16_t        altitude;    //altitude of the waypoint in meters (altitude is relative to the startup altitude)
  uint8_t         flags;       //to be defined
} WAYPOINT;

typedef struct {

//Status and command registers
  STATUS_REGISTER       status;                   // 0x00  status register
  COMMAND_REGISTER      command;                  // 0x01  command register
  WAYPOINT_REGISTER     wp_register;              // 0x02  waypoint register (current, previus)
  uint8_t               sw_version;               // 0x03  Version of the I2C_GPS sw
  uint8_t               res3;                     // 0x04  reserved for future use
  uint8_t               res4;                     // 0x05  reserved for future use
  uint8_t    	 	res5;                     // 0x06  reserved for future use

//GPS & navigation data
  GPS_COORDINATES       gps_loc;                  // current location (8 byte) lat,lon
  int16_t	        nav_lat;                  // The desired bank towards North (Positive) or South (Negative)      1 deg = 100 max 30deg (3000)
  int16_t	        nav_lon;                  // The desired bank towards East (Positive) or West (Negative)        1 deg = 100 max 30deg (3000)
  uint32_t              wp_distance;              // distance to active coordinates  (calculated) in cm
  int16_t               wp_target_bearing;        // direction to active coordinates (calculated)   1deg = 10 / -1800 - 1800
  int16_t               nav_bearing;              // crosstrack corrected navigational bearing 1deg = 10
  int16_t               home_to_copter_bearing;   // 1deg = 10
  uint16_t              distance_to_home;         // distance to home in cm
  uint16_t              ground_speed;             // ground speed from gps m/s*100
  int16_t               altitude;                 // gps altitude
  uint16_t	        ground_course;	          // GPS ground course
  uint16_t              res6;                     // reserved for future use
  uint32_t		time;	                  // UTC Time from GPS
  
//Parameters
  uint8_t		nav_crosstrack_gain;      // Crosstrack gain *100 (1 - 0.01 , 100 - 1)
  uint8_t		nav_speed_min;		  // minimum speed for navigation cm/s
  uint16_t		nav_speed_max;		  // maxiumum speed for navigation cm/s
  uint16_t		nav_bank_max;		  // maximum banking 1deg = 100, 30deg = 3000
  uint16_t		wp_radius;		  // waypoint radius, if we within this radius, then we considered that the wp is reached in cm
  uint8_t               nav_flags;                // navigational flags to be defined
             
  
//PID values
  uint8_t              poshold_p;		  // *100
  uint8_t              poshold_i;		  // *100
  uint8_t              poshold_imax;              // *1

  uint8_t              poshold_rate_p;  	  // *10
  uint8_t              poshold_rate_i;	    	  // *100
  uint8_t              poshold_rate_d;		  // *1000
  uint8_t              poshold_rate_imax;	  // *1

  uint8_t              nav_p;			  // *10
  uint8_t              nav_i;			  // *100
  uint8_t              nav_d;			  // *1000
  uint8_t              nav_imax;		  // *1
  
  WAYPOINT              gps_wp[16];               // 16 waypoints, WP#0 is RTH position

  uint16_t				sonar_distance;

} I2C_REGISTERS;

static I2C_REGISTERS   i2c_dataset;

static GPS_COORDINATES _target;                    //internal target location register

static uint8_t         receivedCommands[MAX_SENT_BYTES];
static uint8_t         new_command;                        //new command received (!=0)


//////////////////////////////////////////////////////////////////////////////
// Variables and controllers

//PID controllers
 
PICtrl	pi_poshold_lat(POSHOLD_P, POSHOLD_I, POSHOLD_IMAX * 100);
PICtrl	pi_poshold_lon(POSHOLD_P, POSHOLD_I, POSHOLD_IMAX * 100);
PIDCtrl	pid_poshold_rate_lat(POSHOLD_RATE_P, POSHOLD_RATE_I, POSHOLD_RATE_D, POSHOLD_RATE_IMAX * 100);
PIDCtrl	pid_poshold_rate_lon(POSHOLD_RATE_P, POSHOLD_RATE_I, POSHOLD_RATE_D, POSHOLD_RATE_IMAX * 100);
PIDCtrl	pid_nav_lat(NAV_P,NAV_I,NAV_D,NAV_IMAX * 100);
PIDCtrl	pid_nav_lon(NAV_P,NAV_I,NAV_D,NAV_IMAX * 100);

LeadFilter xLeadFilter;      // Long GPS lag filter 
LeadFilter yLeadFilter;      // Lat  GPS lag filter 

// used to track the elapsed time between GPS reads
static uint32_t                 nav_loopTimer;
// Delta Time in milliseconds for navigation computations, updated with every good GPS read
static float 			dTnav;

//Actual navigation mode, this needed since we swith to poshold ence arrived at home
static int8_t  nav_mode = NAV_MODE_NONE;            //Navigation mode

static int16_t x_actual_speed = 0;
static int16_t y_actual_speed = 0;
static int32_t last_longitude = 0;
static int32_t last_latitude  = 0;

static int16_t x_rate_d;
static int16_t y_rate_d;

// this is used to offset the shrinking longitude as we go towards the poles
static float	GPS_scaleLonDown;
static float	GPS_scaleLonUp;

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t   x_rate_error;
static int16_t   y_rate_error;
static int32_t	 long_error, lat_error;

// The desired bank towards North (Positive) or South (Negative)
static int16_t	nav_lat;
// The desired bank towards East (Positive) or West (Negative)
static int16_t	nav_lon;

//Used for rotation calculations for GPS nav vector
static float sin_yaw_y;
static float cos_yaw_x;

//Currently used WP
static int32_t  GPS_WP_latitude,GPS_WP_longitude;
static uint16_t GPS_WP_altitude;
static uint8_t  GPS_WP_flags;

//Actual position for calculations
static int32_t  GPS_latitude,GPS_longitude;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the "next_WP" location in degrees * 100
static int32_t	target_bearing;
// This is the angle from the copter to the "next_WP" location
// with the addition of Crosstrack error in degrees * 100
static int32_t	nav_bearing;
// saves the bearing at takeof (1deg = 1) used to rotate to takeoff direction when arrives at home
static int16_t  nav_takeoff_bearing;  
////////////////////////////////////////////////////////////////////////////////
// Crosstrack
////////////////////////////////////////////////////////////////////////////////
// deg * 100, The original angle to the next_WP when the next_WP was set
// Also used to check when we pass a WP
static int32_t 	original_target_bearing;
// The amount of angle correction applied to target_bearing to bring the copter back on its optimum path
static int16_t	crosstrack_error;
////////////////////////////////////////////////////////////////////////////////
// The location of the copter in relation to home, updated every GPS read (1deg - 100)
static int32_t	home_to_copter_bearing;
// distance between plane and home in cm
static int32_t	home_distance;
// distance between plane and next_WP in cm
static int32_t	wp_distance;

// used for slow speed wind up when start navigation;
static int16_t waypoint_speed_gov;

// this is the navigation mode what is commanded
static uint8_t GPSMode = GPSMODE_NONAV;          


////////////////////////////////////////////////////////////////////////////////////
// Blink code variables
//
static uint32_t lastframe_time = 0;
static uint32_t _statusled_timer = 0;
static int8_t _statusled_blinks = 0;
static boolean _statusled_state = 0;

#ifdef NEOPIXEL_FEEDBACK
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEOPIXEL_NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

////////////////////////////////////////////////////////////////////////////////////
// Sonar variables
//
static uint32_t _sonar_timer = 0;

////////////////////////////////////////////////////////////////////////////////////
// moving average filter variables
//
static uint8_t GPS_filter_index = 0;
static int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
static int32_t GPS_filter_sum[2];
static int32_t GPS_read[2];
static int32_t GPS_filtered[2];
static int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
static int32_t GPS_lead_latitude, GPS_lead_longitude;

////////////////////////////////////////////////////////////////////////////////////
// this is used to offset the shrinking longitude as we go towards the poles	
// It's ok to calculate this once per waypoint setting, since it changes a little within the reach of a multicopter
//
void GPS_calc_longitude_scaling(int32_t lat)
{
	float rads 			= (abs((float)lat) / 10000000.0) * 0.0174532925;
	GPS_scaleLonDown 		= cos(rads);
	GPS_scaleLonUp 		        = 1.0f / cos(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Sets the waypoint to navigate, reset neccessary variables and calculate initial values
// Waypoint 16 is a virtual waypoint, it's a pos hold
void GPS_set_next_wp(uint8_t wp_number)
{
  if (wp_number > 16) { return; }      //Do nothing with a wrong WP number

  if (wp_number <=15 ) {
    GPS_WP_latitude  = i2c_dataset.gps_wp[wp_number].position.lat;
    GPS_WP_longitude = i2c_dataset.gps_wp[wp_number].position.lon;
    GPS_WP_altitude  = i2c_dataset.gps_wp[wp_number].altitude;
    GPS_WP_flags     = i2c_dataset.gps_wp[wp_number].flags;
    i2c_dataset.status.wp_reached = 0;    
  } else {
    GPS_WP_latitude  = GPS_latitude;
    GPS_WP_longitude = GPS_longitude;
    GPS_WP_altitude  = 0;
    GPS_WP_flags     = 0;
    i2c_dataset.status.wp_reached = 1;        //With poshold we assume that the wp is reached    
  }    
  
  GPS_calc_longitude_scaling(GPS_WP_latitude);
  
  wp_distance = GPS_distance_cm(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
  target_bearing = GPS_bearing(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
  nav_bearing = target_bearing;
  GPS_calc_location_error(GPS_WP_latitude,GPS_WP_longitude,GPS_latitude,GPS_longitude);
  original_target_bearing = target_bearing;
  waypoint_speed_gov = i2c_dataset.nav_speed_min;
  
}

////////////////////////////////////////////////////////////////////////////////////
// Check if we missed the destination somehow
//
static bool check_missed_wp()
{
	int32_t temp;
	temp = target_bearing - original_target_bearing;
	temp = wrap_18000(temp);
	return (labs(temp) > 9000);	// we passed the waypoint by 100 degrees
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
//
uint32_t GPS_distance_cm(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
  float dLat = (lat2 - lat1);                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(lon2 - lon1) * GPS_scaleLonDown;
  float dist = sqrt(sq(dLat) + sq(dLon)) * 1.113195;
  return dist;
}

////////////////////////////////////////////////////////////////////////////////////
// get bearing from pos1 to pos2, returns an 1deg = 100 precision
//
int32_t GPS_bearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
        float off_x = (float)lon2 - lon1;
        float off_y = ((float)(lat2 - lat1)) * GPS_scaleLonUp;
	float bearing =	9000.0f + atan2(-off_y, off_x) * 5729.57795f;      //Convert the output redians to 100xdeg

        if (bearing < 0) bearing += 36000;
	return bearing;
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
static void GPS_calc_velocity( int32_t gps_latitude, int32_t gps_longitude){

	static int16_t x_speed_old = 0;
	static int16_t y_speed_old = 0;

	// y_GPS_speed positve = Up
	// x_GPS_speed positve = Right

	// initialise last_longitude and last_latitude
	if( last_longitude == 0 && last_latitude == 0 ) {
		last_longitude = gps_longitude;
		last_latitude = gps_latitude;
	}

	float tmp = 1.0/dTnav;
	x_actual_speed 	= (float)(gps_longitude - last_longitude) *  GPS_scaleLonDown * tmp;
	y_actual_speed	= (float)(gps_latitude  - last_latitude)  * tmp;

#if !defined(GPS_LEAD_FILTER)
	x_actual_speed	= (x_actual_speed + x_speed_old) / 2;
	y_actual_speed	= (y_actual_speed + y_speed_old) / 2;
	x_speed_old 	= x_actual_speed;
	y_speed_old 	= y_actual_speed;
#endif

	last_longitude 	= gps_longitude;
	last_latitude 	= gps_latitude;

#if defined(GPS_LEAD_FILTER)
        GPS_lead_longitude = xLeadFilter.get_position(gps_longitude,x_actual_speed, GPS_LAG);
        GPS_lead_latitude  = yLeadFilter.get_position(gps_latitude,y_actual_speed, GPS_LAG);
#endif


}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Becuase we are using lat and lon to do our distance errors here's a quick chart:
//	100 	= 1m
//	1000 	= 11m	 = 36 feet
//	1800 	= 19.80m = 60 feet
//	3000 	= 33m
//	10000 	= 111m
//
static void GPS_calc_location_error( int32_t target_lat, int32_t target_lng, int32_t gps_lat, int32_t gps_lng )
{
        // X Error
	long_error	= (float)(target_lng - gps_lng) * GPS_scaleLonDown; 
	// Y Error
	lat_error	= target_lat - gps_lat;						
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold(int x_error, int y_error)
{
	int32_t p,i,d;						
	int32_t output;
	int32_t x_target_speed, y_target_speed;

	// East / West
	x_target_speed 	= pi_poshold_lon.get_p(x_error);			// calculate desired speed from lon error
	x_rate_error	= x_target_speed - x_actual_speed;	                // calc the speed error

	p				= pid_poshold_rate_lon.get_p(x_rate_error);
	i				= pid_poshold_rate_lon.get_i(x_rate_error + x_error, dTnav);
	d				= pid_poshold_rate_lon.get_d(x_error, dTnav);
    d               = constrain(d, -2000, 2000);

    // get rid of noise
    if ( (i2c_dataset.nav_flags & I2C_NAV_FLAG_LOW_SPEED_D_FILTER) && (abs(x_actual_speed) < 50)) { 
        d = 0;
    }

	output			= p + i + d;
        nav_lon			= constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX); 		

	// North / South
	y_target_speed 	= pi_poshold_lat.get_p(y_error);			// calculate desired speed from lat error
	y_rate_error	= y_target_speed - y_actual_speed;

	p				= pid_poshold_rate_lat.get_p(y_rate_error);
	i				= pid_poshold_rate_lat.get_i(y_rate_error + y_error, dTnav);
	d				= pid_poshold_rate_lat.get_d(y_error, dTnav);
        d                               = constrain(d, -2000, 2000);
        // get rid of noise
        if ( (i2c_dataset.nav_flags & I2C_NAV_FLAG_LOW_SPEED_D_FILTER) && (abs(y_actual_speed) < 50)) { 
           d = 0;
        }
	output			= p + i + d;
	nav_lat			= constrain(output, -NAV_BANK_MAX, NAV_BANK_MAX); 

	// copy over I term to Nav_Rate -- if we change from poshold to RTH this will keep the wind compensation
	pid_nav_lon.set_integrator(pid_poshold_rate_lon.get_integrator());
	pid_nav_lat.set_integrator(pid_poshold_rate_lat.get_integrator());

}
////////////////////////////////////////////////////////////////////////////////////
// Calculate the desired nav_lat and nav_lon for distance flying such as RTH
//
static void GPS_calc_nav_rate(int max_speed)
{
	// push us towards the original track
	GPS_update_crosstrack();

	// nav_bearing includes crosstrack
	float temp 		= (9000l - nav_bearing) * RADX100;

	// East / West
	x_rate_error 	        = (cos(temp) * max_speed) - x_actual_speed; 
	x_rate_error 	        = constrain(x_rate_error, -1000, 1000);
	nav_lon			= pid_nav_lon.get_pid(x_rate_error, dTnav);
	nav_lon			= constrain(nav_lon, -NAV_BANK_MAX, NAV_BANK_MAX);

	// North / South
	y_rate_error 	        = (sin(temp) * max_speed) - y_actual_speed; 
	y_rate_error 	         = constrain(y_rate_error, -1000, 1000);	// added a rate error limit to keep pitching down to a minimum
	nav_lat			= pid_nav_lat.get_pid(y_rate_error, dTnav);
	nav_lat			= constrain(nav_lat, -NAV_BANK_MAX, NAV_BANK_MAX);

	// copy over I term to poshold_rate - So when arriving and entering to poshold we will have the wind compensation
	pid_poshold_rate_lon.set_integrator(pid_nav_lon.get_integrator());
	pid_poshold_rate_lat.set_integrator(pid_nav_lat.get_integrator());

}

////////////////////////////////////////////////////////////////////////////////////
// Calculating cross track error, this tries to keep the copter on a direct line 
// when flying to a waypoint.
//
static void GPS_update_crosstrack(void)
{
	if (abs(wrap_18000(target_bearing - original_target_bearing)) < 4500) {	 // If we are too far off or too close we don't do track following
		float temp = (target_bearing - original_target_bearing) * RADX100;
		crosstrack_error = sin(temp) * (wp_distance * (float)((float)i2c_dataset.nav_crosstrack_gain/100.0f));	 // Meters we are off track line
		nav_bearing = target_bearing + constrain(crosstrack_error, -3000, 3000);
		nav_bearing = wrap_36000(nav_bearing);
	}else{
		nav_bearing = target_bearing;
	}
}

////////////////////////////////////////////////////////////////////////////////////
// Determine desired speed when navigating towards a waypoint, also implement slow 
// speed rampup when starting a navigation
//
//	|< WP Radius
//	0  1   2   3   4   5   6   7   8m
//	...|...|...|...|...|...|...|...|
//		  100  |  200	  300	  400cm/s
//	           |  		 		            +|+
//	           |< we should slow to 1.5 m/s as we hit the target
//
static int16_t GPS_calc_desired_speed(int16_t max_speed, bool _slow)
{
	// max_speed is default 400 or 4m/s

	if(_slow){
		max_speed 		= min(max_speed, wp_distance / 2);
		max_speed 		= max(max_speed, 0);
	}else{
		max_speed 		= min(max_speed, wp_distance);
		max_speed 		= max(max_speed, i2c_dataset.nav_speed_min);	// go at least 100cm/s
	}

	// limit the ramp up of the speed
	// waypoint_speed_gov is reset to NAV_MIN_SPEED at each new WP command
	if(max_speed > waypoint_speed_gov){
		waypoint_speed_gov += (int)(100.0 * dTnav); // increase at .5/ms
		max_speed = waypoint_speed_gov;
	}

	return max_speed;
}

////////////////////////////////////////////////////////////////////////////////////
// Resets all GPS nev parameters and clears up the PID controllers. Prepares for a restarted poshold/navigation
//
void GPS_reset_nav()
{
      pi_poshold_lat.reset_I();
      pi_poshold_lon.reset_I();
      pid_poshold_rate_lon.reset_I();
      pid_poshold_rate_lat.reset_I();
      pid_nav_lon.reset_I();
      pid_nav_lat.reset_I();
      nav_lon = 0;
      nav_lat = 0;
}

////////////////////////////////////////////////////////////////////////////////////
// Update i2c_dataset from navigation output
//
void GPS_update_i2c_dataset()
{
 i2c_dataset.nav_lat           = nav_lat;
 i2c_dataset.nav_lon           = nav_lon;
 i2c_dataset.wp_distance       = wp_distance;
 i2c_dataset.wp_target_bearing = target_bearing;
 i2c_dataset.nav_bearing       = nav_bearing;
  
}

////////////////////////////////////////////////////////////////////////////////////
// Utilities
//
int32_t wrap_18000(int32_t error)
{
	if (error > 18000)	error -= 36000;
	if (error < -18000)	error += 36000;
	return error;
}

int32_t wrap_36000(int32_t angle)
{
	if (angle > 36000)	angle -= 36000;
	if (angle < 0)		angle += 36000;
	return angle;
}


#if defined(NMEA)

/* The latitude or longitude is coded this way in NMEA frames
  dddmm.mmmm   coded as degrees + minutes + minute decimal
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000
  I increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased 
  resolution also increased precision of nav calculations
*/

#define DIGIT_TO_VAL(_x)	(_x - '0')
uint32_t GPS_coord_to_degrees(char* s)
{
	char *p, *q;
	uint8_t deg = 0, min = 0;
	unsigned int frac_min = 0;

	// scan for decimal point or end of field
	for (p = s; isdigit(*p); p++)
		;
	q = s;

	// convert degrees
	while ((p - q) > 2) {
		if (deg)
			deg *= 10;
		deg += DIGIT_TO_VAL(*q++);
	}
	// convert minutes
	while (p > q) {
		if (min)
			min *= 10;
		min += DIGIT_TO_VAL(*q++);
	}
	// convert fractional minutes
	// expect up to four digits, result is in
	// ten-thousandths of a minute
	if (*p == '.') {
		q = p + 1;
		for (int i = 0; i < 4; i++) {
			frac_min *= 10;
			if (isdigit(*q))
				frac_min += *q++ - '0';
		}
	}
	return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

/* This is am expandable implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA, GSA and RMC frames to decode on the serial bus
   Using the following data :
   GGA
     - time
     - latitude
     - longitude
     - GPS fix 
     - GPS num sat (5 is enough to be +/- reliable)
     - GPS alt
   GSA
     - 3D fix (it could be left out since we have a 3D fix if we have more than 4 sats  
   RMC
     - GPS speed over ground, it will be handy for wind compensation (future)  
     
*/

#define NO_FRAME    0
#define GPGGA_FRAME 1
#define GPGSA_FRAME 2
#define GPRMC_FRAME 3

bool GPS_NMEA_newFrame(char c) {

  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, gps_frame = NO_FRAME;

  switch (c) {
    case '$': param = 0; offset = 0; parity = 0; 
              break;
    case ',':
    case '*':  string[offset] = 0;
                if (param == 0) { //frame identification
                  gps_frame = NO_FRAME;  
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') gps_frame = GPGGA_FRAME;
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'G' && string[3] == 'S' && string[4] == 'A') gps_frame = GPGSA_FRAME;
                  if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') gps_frame = GPRMC_FRAME;
                }
                  
                switch (gps_frame)
                {
                  
                  //************* GPGGA FRAME parsing
                  case GPGGA_FRAME: 
                    switch (param)
                     {
                      case 1: i2c_dataset.time = (atof(string)*1000);      //up to .000 s precision not needed really but the data is there anyway
                              break;
                      //case 2: i2c_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
                      case 2: GPS_read[LAT] = GPS_coord_to_degrees(string);
                              break;
                      //case 3: if (string[0] == 'S') i2c_dataset.gps_loc.lat = -i2c_dataset.gps_loc.lat;
                      case 3: if (string[0] == 'S') GPS_read[LAT] = -GPS_read[LAT];
                              break;
                      //case 4: i2c_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
                      case 4: GPS_read[LON] = GPS_coord_to_degrees(string);
                              break;
                      //case 5: if (string[0] == 'W') i2c_dataset.gps_loc.lon = -i2c_dataset.gps_loc.lon;
                      case 5: if (string[0] == 'W') GPS_read[LON] = -GPS_read[LON];
                              break;
                      case 6: i2c_dataset.status.gps2dfix = string[0]  > '0';
                              break;
                      case 7: i2c_dataset.status.numsats = atoi(string);
                              break;
                      case 9: i2c_dataset.altitude = atoi(string);
                              break;
                     }
                     #if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
                   break;         
                   //************* GPGSA FRAME parsing
                   case GPGSA_FRAME:
                     switch (param)
                     {
                      case 2: i2c_dataset.status.gps3dfix = string[0] == '3';
                      break;
                     }
                     #if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
                   break;
                   //************* GPGSA FRAME parsing
                   case GPRMC_FRAME:
                     switch(param)
                     {
                       case 7: i2c_dataset.ground_speed = (atof(string)*0.5144444)*10;      //convert to m/s*100
                               break; 
	               case 8: i2c_dataset.ground_course = (atof(string)*10);				//Convert to degrees *10 (.1 precision)
							   break;
                     }
                   #if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
                   break;                   
                }
            
                param++; offset = 0;
                if (c == '*') checksum_param=1;
                else parity ^= c;
                break;
     case '\r':
     case '\n':  
                if (checksum_param) { //parity checksum
                  uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
                  if (checksum == parity) frameOK = 1;
                }
                checksum_param=0;
                break;
     default:
             if (offset < 15) string[offset++] = c;
             if (!checksum_param) parity ^= c;
             
  }
  return frameOK && (gps_frame == GPGGA_FRAME);
}
#endif 

#if defined(UBLOX)

	struct ubx_header {
		uint8_t preamble1;
		uint8_t preamble2;
		uint8_t msg_class;
		uint8_t msg_id;
		uint16_t length;
	};

    struct ubx_nav_posllh {
        uint32_t	time;				// GPS msToW
        int32_t		longitude;
        int32_t		latitude;
        int32_t		altitude_ellipsoid;
        int32_t		altitude_msl;
        uint32_t	horizontal_accuracy;
        uint32_t	vertical_accuracy;
    };
    struct ubx_nav_status {
        uint32_t	time;				// GPS msToW
        uint8_t		fix_type;
        uint8_t		fix_status;
        uint8_t		differential_status;
        uint8_t		res;
        uint32_t	time_to_first_fix;
        uint32_t	uptime;				// milliseconds
    };
    struct ubx_nav_solution {
        uint32_t	time;
        int32_t		time_nsec;
        int16_t		week;
        uint8_t		fix_type;
        uint8_t		fix_status;
        int32_t		ecef_x;
        int32_t		ecef_y;
        int32_t		ecef_z;
        uint32_t	position_accuracy_3d;
        int32_t		ecef_x_velocity;
        int32_t		ecef_y_velocity;
        int32_t		ecef_z_velocity;
        uint32_t	speed_accuracy;
        uint16_t	position_DOP;
        uint8_t		res;
        uint8_t		satellites;
        uint32_t	res2;
    };
    struct ubx_nav_velned {
        uint32_t	time;				// GPS msToW
        int32_t		ned_north;
        int32_t		ned_east;
        int32_t		ned_down;
        uint32_t	speed_3d;
        uint32_t	speed_2d;
        int32_t		heading_2d;
        uint32_t	speed_accuracy;
        uint32_t	heading_accuracy;
    };

    enum ubs_protocol_bytes {
        PREAMBLE1 = 0xb5,
        PREAMBLE2 = 0x62,
        CLASS_NAV = 0x01,
        CLASS_ACK = 0x05,
        CLASS_CFG = 0x06,
		MSG_ACK_NACK = 0x00,
		MSG_ACK_ACK = 0x01,
        MSG_POSLLH = 0x2,
        MSG_STATUS = 0x3,
        MSG_SOL = 0x6,
        MSG_VELNED = 0x12,
        MSG_CFG_PRT = 0x00,
        MSG_CFG_RATE = 0x08,
        MSG_CFG_SET_RATE = 0x01,
		MSG_CFG_NAV_SETTINGS = 0x24
    };
    enum ubs_nav_fix_type {
        FIX_NONE = 0,
        FIX_DEAD_RECKONING = 1,
        FIX_2D = 2,
        FIX_3D = 3,
        FIX_GPS_DEAD_RECKONING = 4,
        FIX_TIME = 5
    };
    enum ubx_nav_status_bits {
        NAV_STATUS_FIX_VALID = 1
    };

    // Packet checksum accumulators
    static uint8_t		_ck_a;
    static uint8_t		_ck_b;

    // State machine state
    static uint8_t		_step;
    static uint8_t		_msg_id;
    static uint16_t	_payload_length;
    static uint16_t	_payload_counter;

    static bool        next_fix;
    
    static uint8_t     _class;

	// do we have new position information?
	static bool		_new_position;

	// do we have new speed information?
	static bool		_new_speed;

	static uint8_t	    _disable_counter;

    // Receive buffer
    static union {
        ubx_nav_posllh		posllh;
        ubx_nav_status		status;
        ubx_nav_solution	solution;
        ubx_nav_velned		velned;
        uint8_t	bytes[];
    } _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b)
{
	while (len--) {
		ck_a += *data;
		ck_b += ck_a;
		data++;
	}
}

bool GPS_UBLOX_newFrame(uint8_t data)
{
       bool parsed = false;

        switch(_step) {

        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
        case 0:
            if(PREAMBLE1 == data) _step++;
            break;

        case 2:
            _step++;
	    _class = data;
	    _ck_b = _ck_a = data;			// reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte
            _payload_length = data;				// payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);			// checksum byte

            _payload_length += (uint16_t)(data<<8);
			if (_payload_length > 512) {
				_payload_length = 0;
				_step = 0;
			}
            _payload_counter = 0;				// prepare to receive payload
            break;
        case 6:
            _ck_b += (_ck_a += data);			// checksum byte
			if (_payload_counter < sizeof(_buffer)) {
				_buffer.bytes[_payload_counter] = data;
			}
            if (++_payload_counter == _payload_length)
                _step++;
            break;
        case 7:
            _step++;
            if (_ck_a != data) _step = 0;						// bad checksum
            break;
        case 8:
            _step = 0;
            if (_ck_b != data)  break; 							// bad checksum
 	    if (UBLOX_parse_gps())  { parsed = true; }
        } //end switch
   return parsed;
}

bool UBLOX_parse_gps(void)
{

    switch (_msg_id) {
    case MSG_POSLLH:
        i2c_dataset.time	        = _buffer.posllh.time;
        GPS_read[LON]	                = _buffer.posllh.longitude;
        GPS_read[LAT]	                = _buffer.posllh.latitude;
        i2c_dataset.altitude  	        = _buffer.posllh.altitude_msl / 10 /100;      //alt in m
	i2c_dataset.status.gps3dfix	= next_fix;
	_new_position = true;
#if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
	break;
    case MSG_STATUS:
        next_fix	= (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
	if (!next_fix) i2c_dataset.status.gps3dfix = false;
#if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
        break;
    case MSG_SOL:
        next_fix	= (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
	if (!next_fix) i2c_dataset.status.gps3dfix = false;
        i2c_dataset.status.numsats	= _buffer.solution.satellites;
        //GPS_hdop		= _buffer.solution.position_DOP;
        //debug[3] = GPS_hdop;
        #if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
        break;
    case MSG_VELNED:
        //speed_3d	= _buffer.velned.speed_3d;				// cm/s
        i2c_dataset.ground_speed = _buffer.velned.speed_2d;				// cm/s
        i2c_dataset.ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);	// Heading 2D deg * 100000 rescaled to deg * 10
	_new_speed = true;
#if defined(NEOPIXEL_FEEDBACK)
                     setFrameStatus();
                     #endif
        break;
    default:
        return false;
    }

	// we only return true when we get new position and speed data
	// this ensures we don't use stale data
	if (_new_position && _new_speed) {
		_new_speed = _new_position = false;
		return true;
	}
	return false;
}


#endif //UBLOX

#if defined(MTK_BINARY16) || defined(MTK_BINARY19)
    struct diyd_mtk_msg {
        int32_t		latitude;
        int32_t		longitude;
        int32_t		altitude;
        int32_t		ground_speed;
        int32_t		ground_course;
        uint8_t		satellites;
        uint8_t		fix_type;
        uint32_t	utc_date;
        uint32_t	utc_time;
        uint16_t	hdop;
    };
// #pragma pack(pop)
    enum diyd_mtk_fix_type {
       FIX_NONE = 1,
	   FIX_2D = 2,
	   FIX_3D = 3,
	   FIX_2D_SBAS = 6,
	   FIX_3D_SBAS = 7 
    };

#if defined(MTK_BINARY16)
    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xd0,
        PREAMBLE2 = 0xdd,
    };
#endif

#if defined(MTK_BINARY19)
    enum diyd_mtk_protocol_bytes {
        PREAMBLE1 = 0xd1,
        PREAMBLE2 = 0xdd,
    };
#endif

    // Packet checksum accumulators
    uint8_t 	_ck_a;
    uint8_t 	_ck_b;

    // State machine state
    uint8_t 	_step;
    uint8_t		_payload_counter;

    // Time from UNIX Epoch offset
    long		_time_offset;
    bool		_offset_calculated;

    // Receive buffer
    union {
        diyd_mtk_msg	msg;
        uint8_t			bytes[];
    } _buffer;

inline long _swapl(const void *bytes)
{
    const uint8_t	*b = (const uint8_t *)bytes;
    union {
        long	v;
        uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return(u.v);
}

bool GPS_MTK_newFrame(uint8_t data)
{
       bool parsed = false;

restart:
        switch(_step) {

            // Message preamble, class, ID detection
            //
            // If we fail to match any of the expected bytes, we
            // reset the state machine and re-consider the failed
            // byte as the first byte of the preamble.  This
            // improves our chances of recovering from a mismatch
            // and makes it less likely that we will be fooled by
            // the preamble appearing as data in some other message.
            //
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            goto restart;
        case 2:
            if (sizeof(_buffer) == data) {
                _step++;
                _ck_b = _ck_a = data;				// reset the checksum accumulators
                _payload_counter = 0;
            } else {
                _step = 0;							// reset and wait for a message of the right class
                goto restart;
            }
            break;

            // Receive message data
            //
        case 3:
            _buffer.bytes[_payload_counter++] = data;
            _ck_b += (_ck_a += data);
            if (_payload_counter == sizeof(_buffer))
                _step++;
            break;

            // Checksum and message processing
            //
        case 4:
            _step++;
            if (_ck_a != data) {
                _step = 0;
            }
            break;
        case 5:
            _step = 0;
            if (_ck_b != data) {
                break;
            }

            i2c_dataset.status.gps3dfix 			= ((_buffer.msg.fix_type == FIX_3D) || (_buffer.msg.fix_type == FIX_3D_SBAS));
#if defined(MTK_BINARY16)
            GPS_read[LAT]		= _buffer.msg.latitude * 10;	// XXX doc says *10e7 but device says otherwise
            GPS_read[LON]		= _buffer.msg.longitude * 10;	// XXX doc says *10e7 but device says otherwise
#endif
#if defined(MTK_BINARY19)
            GPS_read[LAT]		= _buffer.msg.latitude;	// XXX doc says *10e7 but device says otherwise
            GPS_read[LON]		= _buffer.msg.longitude;	// XXX doc says *10e7 but device says otherwise
#endif
			i2c_dataset.altitude		= _buffer.msg.altitude /100;
            i2c_dataset.ground_speed	                = _buffer.msg.ground_speed;
            i2c_dataset.ground_course	        = _buffer.msg.ground_course;
            i2c_dataset.status.numsats		        = _buffer.msg.satellites;
            //GPS_hdop			= _buffer.msg.hdop;
            parsed = true;
            //GPS_Present = 1;
        }
    return parsed;
}
#endif //MTK



//////////////////////////////////////////////////////////////////////////////////////
// I2C handlers
// Handler for requesting data
//
void requestEvent()
{
 if (receivedCommands[0] >= I2C_GPS_GROUND_SPEED) i2c_dataset.status.new_data = 0;        //Accessing gps data, switch new_data_flag;
 //Write data from the requested data register position
 Wire.write((uint8_t *)&i2c_dataset+receivedCommands[0],32);                    //Write up to 32 byte, since master is responsible for reading and sending NACK
 //32 byte limit is in the Wire library, we have to live with it unless writing our own wire library

}

//Handler for receiving data
void receiveEvent(int bytesReceived)
{
     uint8_t  *ptr;
     for (int a = 0; a < bytesReceived; a++) {
          if (a < MAX_SENT_BYTES) {
               receivedCommands[a] = Wire.read();
          } else {
               Wire.read();  // if we receive more data then allowed just throw it away
          }
     }

    if (receivedCommands[0] == I2C_GPS_COMMAND) { new_command = receivedCommands[1]; return; }  //Just one byte, ignore all others

     if(bytesReceived == 1 && (receivedCommands[0] < REG_MAP_SIZE)) { return; }        //read command from a given register
     if(bytesReceived == 1 && (receivedCommands[0] >= REG_MAP_SIZE)){                  //Addressing over the reg_map fallback to first byte
          receivedCommands[0] = 0x00;
          return;
     }
    //More than 1 byte was received, so there is definitely some data to write into a register
    //Check for writeable registers and discard data is it's not writeable
    
    if ((receivedCommands[0]>=I2C_GPS_CROSSTRACK_GAIN) && (receivedCommands[0]<=REG_MAP_SIZE)) {    //Writeable registers above I2C_GPS_WP0
     ptr = (uint8_t *)&i2c_dataset+receivedCommands[0];
     for (int a = 1; a < bytesReceived; a++) { *ptr++ = receivedCommands[a]; }
    }
}


void blink_sonar_update()
{

	uint32_t now = millis();

#if defined(SONAR) && !defined(MAXBOTIX_PWM)
  if(_sonar_timer < now)//update sonar readings every 50ms
  {
   _sonar_timer = now + SONAR_UPDATE_RATE_US;
   Sonar_update();
  }
#endif

  if(_statusled_timer < now) {
    if(lastframe_time+5000 < now) {
      // no gps communication
        state2_r = 255;
        state2_g = 0;
        state2_b = 0;
        
      
      _statusled_state = !_statusled_state;
      digitalWrite(13, _statusled_state ? HIGH : LOW);   // set the LED off
      #if defined(NEOPIXEL_FEEDBACK)                     
      _statusled_state ? setColour() : clearColour();
      #endif
      _statusled_timer = now + 1000;
      return;
    }
        
    if(_statusled_blinks==0) {
      if(i2c_dataset.status.gps3dfix == 1) {
        _statusled_blinks=3;
        #if defined(NEOPIXEL_FEEDBACK)
        state2_r = 0;
        state2_g = 255;
        state2_b = 0;
        #endif
      } else if(i2c_dataset.status.gps2dfix == 1) {
        #if defined(NEOPIXEL_FEEDBACK)
        state2_r = 0;
        state2_g = 255;
        state2_b = 0;
        #endif
        _statusled_blinks=2;
      } else {
        _statusled_blinks=1;    
      }
    }
    
    if(_statusled_state) {
      _statusled_blinks--;
      _statusled_state = false;
      _statusled_timer = now + ((_statusled_blinks>0) ? BLINK_INTERVAL : 1000);
      digitalWrite(13, LOW);   // set the LED off
      #if defined(NEOPIXEL_FEEDBACK)
      clearColour();
      #endif
    } else {
      _statusled_state = true;
      _statusled_timer = now + BLINK_INTERVAL;
      digitalWrite(13, HIGH);   // set the LED on
      #if defined(NEOPIXEL_FEEDBACK)
      setColour();
      #endif
    }
  }
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPS initialisation
//

  uint32_t init_speed[5] = {9600,19200,38400,57600,115200};

 #if defined(UBLOX)
   const char UBLOX_INIT[] PROGMEM = {                          // PROGMEM array must be outside any function !!!
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19,                            //disable all default NMEA messages
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13,
     0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17,
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47,                            //set POSLLH MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x03,0x01,0x0F,0x49,                            //set STATUS MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F,                            //set SOL MSG rate
     0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x12,0x01,0x1E,0x67,                            //set VELNED MSG rate
     0xB5,0x62,0x06,0x16,0x08,0x00,0x03,0x07,0x03,0x00,0x51,0x08,0x00,0x00,0x8A,0x41,   //set WAAS to EGNOS
     0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
   };
 #endif

  void GPS_SerialInit() {
  Serial.begin(GPS_SERIAL_SPEED);  
  delay(1000);

#if defined(UBLOX)
	//Set speed
      for(uint8_t i=0;i<5;i++){
        Serial.begin(init_speed[i]);          // switch UART speed for sending SET BAUDRATE command (NMEA mode)
        #if (GPS_SERIAL_SPEED==19200)
          Serial.write(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));     // 19200 baud - minimal speed for 5Hz update rate
        #endif  
        #if (GPS_SERIAL_SPEED==38400)
          Serial.write(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));     // 38400 baud
        #endif  
        #if (GPS_SERIAL_SPEED==57600)
          Serial.write(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));     // 57600 baud
        #endif  
        #if (GPS_SERIAL_SPEED==115200)
          Serial.write(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));    // 115200 baud
        #endif  
        delay(300);		//Wait for init 
      }
      delay(200);
      Serial.begin(GPS_SERIAL_SPEED);  
      for(uint8_t i=0; i<sizeof(UBLOX_INIT); i++) {                        // send configuration data in UBX protocol
        Serial.write(pgm_read_byte(UBLOX_INIT+i));
        //delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
      }

#elif defined(INIT_MTK_GPS)                            // MTK GPS setup
      for(uint8_t i=0;i<5;i++){
        Serial.begin(init_speed[i]);					   // switch UART speed for sending SET BAUDRATE command
        #if (GPS_SERIAL_SPEED==19200)
          Serial.write("$PMTK251,19200*22\r\n");     // 19200 baud - minimal speed for 5Hz update rate
        #endif  
        #if (GPS_SERIAL_SPEED==38400)
          Serial.write("$PMTK251,38400*27\r\n");     // 38400 baud
        #endif  
        #if (GPS_SERIAL_SPEED==57600)
          Serial.write("$PMTK251,57600*2C\r\n");     // 57600 baud
        #endif  
        #if (GPS_SERIAL_SPEED==115200)
          Serial.write("$PMTK251,115200*1F\r\n");    // 115200 baud
        #endif  
        delay(300);
      }
      // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
      Serial.begin(GPS_SERIAL_SPEED);

	  Serial.write(MTK_NAVTHRES_OFF);
	  delay(100);
      Serial.write(SBAS_ON);
	  delay(100);
      Serial.write(WAAS_ON);
	  delay(100);
      Serial.write(SBAS_TEST_MODE);
	  delay(100);
      Serial.write(MTK_OUTPUT_5HZ);           // 5 Hz update rate
	  delay(100);

      #if defined(NMEA)
        Serial.write(MTK_SET_NMEA_SENTENCES); // only GGA and RMC sentence
      #endif     
	  #if defined(MTK_BINARY19) || defined(MTK_BINARY16)
		Serial.write(MTK_SET_BINARY);
      #endif



#endif     
  }

#if defined(SONAR)

volatile uint32_t Sonar_starTime = 0;
volatile uint32_t Sonar_echoTime = 0;
volatile uint16_t Sonar_waiting_echo = 0;

void Sonar_init()
{
  // Pin change interrupt control register - enables interrupt vectors
  PCICR  |= (1<<PCIE1); // Port C
 
  // Pin change mask registers decide which pins are enabled as triggers
  PCMSK1 |= (1<<PCINT10); // pin 2 PC2
  
  DDRC |= 0x08; //triggerpin PC3 as output
#if !defined(MAXBOTIX_PWM) 
  Sonar_update();
#endif
}

ISR(PCINT1_vect) {

    //uint8_t pin = PINC;

    if (PINC & 1<<PCINT10) {     //indicates if the bit 0 of the arduino port [B0-B7] is at a high state
      Sonar_starTime = micros();
    }
    else {
      Sonar_echoTime = micros() - Sonar_starTime; // Echo time in microseconds
      int maxTime = (SONAR_MAX_DISTANCE * SONAR_US_PER_CM);
      if (Sonar_echoTime <= maxTime) {     // valid distance
        i2c_dataset.sonar_distance = Sonar_echoTime / SONAR_US_PER_CM;
        Serial.print(i2c_dataset.sonar_distance);
        Serial.println("cm");
      }
      else
      {
      // No valid data
        i2c_dataset.sonar_distance = -1;
      }
      Sonar_waiting_echo = 0;
    }
}


void Sonar_update()
{
 
#if !defined(MAXBOTIX_PWM)
  if (Sonar_waiting_echo == 0)
  {
    // Send 2ms LOW pulse to ensure we get a nice clean pulse
    PORTC &= ~(0x08);//PC3 low    
    delayMicroseconds(2);
   
    // send 10 microsecond pulse
    PORTC |= (0x08);//PC3 high 
    // wait 10 microseconds before turning off
    delayMicroseconds(SONAR_PULSE_LENGTH_US);
    // stop sending the pulse
    PORTC &= ~(0x08);//PC3 low
   
    Sonar_waiting_echo = 1;
  }
#endif
}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
//
void setup() {
  #if defined(NEOPIXEL_FEEDBACK)
  state_r = 255;
  state_g = 0;
  state_b = 0;
  
  state2_r = 255;
  state2_g = 0;
  state2_b = 0;
  
  pixels.begin();
  #endif
  uint8_t i;

  //Init Sonar
#if defined(SONAR)
  Sonar_init();
#endif

  //Init GPS
  GPS_SerialInit();
  
  //Init i2c_dataset;
  uint8_t *ptr = (uint8_t *)&i2c_dataset;
  for (i=0;i<sizeof(i2c_dataset);i++) { *ptr = 0; ptr++;}

  //Set up default parameters
  i2c_dataset.sw_version          = VERSION;

  i2c_dataset.nav_crosstrack_gain = 100; // Crosstrack gain = 1
  i2c_dataset.nav_speed_min       = 100; // cm/s
  i2c_dataset.nav_speed_max       = 400; // cm/s
  i2c_dataset.nav_bank_max        = 3000; // 30deg
  i2c_dataset.wp_radius           = 200;   //cm -> 2m
  
  i2c_dataset.poshold_p           = POSHOLD_P * 100;
  i2c_dataset.poshold_i           = POSHOLD_I * 100;
  i2c_dataset.poshold_imax        = POSHOLD_IMAX;
  
  i2c_dataset.poshold_rate_p      = POSHOLD_RATE_P * 10;
  i2c_dataset.poshold_rate_i      = POSHOLD_RATE_I * 100;
  i2c_dataset.poshold_rate_d      = POSHOLD_RATE_D * 1000;
  i2c_dataset.poshold_rate_imax   = POSHOLD_RATE_IMAX;
  
  i2c_dataset.nav_p               = NAV_P * 10;
  i2c_dataset.nav_i               = NAV_I * 100;
  i2c_dataset.nav_d               = NAV_D * 1000;
  i2c_dataset.nav_imax            = NAV_IMAX;

  i2c_dataset.nav_flags           = 0x80 + 0x40;      // GPS filter and low speed filters are on

  //Start I2C communication routines
  Wire.begin(I2C_ADDRESS);               // DO NOT FORGET TO COMPILE WITH 400KHz!!! else change TWBR Speed to 100khz on Host !!! Address 0x40 write 0x41 read
  Wire.onRequest(requestEvent);          // Set up event handlers
  Wire.onReceive(receiveEvent);

}

#if defined(NEOPIXEL_FEEDBACK)
void setFrameStatus() {
 state_r = 0;
 state_g = 0;
 state_b = 255;

 if (i2c_dataset.status.gps2dfix==true)
 {
   state_r = 127;
   state_g = 0;
   state_b = 110; 
 }
 
 if (i2c_dataset.status.gps3dfix==true) 
 {
   state_r = 255;
   state_g = 255;
   state_b = 255; 
 } 
 

}

void setColour() {
  for(int i=0;i<6;i++){
    pixels.setPixelColor(i, pixels.Color(state_r,state_g,state_b)); 
  }
  
  int count = i2c_dataset.status.numsats;
  int x = 0;
  pixels.setPixelColor(6, pixels.Color(state2_r,state2_g,state2_b));
  for(int i=7;i<12;i++){
      if (x < count) { 
        pixels.setPixelColor(i, pixels.Color(state2_r,state2_g,state2_b));
      } else {
        pixels.setPixelColor(i, pixels.Color(0,0,0));
      } 
      x++;
  }
  
  pixels.show(); // This sends the updated pixel color to the hardware.
}

void clearColour() {
  for(int i=0;i<12;i++){
    pixels.setPixelColor(i, pixels.Color(0,0,0)); 
  }
  
  pixels.show(); // This sends the updated pixel color to the hardware.
}

#endif
/******************************************************************************************************************/
/******************************************************* Main loop ************************************************/
/******************************************************************************************************************/
void loop() {
  
  static uint8_t GPS_fix_home;
  static uint8_t _command_wp;
  static uint8_t _command;
  static uint32_t _watchdog_timer = 0;
  uint8_t axis;
  uint16_t fraction3[2];

#pragma region while serial available

     while (Serial.available()) {

#if defined(NMEA)
       if (GPS_NMEA_newFrame(Serial.read())) {
#endif 
#if defined(UBLOX)
       if (GPS_UBLOX_newFrame(Serial.read())) {
#endif
#if defined(MTK_BINARY16) || defined(MTK_BINARY19)
       if (GPS_MTK_newFrame(Serial.read())) {
#endif


       // We have a valid GGA frame and we have lat and lon in GPS_read_lat and GPS_read_lon, apply moving average filter
       // this is a little bit tricky since the 1e7/deg precision easily overflow a long, so we apply the filter to the fractions
       // only, and strip the full degrees part. This means that we have to disable the filter if we are very close to a degree line

#pragma region GPS FILTER
       if (i2c_dataset.nav_flags & I2C_NAV_FLAG_GPS_FILTER) {      //is filtering switched on ?

         GPS_filter_index = ++GPS_filter_index % GPS_FILTER_VECTOR_LENGTH;
         
         for (axis = 0; axis< 2; axis++) {
			GPS_degree[axis] = GPS_read[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
  
	         // How close we are to a degree line ? its the first three digits from the fractions of degree
			//Check if we are close to a degree line, if yes, disable averaging,
			fraction3[axis] = (GPS_read[axis]- GPS_degree[axis]*10000000) / 10000;
	  
			GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
			GPS_filter[axis][GPS_filter_index] = GPS_read[axis] - (GPS_degree[axis]*10000000); 
			GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
			GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
         }       
         
         if ( nav_mode == NAV_MODE_POSHOLD) {      //we use gps averaging only in poshold mode...
             if ( fraction3[LAT]>1 && fraction3[LAT]<999 ) i2c_dataset.gps_loc.lat = GPS_filtered[LAT]; else i2c_dataset.gps_loc.lat = GPS_read[LAT];
             if ( fraction3[LON]>1 && fraction3[LON]<999 ) i2c_dataset.gps_loc.lon = GPS_filtered[LON]; else i2c_dataset.gps_loc.lon = GPS_read[LON];       
         } else {
             i2c_dataset.gps_loc.lat = GPS_read[LAT];
             i2c_dataset.gps_loc.lon = GPS_read[LON];
         }
       } else { // ignore filtering since it switced off in the nav_flags
           i2c_dataset.gps_loc.lat = GPS_read[LAT];
           i2c_dataset.gps_loc.lon = GPS_read[LON];
       }
       
#pragma endregion

       if (i2c_dataset.status.gps3dfix == 1 && i2c_dataset.status.numsats >= 5) {
          
         lastframe_time = millis();
         //copy the gps coordinates to variables used for calculations
         GPS_latitude = i2c_dataset.gps_loc.lat;
         GPS_longitude = i2c_dataset.gps_loc.lon;

         // It's just for safety since home position must be set from the host
         if (GPS_fix_home == 0) {
            GPS_fix_home = 1;
            i2c_dataset.gps_wp[0].position.lat = GPS_latitude;
            i2c_dataset.gps_wp[0].position.lon = GPS_longitude;
            GPS_calc_longitude_scaling(GPS_latitude);  //need an initial value for distance and bearing calc
         }
         //dTnav calculation
         //Time for calculating x,y speed and navigation pids
		 dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
		 nav_loopTimer = millis();
         // prevent runup from bad GPS
		 dTnav = min(dTnav, 1.0);  

         _watchdog_timer = millis();  //Reset watchdog timer
          
         //calculate distance and bearings for gui and other stuff continously this is independent from navigation
         i2c_dataset.distance_to_home = GPS_distance_cm(GPS_latitude,GPS_longitude,i2c_dataset.gps_wp[0].position.lat,i2c_dataset.gps_wp[0].position.lon);
         i2c_dataset.home_to_copter_bearing = GPS_bearing(i2c_dataset.gps_wp[0].position.lat,i2c_dataset.gps_wp[0].position.lon,GPS_latitude,GPS_longitude);
         //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
         GPS_calc_velocity(GPS_latitude,GPS_longitude);        
          
         if (GPSMode != 0){    //ok we are navigating 
            //do gps nav calculations here   

#if defined(GPS_LEAD_FILTER)
             wp_distance = GPS_distance_cm(GPS_lead_latitude,GPS_lead_longitude,GPS_WP_latitude,GPS_WP_longitude);
             target_bearing = GPS_bearing(GPS_lead_latitude,GPS_lead_longitude,GPS_WP_latitude,GPS_WP_longitude);
             GPS_calc_location_error(GPS_WP_latitude,GPS_WP_longitude,GPS_lead_latitude,GPS_lead_longitude);
#else 
             wp_distance = GPS_distance_cm(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
             target_bearing = GPS_bearing(GPS_latitude,GPS_longitude,GPS_WP_latitude,GPS_WP_longitude);
             GPS_calc_location_error(GPS_WP_latitude,GPS_WP_longitude,GPS_latitude,GPS_longitude);
#endif

             switch (nav_mode) {
              case NAV_MODE_POSHOLD: 
                //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
	        GPS_calc_poshold(long_error, lat_error);
                break;
               case NAV_MODE_WP:
		int16_t speed = GPS_calc_desired_speed(i2c_dataset.nav_speed_max, true);      //slow navigation 
		// use error as the desired rate towards the target
		GPS_calc_nav_rate(speed);

                // Are we there yet ?(within 2 meters of the destination)
	        if ((wp_distance <= i2c_dataset.wp_radius) || check_missed_wp()){         //if yes switch to poshold mode
                     nav_mode = NAV_MODE_POSHOLD;
                     //set reached flag
                     i2c_dataset.status.wp_reached = 1;
                   } 
               break;               
              } //switch nav mode
           } // if GPSmode!=0  
           // update i2c dataset from nav 
           GPS_update_i2c_dataset();
        } else {      // we does not have 3d fix or numsats less than 5 , stop navigation
                 nav_lat = 0;
                 nav_lon = 0;
                 GPSMode = GPSMODE_NONAV;
                 nav_mode = NAV_MODE_NONE;
                 wp_distance = 0;
                 i2c_dataset.distance_to_home = 0;
                 i2c_dataset.home_to_copter_bearing = 0;
                 GPS_update_i2c_dataset();
               }
        // have new data at this point anyway
       i2c_dataset.status.new_data = 1;

      } // new frame
     } //while 
#pragma endregion

blink_sonar_update();   


//check watchdog timer, after 1200ms without valid packet, assume that gps communication is lost.
if (_watchdog_timer != 0)
{  
  if (_watchdog_timer+1200 < millis()) 
     {
       i2c_dataset.status.gps2dfix = 0;
       i2c_dataset.status.gps3dfix = 0;
       i2c_dataset.status.numsats = 0;
       i2c_dataset.gps_loc.lat = 0;
       i2c_dataset.gps_loc.lon = 0;
       nav_lat = 0;
       nav_lon = 0;
       GPS_update_i2c_dataset();
       _watchdog_timer = 0;
       i2c_dataset.status.new_data = 1;
     }
}

  //Check for new incoming command on I2C
  if (new_command!=0) {
    _command = new_command;                                                   //save command byte for processing
    new_command = 0;                                                          //clear it

    _command_wp = (_command & 0xF0) >> 4;                                     //mask 4 MSB bits and shift down
    _command = _command & 0x0F;                                               //empty 4MSB bits

   switch (_command) {
     case I2C_GPS_COMMAND_POSHOLD:
          GPS_set_next_wp(16);                                                //wp16 is a virtual one, means current location
          GPSMode = GPSMODE_HOLD;
          nav_mode = NAV_MODE_POSHOLD;
          i2c_dataset.status.new_data = 0;                                    //invalidate current dataset
     break;         
     case I2C_GPS_COMMAND_START_NAV:
          GPS_set_next_wp(_command_wp);
          GPSMode = GPSMODE_WP;
          nav_mode = NAV_MODE_WP;
          i2c_dataset.status.new_data = 0;                                    //invalidate current dataset
      break;          
      case I2C_GPS_COMMAND_SET_WP:
          i2c_dataset.gps_wp[_command_wp].position.lat = GPS_latitude;
          i2c_dataset.gps_wp[_command_wp].position.lon = GPS_longitude;
      break;
      case I2C_GPS_COMMAND_UPDATE_PIDS:
          pi_poshold_lat.kP((float)i2c_dataset.poshold_p/100.0f);
          pi_poshold_lon.kP((float)i2c_dataset.poshold_p/100.0f);
          pi_poshold_lat.kI((float)i2c_dataset.poshold_i/100.0f);
          pi_poshold_lon.kI((float)i2c_dataset.poshold_i/100.0f);
          pi_poshold_lat.imax(i2c_dataset.poshold_imax*100);
          pi_poshold_lon.imax(i2c_dataset.poshold_imax*100);
      
          pid_poshold_rate_lat.kP((float)i2c_dataset.poshold_rate_p/10.0f);
          pid_poshold_rate_lon.kP((float)i2c_dataset.poshold_rate_p/10.0f);
          pid_poshold_rate_lat.kI((float)i2c_dataset.poshold_rate_i/100.0f);
          pid_poshold_rate_lon.kI((float)i2c_dataset.poshold_rate_i/100.0f);
          pid_poshold_rate_lat.kD((float)i2c_dataset.poshold_rate_d/1000.0f);
          pid_poshold_rate_lon.kD((float)i2c_dataset.poshold_rate_d/1000.0f);
          pid_poshold_rate_lat.imax(i2c_dataset.poshold_rate_imax*100);
          pid_poshold_rate_lon.imax(i2c_dataset.poshold_rate_imax*100);
    
          pid_nav_lat.kP((float)i2c_dataset.nav_p/10.0f);
          pid_nav_lon.kP((float)i2c_dataset.nav_p/10.0f);
          pid_nav_lat.kI((float)i2c_dataset.nav_i/100.0f);
          pid_nav_lon.kI((float)i2c_dataset.nav_i/100.0f);
          pid_nav_lat.kD((float)i2c_dataset.nav_d/1000.0f);
          pid_nav_lon.kD((float)i2c_dataset.nav_d/1000.0f);
          pid_nav_lat.imax(i2c_dataset.nav_imax*100);
          pid_nav_lon.imax(i2c_dataset.nav_imax*100);
       break;  
      case I2C_GPS_COMMAND_STOP_NAV:
          GPS_reset_nav();
          GPSMode = GPSMODE_NONAV;
          nav_mode = NAV_MODE_NONE;
          GPS_update_i2c_dataset();
          i2c_dataset.status.new_data = 1;
      break;
     
   } //switch  
  }
}



