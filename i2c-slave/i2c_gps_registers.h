///////////////////////////////////////////////////////////////////////////////////////////////////
// I2C GPS NAV registers
///////////////////////////////////////////////////////////////////////////////////////////////////
//
#define I2C_GPS_STATUS_00                            00 //(Read only)
#define I2C_GPS_STATUS_NEW_DATA       0x01      // New data is available (after every GGA frame)
#define I2C_GPS_STATUS_2DFIX          0x02      // 2dfix achieved
#define I2C_GPS_STATUS_3DFIX          0x04      // 3dfix achieved
#define I2C_GPS_STATUS_WP_REACHED     0x08      // Active waypoint has been reached (not cleared until new waypoint is set)
#define I2C_GPS_STATUS_NUMSATS        0xF0      // Number of sats in view

#define I2C_GPS_COMMAND                              01 // (write only)
#define I2C_GPS_COMMAND__1            0x01      
#define I2C_GPS_COMMAND__2            0x02      
#define I2C_GPS_COMMAND__3            0x03      
#define I2C_GPS_COMMAND__4            0x04      
#define I2C_GPS_COMMAND__5            0x05      
#define I2C_GPS_COMMAND__6            0x06      
#define I2C_GPS_COMMAND__7            0x07
#define I2C_GPS_COMMAND__8            0x08      
#define I2C_GPS_COMMAND__9            0x09
#define I2C_GPS_COMMAND__a            0x0a
#define I2C_GPS_COMMAND__b            0x0b
#define I2C_GPS_COMMAND__c            0x0c
#define I2C_GPS_COMMAND__d            0x0d
#define I2C_GPS_COMMAND__e            0x0e
#define I2C_GPS_COMMAND__f            0x0f


#define I2C_GPS_REG_VERSION                         03   // Version of the I2C_NAV SW uint8_t
#define I2C_GPS_REG_RES2                            04   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES3                            05   // reserved for future use (uint8_t)
#define I2C_GPS_REG_RES4                            06   // reserved for future use (uint8_t)

#define I2C_GPS_DAY                                 07   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_MONTH                               08   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_YEAR                                09   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)

#define I2C_GPS_TIME                                11   // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_LOCATION                            15   // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_LAT                                 15   // 
#define I2C_GPS_LON                                 19   // 
#define I2C_GPS_GROUND_SPEED                        23   // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE                            25   // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE			                  27   // GPS ground course (uint16_t)
#define I2C_GPS_FIX_AGE                             29   // GPS fix age 0.001 sec resolution
#define I2C_GPS_RES1                                31   // reserved for future use (uint16_t)


typedef struct {
	uint8_t    new_data : 1;
	uint8_t    gps2dfix : 1;
	uint8_t    gps3dfix : 1;
	uint8_t    numsats : 4;
} STATUS_REGISTER;

typedef struct {
	long      lat;            //degree*10 000 000
	long      lon;            //degree*10 000 000
} GPS_COORDINATES;

typedef struct {

	//Status and command registers
	STATUS_REGISTER       status;                   // 00  status register
  uint8_t               res1;
  uint8_t               res2;
	uint8_t               sw_version;               // 03  Version of the I2C_GPS sw
	uint8_t               res3;                     // 04  reserved for future use
	uint8_t               res4;                     // 05  reserved for future use
	uint8_t               res5;                     // 06  reserved for future use
	uint8_t               day;                      // 07
	uint8_t               month;                    // 08
	uint16_t              year;                     // 09
	uint32_t              time;                     // 11 UTC Time from GPS

	//GPS & navigation data
	GPS_COORDINATES       gps_loc;                  // 15 current location (8 byte) lat,lon
	int32_t               nav_lat;                  // 15 The desired bank towards North (Positive) or South (Negative)      1 deg = 100 max 30deg (3000)
	int32_t               nav_lon;                  // 19 The desired bank towards East (Positive) or West (Negative)        1 deg = 100 max 30deg (3000)
	uint16_t              ground_speed;             // 23 ground speed from gps m/s*100
	int16_t               altitude;                 // 25 gps altitude
	uint16_t              ground_course;            // 27 GPS ground course
	uint16_t              fix_age;                  // 29 GPS fix age 0.001 sec resolution
	uint16_t              res6;                     // 31 reserved for future use

} I2C_REGISTERS;

///////////////////////////////////////////////////////////////////////////////////////////////////
// End register definition 
///////////////////////////////////////////////////////////////////////////////////////////////////
