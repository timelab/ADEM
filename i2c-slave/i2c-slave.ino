/*******************************************************************************************************************************
 *
 * i2C-gps is an adapted version of I2CGPS see below
 *
 * It has been adapted to accomodate additional functionality to support low power mode in the future
 *
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

// Software version for cross checking
#define VERSION 33

//#include "config.h"
#include <SoftwareSerial.h>
#include "WireMW.h"
//Register definitions
#include "i2c_gps_registers.h"

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define I2C_ADDRESS 0x20                      //7 bit address 0x40 write, 0x41 read

// Serial speed of the GPS
#define GPS_SERIAL_SPEED 9600

// Used pins for GPS
#define GPS_RX_PIN 5
#define GPS_TX_PIN 6

/* GPS protocol
 * NMEA         - Standard NMEA protocol GGA, GSA and RMC  sentences are needed
 * UBLOX        - U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt) from the source tree·
 * MTK_BINARY16 - MTK binary protocol (DIYDrones v1.6)·
 * MTK_BINARY19 - MTK binary protocol (DIYDrones v1.9)·
 * MTK_INIT     - Initialize MTK GPS (if MTK_BINARY16 or 19 is not defined then it goes to NMEA, otherwise it goes for binary)
 * With MTK and UBLOX you don't have to use GPS_FILTERING in multiwii code !!!
 */

#define NMEA
//#define UBLOX
//#define MTK_BINARY16
//#define MTK_BINARY19
//#define INIT_MTK_GPS

// Size of register map
#define REG_MAP_SIZE sizeof(i2c_dataset)
// Maximum amount of data that I could receive from a master device (register, plus 11 byte waypoint data)
#define MAX_SENT_BYTES 0x0C

#define LAT  0
#define LON  1

// Blink feedback, by guru_florida
#define BLINK_INTERVAL 90

#define DEBUG_I2CGPS 1

#ifdef DEBUG_I2CGPS
#define SERIAL_BAUD 38400
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

#ifdef INIT_MTK_GPS

 #define MTK_SET_BINARY          "$PGCMD,16,0,0,0,0,0*6A\r\n"
 #define MTK_SET_NMEA            "$PGCMD,16,1,1,1,1,1*6B\r\n"
 #define MTK_SET_NMEA_SENTENCES  "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
 #define MTK_OUTPUT_4HZ          "$PMTK220,250*29\r\n"
 #define MTK_OUTPUT_5HZ          "$PMTK220,200*2C\r\n"
 #define MTK_OUTPUT_10HZ         "$PMTK220,100*2F\r\n"
 #define MTK_NAVTHRES_OFF        "$PMTK397,0*23\r\n" // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s
 #define SBAS_ON                 "$PMTK313,1*2E\r\n"
 #define WAAS_ON                 "$PMTK301,2*2E\r\n"
 #define SBAS_TEST_MODE          "$PMTK319,0*25\r\n" // Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)

#endif

uint32_t init_speed[5] = { 9600, 19200, 38400, 57600, 115200 };

#ifdef UBLOX
// PROGMEM array must be outside any function !!!
const char UBLOX_INIT[] PROGMEM = {
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19, // Disable all default NMEA messages
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47, // Set POSLLH MSG rate
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49, // Set STATUS MSG rate
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F, // Set SOL MSG rate
  0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67, // Set VELNED MSG rate
  0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41, // Set WAAS to EGNOS
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A              // Set rate to 5Hz
};
#endif


static I2C_REGISTERS i2c_dataset;

static uint8_t receivedCommands[MAX_SENT_BYTES];
// New command received (!=0)
static uint8_t new_command;

static SoftwareSerial *swSerial = new SoftwareSerial(GPS_RX_PIN, GPS_TX_PIN, false);

// Blink code variables
static uint32_t lastframe_time = 0;
static uint32_t _statusled_timer = 0;
static int8_t _statusled_blinks = 0;
static boolean _statusled_state = 0;

static int32_t GPS_read[2];

enum frame_t {
  NO_FRAME,
  GPGGA_FRAME,
  GPGSA_FRAME,
  GPRMC_FRAME,
  GPZDA_FRAME,
};

#ifdef NMEA

/* The latitude or longitude is coded this way in NMEA frames
  dddmm.mmmm   coded as degrees + minutes + minute decimal
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000
  I increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
*/

#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;

  // Scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) {
    ;
  }
  q = s;

  // Convert degrees
  while ((p - q) > 2) {
    if (deg) {
      deg *= 10;
    }
    deg += DIGIT_TO_VAL(*q++);
  }
  // Convert minutes
  while (p > q) {
    if (min) {
      min *= 10;
    }
    min += DIGIT_TO_VAL(*q++);
  }
  // Convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (int i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q)) {
        frac_min += *q++ - '0';
      }
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
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

bool GPS_NMEA_newFrame(char c) {

  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, gps_frame = NO_FRAME;

//  __LOG(c);
  switch (c) {

    case '$':
      param = 0;
      offset = 0;
      parity = 0;
      break;

    case ',':
    case '*':
      string[offset] = 0;
      if (param == 0) {

        // Frame identification
        if (strncmp(string, "GPGGA", 5) == 0) {
          gps_frame = GPGGA_FRAME;
        } else if (strncmp(string, "GPGSA", 5) == 0) {
          gps_frame = GPGSA_FRAME;
        } else if (strncmp(string, "GPRMC", 5) == 0) {
          gps_frame = GPRMC_FRAME;
        } else if (strncmp(string, "GPZDA", 5) == 0) {
          gps_frame = GPZDA_FRAME;
        } else {
          gps_frame = NO_FRAME;
        }
      }

      switch (gps_frame) {

        // GPGGA FRAME parsing
        case GPGGA_FRAME:
          switch (param) {
            case 1:
              // Up to .000 s precision not needed really but the data is there anyway
              i2c_dataset.time = (atof(string)*1000);
              break;

            //case 2: i2c_dataset.gps_loc.lat = GPS_coord_to_degrees(string);
            case 2:
              GPS_read[LAT] = GPS_coord_to_degrees(string);
              break;

            //case 3: if (string[0] == 'S') i2c_dataset.gps_loc.lat = -i2c_dataset.gps_loc.lat;
            case 3:
              if (string[0] == 'S') {
                GPS_read[LAT] = -GPS_read[LAT];
              }
              break;

            //case 4: i2c_dataset.gps_loc.lon = GPS_coord_to_degrees(string);
            case 4:
              GPS_read[LON] = GPS_coord_to_degrees(string);
              break;

            //case 5: if (string[0] == 'W') i2c_dataset.gps_loc.lon = -i2c_dataset.gps_loc.lon;
            case 5:
              if (string[0] == 'W') {
                GPS_read[LON] = -GPS_read[LON];
              }
              break;

            case 6:
              i2c_dataset.status.gps2dfix = string[0]  > '0';
              break;

            case 7:
              i2c_dataset.status.numsats = atoi(string);
              break;

            case 9:
              i2c_dataset.altitude = atoi(string);
              break;
          } // switch
          break;

          // GPGSA FRAME parsing
          case GPGSA_FRAME:
            switch (param) {
              case 2:
                i2c_dataset.status.gps3dfix = string[0] == '3';
                break;
            } // switch
            break;

          // GPGSA FRAME parsing
          //
          // eg2. $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
          //
          //                225446       Time of fix 22:54:46 UTC
          //                A            Navigation receiver warning A = Valid position, V = Warning
          //                4916.45,N    Latitude 49 deg. 16.45 min. North
          //                12311.12,W   Longitude 123 deg. 11.12 min. West
          //                000.5        Speed over ground, Knots
          //                054.7        Course Made Good, degrees true
          //                191194       UTC Date of fix, 19 November 1994
          //                020.3,E      Magnetic variation, 20.3 deg. East
          //                *68          mandatory checksum
          //
          // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,m*hh
          //     Field #
          //     1    = UTC time of fix
          //     2    = Data status (A=Valid position, V=navigation receiver warning)
          //     3    = Latitude of fix
          //     4    = N or S of longitude
          //     5    = Longitude of fix
          //     6    = E or W of longitude
          //     7    = Speed over ground in knots
          //     8    = Track made good in degrees True
          //     9    = UTC date of fix
          //     10   = Magnetic variation degrees (Easterly var. subtracts from true course)
          //     11   = E or W of magnetic variation
          //     12   = Mode indicator, (A=Autonomous, D=Differential, E=Estimated, N=Data not valid)
          //     13   = Checksum
          case GPRMC_FRAME:
            switch(param) {
              case 7:
                // Convert to m/s*100
                i2c_dataset.ground_speed = (atof(string) * 0.5144444) * 10;
                break;

              case 8:
                // Convert to degrees *10 (.1 precision)
                i2c_dataset.ground_course = (atof(string) * 10);
                break;

              case 9:
                long tmp = atol(string);
                i2c_dataset.day = tmp / 10000;
                i2c_dataset.month = ( tmp / 100 ) % 100;
                i2c_dataset.year = ( tmp % 100 );
                break;

            }
            break;

          // GPZDA FRAME parsing
          case GPZDA_FRAME:
            switch (param) {
              case 1:
                // Up to .000 s precision not needed really but the data is there anyway
                i2c_dataset.time = (atof(string) * 1000);
                break;

              case 2:
                // Day as number
                i2c_dataset.day = atoi(string);
                break;

              case 3:
                // Month as number
                i2c_dataset.month = atoi(string);
                break;

              case 4:
                // Year as number
                i2c_dataset.year = atoi(string);
                break;
            } // switch
            break;

        } // switch

        param++;
        offset = 0;
        if (c == '*') {
          checksum_param=1;
        } else {
          parity ^= c;
        }
        break;

     case '\r':
     case '\n':
       if (checksum_param) {
         // Parity checksum
         uint8_t checksum = 16 * ((string[0]>='A') ? string[0] - 'A'+10: string[0] - '0') + ((string[1]>='A') ? string[1] - 'A'+10: string[1]-'0');
         if (checksum == parity) {
           frameOK = 1;
         }
       }
       checksum_param=0;
       break;

     default:
       if (offset < 15) {
         string[offset++] = c;
       }
       if (!checksum_param) {
         parity ^= c;
       }
  } // switch
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
  uint32_t time;                // GPS msToW
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
};

struct ubx_nav_status {
  uint32_t time;                // GPS msToW
  uint8_t fix_type;
  uint8_t fix_status;
  uint8_t differential_status;
  uint8_t res;
  uint32_t time_to_first_fix;
  uint32_t uptime;              // in milliseconds
};

struct ubx_nav_solution {
  uint32_t time;
  int32_t time_nsec;
  int16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};

struct ubx_nav_velned {
  uint32_t time;                // GPS msToW
  int32_t ned_north;
  int32_t ned_east;
  int32_t ned_down;
  uint32_t speed_3d;
  uint32_t speed_2d;
  int32_t heading_2d;
  uint32_t speed_accuracy;
  uint32_t heading_accuracy;
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
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

static bool next_fix;

static uint8_t _class;

// Do we have new position information?
static bool _new_position;

// Do we have new speed information?
static bool _new_speed;

static uint8_t _disable_counter;

// Receive buffer
static union {
  ubx_nav_posllh posllh;
  ubx_nav_status status;
  ubx_nav_solution solution;
  ubx_nav_velned velned;
  uint8_t bytes[];
} _buffer;

void _update_checksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b) {
  while (len--) {
    ck_a += *data;
    ck_b += ck_a;
    data++;
  }
}

bool GPS_UBLOX_newFrame(uint8_t data) {
  bool parsed = false;

  switch(_step) {

    case 1:
      if (PREAMBLE2 == data) {
        _step++;
        break;
      }
      _step = 0;

    case 0:
      if (PREAMBLE1 == data) {
        _step++;
      }
      break;

    case 2:
      _step++;
      _class = data;
      // Reset the checksum accumulators
      _ck_b = _ck_a = data;
      break;

    case 3:
      _step++;
      // Checksum byte
      _ck_b += (_ck_a += data);
      _msg_id = data;
      break;

    case 4:
      _step++;
      // Checksum byte
      _ck_b += (_ck_a += data);
      // Payload length low byte
      _payload_length = data;
      break;

    case 5:
      _step++;
      // Checksum byte
      _ck_b += (_ck_a += data);

      _payload_length += (uint16_t) (data<<8);
      if (_payload_length > 512) {
        _payload_length = 0;
        _step = 0;
      }
      // Prepare to receive payload
      _payload_counter = 0;
      break;

    case 6:
      // Checksum byte
      _ck_b += (_ck_a += data);
      if (_payload_counter < sizeof(_buffer)) {
        _buffer.bytes[_payload_counter] = data;
      }
      if (++_payload_counter == _payload_length) {
        _step++;
      }
      break;

    case 7:
      _step++;
      if (_ck_a != data) {
        // Bad checksum
        _step = 0;
      }
      break;

    case 8:
      _step = 0;
      if (_ck_b != data) {
        // Bad checksum
        break;
      }
      if (UBLOX_parse_gps())  {
        parsed = true;
      }
  } // end switch

  return parsed;
}

bool UBLOX_parse_gps(void) {

  switch (_msg_id) {
    case MSG_POSLLH:
      i2c_dataset.time = _buffer.posllh.time;
      GPS_read[LON] = _buffer.posllh.longitude;
      GPS_read[LAT] = _buffer.posllh.latitude;
      // Alt in m
      i2c_dataset.altitude = _buffer.posllh.altitude_msl / 10 /100;
      i2c_dataset.status.gps3dfix = next_fix;
      _new_position = true;
      break;

    case MSG_STATUS:
      next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
      if (!next_fix) {
        i2c_dataset.status.gps3dfix = false;
      }
      break;

    case MSG_SOL:
      next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
      if (!next_fix) {
        i2c_dataset.status.gps3dfix = false;
      }
      i2c_dataset.status.numsats = _buffer.solution.satellites;
      //GPS_hdop = _buffer.solution.position_DOP;
      //debug[3] = GPS_hdop;

    case MSG_VELNED:
      // Speed in cm/s
      //speed_3d = _buffer.velned.speed_3d;
      // Speed in cm/s
      i2c_dataset.ground_speed = _buffer.velned.speed_2d;
      // Heading 2D deg * 100000 rescaled to deg * 10
      i2c_dataset.ground_course = (uint16_t) (_buffer.velned.heading_2d / 10000);
      _new_speed = true;
      break;

    default:
      return false;

  }

  // We only return true when we get new position and speed data
  // This ensures we don't use stale data
  if (_new_position && _new_speed) {
    _new_speed = _new_position = false;
    return true;
  }

  return false;
}
#endif //UBLOX

#if defined(MTK_BINARY16) || defined(MTK_BINARY19)
struct diyd_mtk_msg {
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  int32_t ground_speed;
  int32_t ground_course;
  uint8_t satellites;
  uint8_t fix_type;
  uint32_t utc_date;
  uint32_t utc_time;
  uint16_t hdop;
};

// #pragma pack(pop)
enum diyd_mtk_fix_type {
  FIX_NONE = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  FIX_2D_SBAS = 6,
  FIX_3D_SBAS = 7
};

#ifdef MTK_BINARY16
enum diyd_mtk_protocol_bytes {
  PREAMBLE1 = 0xd0,
  PREAMBLE2 = 0xdd,
};
#endif

#ifdef MTK_BINARY19
enum diyd_mtk_protocol_bytes {
  PREAMBLE1 = 0xd1,
  PREAMBLE2 = 0xdd,
};
#endif

// Packet checksum accumulators
uint8_t _ck_a;
uint8_t _ck_b;

// State machine state
uint8_t _step;
uint8_t _payload_counter;

// Time from UNIX Epoch offset
long _time_offset;
bool _offset_calculated;

// Receive buffer
union {
  diyd_mtk_msg msg;
  uint8_t bytes[];
} _buffer;

inline long _swapl(const void *bytes) {
  const uint8_t *b = (const uint8_t *) bytes;
  union {
    long v;
    uint8_t b[4];
  } u;

  u.b[0] = b[3];
  u.b[1] = b[2];
  u.b[2] = b[1];
  u.b[3] = b[0];

  return(u.v);
}

bool GPS_MTK_newFrame(uint8_t data) {
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
    case 0:
      if (PREAMBLE1 == data)
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
        // Reset the checksum accumulators
        _ck_b = _ck_a = data;
        _payload_counter = 0;
      } else {
        // Reset and wait for a message of the right class
        _step = 0;
        goto restart;
      }
      break;

    // Receive message data
    case 3:
      _buffer.bytes[_payload_counter++] = data;
      _ck_b += (_ck_a += data);
      if (_payload_counter == sizeof(_buffer)) {
        _step++;
      }
      break;

    // Checksum and message processing
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

      i2c_dataset.status.gps3dfix  = ((_buffer.msg.fix_type == FIX_3D) || (_buffer.msg.fix_type == FIX_3D_SBAS));
#ifdef MTK_BINARY16
      // XXX: Doc says *10e7 but device says otherwise
      GPS_read[LAT] = _buffer.msg.latitude * 10;
      // XXX: Doc says *10e7 but device says otherwise
      GPS_read[LON] = _buffer.msg.longitude * 10;
#endif
#ifdef MTK_BINARY19
      // XXX: Doc says *10e7 but device says otherwise
      GPS_read[LAT] = _buffer.msg.latitude;
      // XXX: Doc says *10e7 but device says otherwise
      GPS_read[LON] = _buffer.msg.longitude;
#endif
      i2c_dataset.altitude = _buffer.msg.altitude / 100;
      i2c_dataset.ground_speed = _buffer.msg.ground_speed;
      i2c_dataset.ground_course = _buffer.msg.ground_course;
      i2c_dataset.status.numsats = _buffer.msg.satellites;
      //GPS_hdop = _buffer.msg.hdop;
      parsed = true;
      //GPS_Present = 1;
  } // end switch
  return parsed;
}
#endif //MTK


// I2C handlers
//
// Handler for requesting data

// Read data from I2C_RAG_MAP and send back
void requestEvent() {
  if (receivedCommands[0] >= I2C_GPS_LST_RECEIVE) {
    // Accessing gps data, switch new_data_flag;
    i2c_dataset.status.new_data = 0;
  }

  // TODO: Shouldn't we limit the data written to sizeof(i2c_dataset) - receivedCommands[0] ?
  //
  __LOG("Send data: "); __LOGLN(receivedCommands[0]);

  // Write data from the requested data register position
  int8_t* ptr = 0;
  ptr = (int8_t *) &i2c_dataset;

//  for (int i = 0; i <= 32; i++) {
//    int8_t  j = ptr[i];
//    __LOG("SD "); __LOG(i); __LOG(":"); __LOGLN((char) j, HEX);
//  }
//  __LOGLN();

  // Write up to 32 byte, since master is responsible for reading and sending NACK
  Wire.write((uint8_t *) &i2c_dataset+receivedCommands[0], 32-receivedCommands[0]);

// 32 byte limit is in the Wire library, we have to live with it unless writing our own wire library
}


// Handler for receiving data

// When first byte is less than the REG_MAP_SIZE we just read the I2C_REG_MAP register
// Else it is a command we have to execute
void receiveEvent(int bytesReceived) {
  // Read all received bytes and store up to MAX_SENT_BYTES in receivedCommands
  uint8_t  *ptr;
  __LOG("Received #bytes: "); __LOGLN(bytesReceived);
  for (int a = 0; a < bytesReceived; a++) {
    if (a < MAX_SENT_BYTES) {
      receivedCommands[a] = Wire.read();
    } else {
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }
  __LOG("receivedCommands[0]"); __LOGLN((int)receivedCommands[0]);

  // If the first byte received is I2C_GPS_COMMAND then the next byte is the actual command
  if (receivedCommands[0] == I2C_GPS_COMMAND) {
    // Just one byte, ignore all others
    new_command = receivedCommands[1];
    return;
  }

  // If we just received 1 bytes then this is a read command and the first byte is the offset in the I2C_REG_MAP
  if (bytesReceived == 1) {
    if (receivedCommands[0] < REG_MAP_SIZE) {
      // Read command from a given register
      return;
    } else {
      // Addressing over the reg_map fallback to first byte
      receivedCommands[0] = 0x00;
      return;
    }
  }
  __LOG("Received byte : "); __LOGLN(receivedCommands[0]);

  // More than 1 byte was received, so there is definitely some data to write into a register
  // Check for writeable registers and discard data is it's not writeable
}


void blink_update() {

  uint32_t now = millis();

  if (_statusled_timer < now) {
    if (lastframe_time + 5000 < now) {
      // No GPS communication

      _statusled_state = !_statusled_state;
      // Set the LED off
      digitalWrite(13, _statusled_state ? HIGH : LOW);
      _statusled_timer = now + 1000;
      return;
    }

    if (_statusled_blinks == 0) {
      if (i2c_dataset.status.gps3dfix == 1) {
        _statusled_blinks = 3;
      } else if(i2c_dataset.status.gps2dfix == 1) {
         _statusled_blinks = 2;
      } else {
        _statusled_blinks = 1;
      }
    }

    if (_statusled_state) {
      _statusled_blinks--;
      _statusled_state = false;
      _statusled_timer = now + ((_statusled_blinks>0) ? BLINK_INTERVAL : 1000);
      // Set the LED off
      digitalWrite(13, LOW);
    } else {
      _statusled_state = true;
      _statusled_timer = now + BLINK_INTERVAL;
      // Set the LED on
      digitalWrite(13, HIGH);
    }
  }
}

void GPS_SerialInit() {

  __LOG("Initializing GPS at speed "); __LOG(GPS_SERIAL_SPEED); __LOG("...");
  swSerial->begin(GPS_SERIAL_SPEED);
  __LOGLN(" OK");
  delay(1000);

#ifdef UBLOX
  //Set speed
  __LOG("Configure UBLOX speed...");
  for ( uint8_t i=0; i < 5; i++ ) {
    // switch UART speed for sending SET BAUDRATE command (NMEA mode)
    swSerial->begin(init_speed[i]);
    #if (GPS_SERIAL_SPEED==19200)
      // 19200 baud - minimal speed for 5Hz update rate
      swSerial->write(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n"));
    #endif
    #if (GPS_SERIAL_SPEED==38400)
      // 38400 baud
      swSerial->write(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n"));
    #endif
    #if (GPS_SERIAL_SPEED==57600)
      // 57600 baud
      swSerial->write(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n"));
    #endif
    #if (GPS_SERIAL_SPEED==115200)
      // 115200 baud
      swSerial->write(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n"));
    #endif
    // Wait for init
    __LOGLN(" OK");
    delay(300);
  }
  delay(200);

  __LOG("Initialize UBLOX...");
  swSerial->begin(GPS_SERIAL_SPEED);
  // Send configuration data in UBX protocol
  for ( uint8_t i=0; i < sizeof(UBLOX_INIT); i++ ) {
    swSerial->write(pgm_read_byte(UBLOX_INIT+i));
    // Simulating a 38400baud pace (or less), otherwise commands are not accepted by the device
//    delay(5);
  }
  __LOGLN(" OK");

// MTK GPS setup
#elif defined(INIT_MTK_GPS)
  __LOG("Configure MTK speed...");
  for (uint8_t i=0; i < 5; i++) {
    // Switch UART speed for sending SET BAUDRATE command
    swSerial->begin(init_speed[i]);
    #if (GPS_SERIAL_SPEED==19200)
      // 19200 baud - minimal speed for 5Hz update rate
      swSerial->write("$PMTK251,19200*22\r\n");
    #endif
    #if (GPS_SERIAL_SPEED==38400)
      // 38400 baud
      swSerial->write("$PMTK251,38400*27\r\n");
    #endif
    #if (GPS_SERIAL_SPEED==57600)
      // 57600 baud
      swSerial->write("$PMTK251,57600*2C\r\n");
    #endif
    #if (GPS_SERIAL_SPEED==115200)
      // 115200 baud
      swSerial->write("$PMTK251,115200*1F\r\n");
    #endif
    delay(300);
  }

  // At this point we have GPS working at selected (via #define GPS_BAUD) baudrate
  swSerial->begin(GPS_SERIAL_SPEED);
  __LOGLN(" OK");

  __LOG("Initialize MTK...");
  swSerial->write(MTK_NAVTHRES_OFF);
  delay(100);
  swSerial->write(SBAS_ON);
  delay(100);
  swSerial->write(WAAS_ON);
  delay(100);
  swSerial->write(SBAS_TEST_MODE);
  delay(100);
  // 5 Hz update rate
  swSerial->write(MTK_OUTPUT_5HZ);
  delay(100);

  #ifdef NMEA
    // Only GGA and RMC sentence
    swSerial->write(MTK_SET_NMEA_SENTENCES);
  #endif

  #if defined(MTK_BINARY19) || defined(MTK_BINARY16)
    swSerial->write(MTK_SET_BINARY);
  #endif
  __LOGLN(" OK");
#endif
}


// Setup
void setup() {

#ifdef DEBUG_I2CGPS
  Serial.begin(SERIAL_BAUD);
#endif

  __LOGLN("Serial initialized.");

  // Initialize GPS
  GPS_SerialInit();

  // Initialize i2c_dataset;
  uint8_t *ptr = (uint8_t *) &i2c_dataset;
  for ( uint8_t i=0; i < sizeof(i2c_dataset); i++) {
    *ptr = 0;
    ptr++;
  }

  // Set up default parameters
  i2c_dataset.sw_version = VERSION;

  // Start I2C communication routines
  __LOG("Initializing I2C...");
  // DO NOT FORGET TO COMPILE WITH 400KHz!!! else change TWBR Speed to 100khz on Host !!!
  // Address 0x40 write 0x41 read
  Wire.begin(I2C_ADDRESS);
  __LOGLN(" OK");

  // Set up event handlers
  __LOG("Set up I2C handlers..");
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  __LOGLN(" OK");

}


// Main loop
void loop() {

  static uint8_t GPS_fix_home;
  static uint8_t _command_wp;
  static uint8_t _command;
  static uint32_t _watchdog_timer = 0;
  uint8_t axis;
  uint16_t fraction3[2];

//#pragma region while serial available

  while (swSerial->available()) {
    i2c_dataset.last_receive = millis();

  #ifdef NMEA
    boolean available_data = GPS_NMEA_newFrame(swSerial->read());
  #endif
  #ifdef UBLOX
    boolean available_data = GPS_UBLOX_newFrame(swSerial->read());
  #endif
  #if defined(MTK_BINARY16) || defined(MTK_BINARY19)
    boolean available_data = GPS_MTK_newFrame(swSerial->read());
  #endif

    if (available_data) {
      // We have a valid GGA frame and we have lat and lon in GPS_read_lat and GPS_read_lon, apply moving average filter
      // this is a little bit tricky since the 1e7/deg precision easily overflow a long, so we apply the filter to the fractions
      // only, and strip the full degrees part. This means that we have to disable the filter if we are very close to a degree line
      i2c_dataset.gps_loc.lat = GPS_read[LAT];
      i2c_dataset.gps_loc.lon = GPS_read[LON];
      if (i2c_dataset.gps_loc.lat != 0 && i2c_dataset.gps_loc.lon != 0) {
        __LOG("LAT: "); __LOG(i2c_dataset.gps_loc.lat);
        __LOG(" LON: "); __LOGLN(i2c_dataset.gps_loc.lon);
      }

      if (i2c_dataset.status.gps3dfix == 1 && i2c_dataset.status.numsats >= 5) {
        lastframe_time = i2c_dataset.last_receive;
        // Reset watchdog timer
        _watchdog_timer = i2c_dataset.last_receive;
      }
      // Have new data at this point anyway
      i2c_dataset.status.new_data = 1;

    } // new frame
  } // while
//#pragma endregion

  blink_update();

  // Check watchdog timer, after 1200ms without valid packet, assume that gps communication is lost.
  if (_watchdog_timer != 0) {
    if (_watchdog_timer+1200 < millis()) {
      i2c_dataset.status.gps2dfix = 0;
      i2c_dataset.status.gps3dfix = 0;
      i2c_dataset.status.numsats = 0;
      i2c_dataset.gps_loc.lat = 0;
      i2c_dataset.gps_loc.lon = 0;
       _watchdog_timer = 0;
      i2c_dataset.status.new_data = 1;
    }
  }

  // Check for new incoming command on I2C
  if (new_command != 0) {
    // Save command byte for processing
    _command = new_command;
    // Clear it
    new_command = 0;

    // Empty 4MSB bits
    _command = _command & 0x0F;

    switch (_command) {
      // Set update frequency
      case I2C_GPS_COMMAND__1:
        // Invalidate current dataset
        i2c_dataset.status.new_data = 0;
        break;

      // Goto low power mode
      case I2C_GPS_COMMAND__2:
        // Invalidate current dataset
        i2c_dataset.status.new_data = 0;
        break;
    } // switch
  }
}
