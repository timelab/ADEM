//////////////////////////////////////////////////////////////////////////////
// i2C comm definitions
//
#define I2C_ADDRESS        0x20                      //7 bit address 0x40 write, 0x41 read

/* Serial speed of the GPS */
#define GPS_SERIAL_SPEED 9600
//#define NEOPIXEL_FEEDBACK

#define GPS_RX_PIN 6
#define GPS_TX_PIN 5

/* GPS protocol 
 * NMEA			- Standard NMEA protocol GGA, GSA and RMC  sentences are needed
 * UBLOX		- U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt) from the source tree 
 * MTK_BINARY16 - MTK binary protocol (DIYDrones v1.6) 
 * MTK_BINARY19 - MTK binary protocol (DIYDrones v1.9) 
 * MTK_INIT     - Initialize MTK GPS (if MTK_BINARY16 or 19 is not defined then it goes to NMEA, otherwise it goes for binary)
 * With MTK and UBLOX you don't have to use GPS_FILTERING in multiwii code !!!
 *
 */

#define NMEA
//#define UBLOX
//#define MTK_BINARY16
//#define MTK_BINARY19
//#define INIT_MTK_GPS
