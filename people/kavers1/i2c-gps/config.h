//////////////////////////////////////////////////////////////////////////////
// i2C comm definitions
//
#define I2C_ADDRESS        0x20                      //7 bit address 0x40 write, 0x41 read

/* GPS Lead filter - predicts gps position based on the x/y speed. helps overcome the gps lag. */
#define GPS_LEAD_FILTER

/* Serial speed of the GPS */
#define GPS_SERIAL_SPEED 57600

/* GPS protocol
 * NMEA         - Standard NMEA protocol GGA, GSA and RMC  sentences are needed
 * UBLOX        - U-Blox binary protocol, use the ublox config file (u-blox-config.ublox.txt) from the source tree
 * MTK_BINARY16 - MTK binary protocol (DIYDrones v1.6)
 * MTK_BINARY19 - MTK binary protocol (DIYDrones v1.9)
 * MTK_INIT     - Initialize MTK GPS (if MTK_BINARY16 or 19 is not defined then it goes to NMEA, otherwise it goes for binary)
 * With MTK and UBLOX you don't have to use GPS_FILTERING in multiwii code !!!
 *
 */

//#define NMEA
#define UBLOX
 #define UBLOX_CONFIG_ALL_PORTS
//#define MTK_BINARY16
//#define MTK_BINARY19
//#define INIT_MTK_GPS


//////////////////////////////////////////////////////////////////////////////
// Sonar support. Based on code mods from Crazy Al
// Connect Sonar trigger to PC3 (arduino pin A3) and echo output to PC2 (arduino pin A2)
// Supported sonars :
// Ping/Pong type sonar modules such as HC-SR04, SRF-04, DYP-ME007 and many others...
// PWM output continous sonars such as the MAXBOTIX series (connect pin2(PWM out) of the sonar to PC2

#define SONAR                       // USE Sonar
#define SONAR_SONARFULL             300 //full sonar reading before this point (cm)
#define SONAR_BAROFULL              450 //full baro reading after this point (cm)
/* Maximum number of errors to switch to baro (integer 1..20) */
#define SONAR_ERROR_MAX             10

//Sonar type uncomment only one at a time
#define PINGPONG
//#define MAXBOTIX_PWM                // PWM output mode sonar


#define ADNS_5050   5050
#define OPTFLOW     ADNS_5050
//#define OF_ROTATE_I

#define OF_SCLK             4//A2
#define OF_SDIO             3//A1
#define OF_NCS              2//A0 

/* Lense focal distance, mm (set it for your own lense) */
#define OF_FOCAL_DIST 7
/* Deadband for ROLL,PITCH sticks where position hold is enabled. Max value 100 */
#define OF_DEADBAND 15
/* Rotate I-term with heading rotation. It will well compensate wind */
#define OF_ROTATE_I
/* Low-pass filter factor to prevent shaking. Possible values 1..8.  Default is 5. */
#define OF_LPF_FACTOR 5
 
// Default PID variables
//////////////////////////////////////////////////////////////////////////////
// POSHOLD control gains
//
#define POSHOLD_P       .11
#define POSHOLD_I       0.0
#define POSHOLD_IMAX        20              // degrees

#define POSHOLD_RATE_P      1.4         //
#define POSHOLD_RATE_I      0.2         // Wind control
#define POSHOLD_RATE_D      0.010           // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX   20          // degrees
//////////////////////////////////////////////////////////////////////////////
// Navigation PID gains
//
#define NAV_P           1.4             //
#define NAV_I           0.20                // Wind control
#define NAV_D           0.006               //
#define NAV_IMAX        20              // degrees

////////////////////////////////////////////////////////////////////////////////////
// Navigational parameters and limiters initial values
//
#define CROSSTRACK_GAIN            1            // Weighting the cross track error
#define NAV_SPEED_MIN              100          // cm/sec minimum navigational speed when NAV_SLOW_NAV id false
#define NAV_SPEED_MAX              300          // cm/sec maximum navigational speed
#define NAV_BANK_MAX               2500         // 20deg max banking when navigating (just for security and testing)

////////////////////////////////////////////////////////////////////////////////////
// GPS data filtering - moving average filter vector length
//
#define GPS_FILTER_VECTOR_LENGTH 5