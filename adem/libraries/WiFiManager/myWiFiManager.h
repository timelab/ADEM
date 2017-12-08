/**************************************************************
   WiFiManager is a library for the ESP8266/Arduino platform
   (https://github.com/esp8266/Arduino) to enable easy
   configuration and reconfiguration of WiFi credentials using a Captive Portal
   inspired by:
   http://www.esp8266.com/viewtopic.php?f=29&t=2520
   https://github.com/chriscook8/esp-arduino-apboot
   https://github.com/esp8266/Arduino/tree/esp8266/hardware/esp8266com/esp8266/libraries/DNSServer/examples/CaptivePortalAdvanced
   Built by AlexT https://github.com/tzapu
   Licensed under MIT license
 **************************************************************/

#ifndef myWiFiManager_h
#define myWiFiManager_h

#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <memory>
#include <ArduinoJson.h> 

extern "C" {
  #include "user_interface.h"
}
//                                          00000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000111111111122222222223333333333444444444455555555556666666666777777777788888888889999999999000000000011111111112222222222333333333344444444445555555555666666666677777777778888888888999999999900000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000111111111122222222223333333333444444444455555555556666666666777777777788888888889999999999000000000011111111112222222222333333333344444444445555555555666666666677777777778888888888999999999900000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000111111111122222222223333333333444444444455555555556666666666777777777788888888889999999999000000000011111111112222222222333333333344444444445555555555666666666677777777778888888888999999999900000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000111111111122222222223333333333444444444455555555556666666666777777777788888888889999999999000000000011111111112222222222333333333344444444445555555555666666666677777777778888888888999999999900000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000111111111122222222223333333333444444444455555555556666666666777777777788888888889999999999000000000011111111112222222222333333333344444444445555555555666666666677777777778888888888999999999900000000001111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990
//                                          01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
const char HTTP_HEAD[] PROGMEM            = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><title>{v}</title>";
const char HTTP_STYLE[] PROGMEM           = "<style>.c{text-align: center;} div,input{padding:5px;font-size:1em;} input{width:95%;} body{text-align: center;font-family:verdana;} button{border:0;border-radius:0.3rem;background-color:#1fa3ec;color:#fff;line-height:2.4rem;font-size:1.1rem;width:100%;} .q{float: right;width: 64px;text-align: right;} .l{background: url(\"data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAAALVBMVEX///8EBwfBwsLw8PAzNjaCg4NTVVUjJiZDRUUUFxdiZGSho6OSk5Pg4eFydHTCjaf3AAAAZElEQVQ4je2NSw7AIAhEBamKn97/uMXEGBvozkWb9C2Zx4xzWykBhFAeYp9gkLyZE0zIMno9n4g19hmdY39scwqVkOXaxph0ZCXQcqxSpgQpONa59wkRDOL93eAXvimwlbPbwwVAegLS1HGfZAAAAABJRU5ErkJggg==\") no-repeat left center;background-size: 1em;}</style>";
const char HTTP_SCRIPT[] PROGMEM          = "<script>function c(l){document.getElementById('s').value=l.innerText||l.textContent;document.getElementById('p').focus();}</script> ";
//const char HTTP_BIT_SCRIPT[] PROGMEM      = "<script>function updatebit(v,n,b){return(b)?v|(1<<n):v&~(1<<n);}</script>";
const char HTTP_HEAD_END[] PROGMEM        = "</head><body><div style='text-align:left;display:inline-block;min-width:260px;'>";
const char HTTP_LOGO_SVG_SMALL[] PROGMEM  = "<svg class=\"top\" xmlns=\"http://www.w3.org/2000/svg\" x=\"0\" y=\"0\" width=\"44px\" height=\"46px\" viewBox=\"0 0 94 97\">   ";
const char HTTP_LOGO_SVG_LARGE[] PROGMEM  = "<svg class=\"top\" xmlns=\"http://www.w3.org/2000/svg\" x=\"0\" y=\"0\" width=\"94px\" height=\"97px\" viewBox=\"0 0 94 97\">   ";
const char HTTP_LOGO_PATH[] PROGMEM       = "<path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M59.707,38.168c0,1.609-1.303,2.912-2.91,2.912 s-2.909-1.303-2.909-2.912c0-1.605,1.302-2.908,2.909-2.908S59.707,36.562,59.707,38.168z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M46.91,91.997c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S46.91,90.392,46.91,91.997z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M67.66,38.711 74.08,42.564 66.034,47.178\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M47,92 c24.855,0,45-20.145,45-45.002C92,22.146,71.855,2,47,2C22.146,2,2,22.146,2,46.998c0,7.953,2.312,18.233,8.667,25.995 c4.848,5.908,15.083,9.917,25.167,5.25c6.155-2.849,12.924-9.339,15.916-12.503c3.741-3.955,10.948-12.77,13.088-16.424 c7.185-12.273-0.257-23.352-14.627-22.324c-7.94,0.566-12.831,7.312-16.68,16.422c-2.546,6.025-3.522,13.453-2.061,18.061\"/><path fill=\"#FFF\" stroke=\"#30A5E7\" stroke-width=\"2\" stroke-linecap=\"round\" d=\"M35.848,47.939 c0,0-7.844,0.533-11.19-2.15c-1.387-1.112-1.408-2.963-1.123-3.996c0.575-2.083,4.444-2.963,4.444-2.963s-4.574-3.471-3.21-6.623 c0.917-2.121,2.964-2.918,4.728-2.906c4.31,0.028,11.14,6.617,12.1,7.938\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M35.707,64.168c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S35.707,62.562,35.707,64.168z\"/></svg> ";
const char HTTP_PORTAL_OPT_STRT[] PROGMEM = "<form action=\"/wifi\" method=\"get\"><button>Configure WiFi</button></form><form action=\"/0wifi\" method=\"get\"><button> No Scan </button></form><br/><form action=\"/info\" method=\"get\"><button>Info</button></form><br/>";
const char HTTP_PORTAL_OPT2[] PROGMEM     = "<form action=\"/{id}\" method=\"get\"><button>{id}</button></form>  " ;
const char HTTP_PORTAL_OPT_END[] PROGMEM  = "<form action=\"/clear\" method=\"post\"><button>Clear Config</button></form><br/><form action=\"/reset\" method=\"post\"><button>Reset</button></form>  ";
const char HTTP_ITEM[] PROGMEM            = "<div><a href='#p' onclick='c(this)'>{v}</a>&nbsp;<span class='q {i}'>{r}%</span></div>  ";
const char HTTP_FORM_START[] PROGMEM      = "<form method='get' action='wifisave'><input id='s' name='s' length=32 placeholder='SSID'><br/><input id='p' name='p' length=64 type='password' placeholder='password'><br/> ";
const char HTTP_FORM_BUTTON[] PROGMEM     = "<form action=\"/{a}\" method=\"get\"><button>{i}</button></form>" ;
const char HTTP_FORM_EXT_BUTTON[] PROGMEM = "<form action=\"/{a}\" method=\"get\"><button>{i}</button></form><br/>   " ;
const char HTTP_FORM_CUSTOM[] PROGMEM     = "<h1>Custom Parameters</h1><form method='post' action='customsave'>  ";
const char HTTP_FORM_PSTRT[] PROGMEM      = "<form method='get' action='{a}'><br/>   ";
const char HTTP_FORM_PARAM[] PROGMEM      = "<br/><input id='{i}' name='{n}' length={l} placeholder='{p}' value='{v}' {c}>   ";
const char HTTP_FORM_END[] PROGMEM        = "<br/><button id='saveSubmit' type='submit'>save</button></form> ";
const char HTTP_SCAN_LINK[] PROGMEM       = "<br/><div class=\"c\"><a href=\"/wifi\">Scan</a></div>  ";

const char HTTP_WIFIxx_SAVE[] PROGMEM     = "<button type='submit' onclick=\"document.getElementById('SSID').value = '{i}';document.forms['configform'].submit(); \">save as SSID{i} ({s})</button>  ";
const char HTTP_SAVED[] PROGMEM           = "<div>Credentials Saved<br />Trying to connect ESP to network.<br />If it fails reconnect to AP to try again</div>   ";
const char HTTP_SAVED_PARAM[] PROGMEM     = "<div>{i} Saved<br /></div>  ";
const char HTTP_OK[] PROGMEM              = "<form action=\"/\" method=\"get\"><button>OK</button></form>" ;
const char HTTP_END[] PROGMEM             = "</div></body></html>";

#define WIFI_MANAGER_MAX_PARAMS 10
#define WIFI_MANAGER_MAX_EXTENAL_PAGES 4
#define STORAGE_SIZE 4

class myWiFiManagerParameter {
  public:
    myWiFiManagerParameter(const char *custom);
    myWiFiManagerParameter(const char *id, const char *placeholder, const char *defaultValue, int length);
    myWiFiManagerParameter(const char *id, const char *placeholder, const char *defaultValue, int length, const char *custom);
    myWiFiManagerParameter(const char *id, const char *placeholder, String(* setupfunc)(void), void (* actionfunc)(void) );
    myWiFiManagerParameter(const char *id, const char *placeholder, String(* setupfunc)(void));
    myWiFiManagerParameter(const char *id, String(* setupfunc)(void), void (* actionfunc)(void) );
    myWiFiManagerParameter(const char *id, String(* setupfunc)(void));
    

    const char *getID();
    const char *getValue();
    const char *getPlaceholder();
    int         getValueLength();
    const char *getCustomHTML();
    bool        isExternal();
  private:
    myWiFiManagerParameter();
    ~myWiFiManagerParameter();
    
    const char *_id;
    const char *_placeholder;
    char       *_value;
    int         _length;
    const char *_customHTML;
    String (*_setup)(void);
    void (*_actioncallback)(void);

    void init(const char *id, const char *placeholder, const char *defaultValue, int length, const char *custom);

    friend class myWiFiManager;
};

class myWiFiManager
{
  public:
    myWiFiManager();

    boolean       autoConnect();
    boolean       autoConnect(char const *apName, char const *apPassword = NULL);

    void          disconnect();

    //if you want to always start the config portal, without trying to connect first
    boolean       startConfigPortal(char const *apName, char const *apPassword = NULL);

    // get the AP name of the config portal, so it can be used in the callback
    String        getConfigPortalSSID();

    void          resetSettings();

    //sets timeout before webserver loop ends and exits even if there has been no setup.
    //usefully for devices that failed to connect at some point and got stuck in a webserver loop
    //in seconds setConfigPortalTimeout is a new name for setTimeout
    void          setConfigPortalTimeout(unsigned long seconds);
    void          setTimeout(unsigned long seconds);

    //sets timeout for which to attempt connecting, usefull if you get a lot of failed connects
    void          setConnectTimeout(unsigned long seconds);


    void          setDebugOutput(boolean debug);
    //defaults to not showing anything under 8% signal quality if called
    void          setMinimumSignalQuality(int quality = 8);
    //sets a custom ip /gateway /subnet configuration
    void          setAPStaticIPConfig(IPAddress ip, IPAddress gw, IPAddress sn);
    //sets config for a static IP
    void          setSTAStaticIPConfig(IPAddress ip, IPAddress gw, IPAddress sn);
    //called when AP mode and config portal is started
    void          setAPCallback( void (*func)(myWiFiManager*) );
    //called when settings have been changed and connection was successful
    void          setSaveConfigCallback( void (*func)(void) );
    //adds a custom parameter
    void          addParameter(myWiFiManagerParameter *p);
    //if this is set, it will exit after config, even if connection is unsucessful.
    void          setBreakAfterConfig(boolean shouldBreak);
    //if this is set, it will go to wifi off when no connection is found
    void          setSleepAfterAutoConnect(boolean shouldSleep);
    //if this is set, try WPS setup when starting (this will delay config portal for up to 2 mins)
    //TODO
    //if this is set, customise style
    void          setCustomHeadElement(const char* element);
    //if this is true, remove duplicated Access Points - defaut true
    void          setRemoveDuplicateAPs(boolean removeDuplicates);
    
  private:
    std::unique_ptr<DNSServer>        dnsServer;
    std::unique_ptr<ESP8266WebServer> server;

    //const int     WM_DONE                 = 0;
    //const int     WM_WAIT                 = 10;

    //const String  HTTP_HEAD = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"/><title>{v}</title>";

    void          setupConfigPortal();
    void          startWPS();

    const char*   _apName                 = "no-net";
    const char*   _apPassword             = NULL;
    String        _ssid                   = "";
    String        _pass                   = "";
    String        _listSSID[STORAGE_SIZE+1];//             = {"","","","",""};
    String        _listPWD[STORAGE_SIZE+1];//              = {"","","","",""};
    unsigned long _configPortalTimeout    = 0;
    unsigned long _connectTimeout         = 0;
    unsigned long _configPortalStart      = 0;
    String        page                    = "";

    IPAddress     _ap_static_ip;
    IPAddress     _ap_static_gw;
    IPAddress     _ap_static_sn;
    IPAddress     _sta_static_ip;
    IPAddress     _sta_static_gw;
    IPAddress     _sta_static_sn;

    int           _paramsCount            = 0;
    int           _minimumQuality         = -1;
    boolean       _removeDuplicateAPs     = true;
    boolean       _shouldBreakAfterConfig = false;
    boolean       _shouldSleep            = false;
    boolean       _tryWPS                 = false;

    const char*   _customHeadElement      = "";

    //String        getEEPROMString(int start, int len);
    //void          setEEPROMString(int start, int len, String string);

    int           status = WL_IDLE_STATUS;
    int           connectWifi(String ssid, String pass);
    uint8_t       waitForConnectResult();

    void          handleRoot();
    void          handleWifi(boolean scan);
    void          handleWifiSave();
    void          handleInfo();
    void          handleClear();
    void          handleDemo();
    void          handleReset();
    void          handleParam();
    void          handleParamSave();
    void          handleCustom();
    void          handleCustomSave();
    void          handleNotFound();
    void          handle204();
    boolean       captivePortal();

    // DNS server
    const byte    DNS_PORT = 53;

    //helpers
    int           getRSSIasQuality(int RSSI);
    boolean       isIp(String str);
    String        toStringIp(IPAddress ip);

    boolean       connect;
    boolean       _debug = true;
    boolean       _newline_debug = true;

    void (*_apcallback)(myWiFiManager*) = NULL;
    void (*_actioncallback)(void) = NULL;

    myWiFiManagerParameter* _params[WIFI_MANAGER_MAX_PARAMS];

    template <typename Generic>
    void          DEBUG_WM(Generic text);
    template <typename Generic>
    void          DEBUG_WM_LN(Generic text);

    template <class T>
    auto optionalIPFromString(T *obj, const char *s) -> decltype(  obj->fromString(s)  ) {
      return  obj->fromString(s);
    }
    auto optionalIPFromString(...) -> bool {
      DEBUG_WM("NO fromString METHOD ON IPAddress, you need ESP8266 core 2.1.0 or newer for Custom IP configuration to work.");
      return false;
    }
};

#endif
