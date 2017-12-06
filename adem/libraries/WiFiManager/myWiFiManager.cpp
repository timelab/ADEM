/**************************************************************
   myWiFiManager is a library for the ESP8266/Arduino platform
   (https://github.com/esp8266/Arduino) to enable easy
   configuration and reconfiguration of WiFi credentials using a Captive Portal
   inspired by:
   http://www.esp8266.com/viewtopic.php?f=29&t=2520
   https://github.com/chriscook8/esp-arduino-apboot
   https://github.com/esp8266/Arduino/tree/esp8266/hardware/esp8266com/esp8266/libraries/DNSServer/examples/CaptivePortalAdvanced
   Built by AlexT https://github.com/tzapu
   Licensed under MIT license
 **************************************************************/

/******************************************

standard wifi pages added to the root
>>> keywords can not be used as id's for parameters or external pages <<<

/wifi   config wifi with network scan
/0wifi  config wifi without network scan
/wifisave   save the wifi configuration
/clear      clear configuration
/info       show info page
/reset      reset controller
/param      show additional parameter page
/paramsave  saves the additional parameters in the json file if customsave function is set then this one will be called

/*****************************************/
#include "myWiFiManager.h"
#include <FS.h> 
#include "BufferedResponse.h"

myWiFiManagerParameter::myWiFiManagerParameter() {
  _id = NULL;
  _placeholder = NULL;
  _length = 0;
  _value = NULL;

  _customHTML = NULL;
  _setup = NULL;
  _actioncallback = NULL;
}

myWiFiManagerParameter::myWiFiManagerParameter(const char *custom) {
  _id = NULL;
  _placeholder = NULL;
  _length = 0;
  _value = NULL;

  _customHTML = custom;
  _setup = NULL;
  _actioncallback = NULL;
}

myWiFiManagerParameter::myWiFiManagerParameter(const char *id, const char *placeholder, const char *defaultValue, int length) {
  init(id, placeholder, defaultValue, length, "");
}

myWiFiManagerParameter::myWiFiManagerParameter(const char *id, const char *placeholder, const char *defaultValue, int length, const char *custom) {
  init(id, placeholder, defaultValue, length, custom);
}

void myWiFiManagerParameter::init(const char *id, const char *placeholder, const char *defaultValue, int length, const char *custom) {
  _id = id;
  _placeholder = placeholder;
  _length = length;
  _value = new char[length + 1];
  for (int i = 0; i < length; i++) {
    _value[i] = 0;
  }
  if (defaultValue != NULL) {
    strncpy(_value, defaultValue, length);
  }

  _customHTML = custom;
  _setup = NULL;
  _actioncallback = NULL;
}

myWiFiManagerParameter::myWiFiManagerParameter(const char *id, const char *placeholder, String(* setupfunc)(void), void (*actionfunc)(void) ){
  _id = id;
  _placeholder = placeholder;
  _length = 0;
  _value = NULL;
  _customHTML = NULL;
  _setup = setupfunc;
  _actioncallback = actionfunc;

}

myWiFiManagerParameter::myWiFiManagerParameter(const char *id, const char *placeholder, String(* setupfunc)(void)){
 _id = id;
  _placeholder = placeholder;
  _length = 0;
  _value = NULL;
  _customHTML = NULL;
  _setup = setupfunc;
  _actioncallback = NULL;
}

myWiFiManagerParameter::myWiFiManagerParameter(const char *id, String(* setupfunc)(void), void (*actionfunc)(void) ){
  _id = id;
  _placeholder = NULL;
  _length = 0;
  _value = NULL;
  _customHTML = NULL;
  _setup = setupfunc;
  _actioncallback = actionfunc;

}

myWiFiManagerParameter::myWiFiManagerParameter(const char *id, String(* setupfunc)(void)){
 _id = id;
  _placeholder = NULL;
  _length = 0;
  _value = NULL;
  _customHTML = NULL;
  _setup = setupfunc;
  _actioncallback = NULL;
}

const char* myWiFiManagerParameter::getValue() {
  return _value;
}
const char* myWiFiManagerParameter::getID() {
  return _id;
}
const char* myWiFiManagerParameter::getPlaceholder() {
  return _placeholder;
}
int myWiFiManagerParameter::getValueLength() {
  return _length;
}
const char* myWiFiManagerParameter::getCustomHTML() {
  return _customHTML;
}

bool myWiFiManagerParameter::isExternal(){
  return _setup != NULL;
}

////==========================================================================================////

myWiFiManager::myWiFiManager() {
  DEBUG_WM("mounting FS...");

  if (SPIFFS.begin()) {
    DEBUG_WM_LN("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      DEBUG_WM("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        DEBUG_WM("....opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          DEBUG_WM_LN("....parsed json");
          const char *tmp;
          char jsonID[10] ="";
          /// Loop over storage size
          for(int i=1;i<=STORAGE_SIZE;i++){
            sprintf(jsonID,"SSID%d",i);
            tmp = json[jsonID];
            _listSSID[i] = String(tmp);
            DEBUG_WM("reading : ");
            DEBUG_WM(jsonID);
            DEBUG_WM(" = ");
            DEBUG_WM_LN(tmp);
            sprintf(jsonID,"PWD%d",i);
            tmp = json[jsonID];
            _listPWD[i] = String(tmp);
          }
   
          /// TODO check if additional parameter are stored and available
          /// access point name
          /// access point password
          
        } else {
          DEBUG_WM_LN("failed to load json config");
        }
      }
    }
  } else {
    DEBUG_WM_LN("failed to mount FS");
  }
}

void myWiFiManager::addParameter(myWiFiManagerParameter *p) {
  _params[_paramsCount] = p;
  _paramsCount++;
  DEBUG_WM("Adding parameter ");
  DEBUG_WM(_paramsCount);
  DEBUG_WM(" : ");
  DEBUG_WM_LN(String(p->getID()));
}

void myWiFiManager::setupConfigPortal() {
  dnsServer.reset(new DNSServer());
  server.reset(new ESP8266WebServer(80));

  DEBUG_WM_LN(F(""));
  _configPortalStart = millis();

  DEBUG_WM(F("Configuring access point... "));
  DEBUG_WM_LN(_apName);
  if (_apPassword != NULL) {
    if (strlen(_apPassword) < 8 || strlen(_apPassword) > 63) {
      // fail passphrase to short or long!
      DEBUG_WM_LN(F("Invalid AccessPoint password. Ignoring"));
      _apPassword = NULL;
    }
    DEBUG_WM_LN(_apPassword);
  }

  //optional soft ip config
  if (_ap_static_ip) {
    DEBUG_WM_LN(F("Custom AP IP/GW/Subnet"));
    WiFi.softAPConfig(_ap_static_ip, _ap_static_gw, _ap_static_sn);
  }

  if (_apPassword != NULL) {
    WiFi.softAP(_apName, _apPassword);//password option
  } else {
    WiFi.softAP(_apName);
  }

  delay(500); // Without delay I've seen the IP address blank
  DEBUG_WM(F("AP IP address: "));
  DEBUG_WM_LN(WiFi.softAPIP());

  /* Setup the DNS server redirecting all the domains to the apIP */
  dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer->start(DNS_PORT, "*", WiFi.softAPIP());

  /* Setup web pages: root, wifi config pages, SO captive portal detectors and not found. */
  server->on("/", std::bind(&myWiFiManager::handleRoot, this));
  server->on("/wifi", std::bind(&myWiFiManager::handleWifi, this, true));
  server->on("/0wifi", std::bind(&myWiFiManager::handleWifi, this, false));
  server->on("/wifisave", std::bind(&myWiFiManager::handleWifiSave, this));
  server->on("/info", std::bind(&myWiFiManager::handleInfo, this));
  server->on("/clear", std::bind(&myWiFiManager::handleClear, this));
  server->on("/reset", std::bind(&myWiFiManager::handleReset, this));
  //server->on("/generate_204", std::bind(&myWiFiManager::handle204, this));  //Android/Chrome OS captive portal check.
  server->on("/fwlink", std::bind(&myWiFiManager::handleRoot, this));  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  server->onNotFound (std::bind(&myWiFiManager::handleNotFound, this));
  server->begin(); // Web server start
  DEBUG_WM_LN(F("HTTP server started"));

}

boolean myWiFiManager::autoConnect() {
  String ssid = "ADEM" + String(ESP.getChipId());
  return autoConnect(ssid.c_str(), NULL);
}

boolean myWiFiManager::autoConnect(char const *apName, char const *apPassword) {
  DEBUG_WM(F(""));
  DEBUG_WM_LN(F("AutoConnect"));


  // attempt to connect; should it fail, fall back to AP
  yield();
  WiFi.mode(WIFI_STA);
  // read eeprom for ssid and pass
  //String ssid = getSSID();
  //String pass = getPassword();
  DEBUG_WM(F("Reading SSID :"));
  _ssid = WiFi.SSID();
  DEBUG_WM_LN(_ssid);
   
  if (connectWifi("", "") == WL_CONNECTED)   {
    DEBUG_WM(F("IP Address: "));
    DEBUG_WM_LN(WiFi.localIP());
    //connected
    return true;
  }
  if (_shouldSleep) {
    WiFi.mode(WIFI_OFF);
    return false;
  }

  return startConfigPortal(apName, apPassword);
}

boolean  myWiFiManager::startConfigPortal(char const *apName, char const *apPassword) {
  //setup AP
  WiFi.mode(WIFI_AP_STA);
  DEBUG_WM_LN("SET AP STA");

  _apName = apName;
  _apPassword = apPassword;

  //notify we entered AP mode
  if ( _apcallback != NULL) {
    _apcallback(this);
  }

  connect = false;
  setupConfigPortal();

  while (_configPortalTimeout == 0 || millis() < _configPortalStart + _configPortalTimeout) {
    //DNS
    dnsServer->processNextRequest();
    //HTTP
    server->handleClient();


    if (connect) {
      connect = false;
      delay(2000);
      DEBUG_WM_LN(F("Connecting to new AP"));

      // using user-provided  _ssid, _pass in place of system-stored ssid and pass
      if (connectWifi(_ssid, _pass) != WL_CONNECTED) {
        DEBUG_WM_LN(F("Failed to connect."));
      } else {
        //connected
        WiFi.mode(WIFI_STA);
        //notify that configuration has changed and any optional parameters should be saved
        if ( _actioncallback != NULL) {
          //todo: check if any custom parameters actually exist, and check if they really changed maybe
          _actioncallback();
        }
        break;
      }

      if (_shouldBreakAfterConfig) {
        //flag set to exit after config after trying to connect
        //notify that configuration has changed and any optional parameters should be saved
        if ( _actioncallback != NULL) {
          //todo: check if any custom parameters actually exist, and check if they really changed maybe
          _actioncallback();
        }
        break;
      }
    }
    yield();
  }

  server.reset();
  dnsServer.reset();

  return  WiFi.status() == WL_CONNECTED;
}


int myWiFiManager::connectWifi(String ssid, String pass) {
  int connRes = 0;
  DEBUG_WM_LN(F("Connecting as wifi client..."));

  // check if we've got static_ip settings, if we do, use those.
  if (_sta_static_ip) {
    DEBUG_WM(F("Custom STA IP/GW/Subnet "));
    WiFi.config(_sta_static_ip, _sta_static_gw, _sta_static_sn);
    DEBUG_WM_LN(WiFi.localIP());
  }
  //fix for auto connect racing issue
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_WM_LN("Already connected. Bailing out.");
    return WL_CONNECTED;
  }
  //check if we have ssid and pass and force those, if not, try with last saved values
  if (ssid != "") {
    // force to connect to specified access point
    WiFi.begin(ssid.c_str(), pass.c_str());
    connRes = waitForConnectResult();
  } else {
    if (WiFi.SSID() && WiFi.SSID() != "") {
      DEBUG_WM("Using last saved values, should be faster : ");
      DEBUG_WM_LN(WiFi.SSID());
      //trying to fix connection in progress hanging
      ETS_UART_INTR_DISABLE();
      wifi_station_disconnect();
      ETS_UART_INTR_ENABLE();

      WiFi.begin();
      connRes = waitForConnectResult();
    } 
    if (connRes != WL_CONNECTED) {
    // no save credential
    // scan the networks and select the strongest one that is in the config file
        DEBUG_WM(F("Scanning access points ..."));
        int n = WiFi.scanNetworks();
        DEBUG_WM_LN(F(" done"));
        if (n == 0) {
          DEBUG_WM_LN(F("No networks found"));
          return connRes; // no connection possible
        } else {
          //sort networks
          int indices[n];
          for (int i = 0; i < n; i++) {
            indices[i] = i;
          }

          // RSSI SORT
          for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
              if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
                std::swap(indices[i], indices[j]);
              }
            }
          }
          for (int i = 0; i < n; i++) {
            for (int j = 1; j < 5; j++) {
              if (WiFi.SSID(indices[i])  == _listSSID[j]) {
                DEBUG_WM("try to connect to ");
                DEBUG_WM(_listSSID[j].c_str());
                DEBUG_WM(" #  ");
                DEBUG_WM(_listPWD[j].c_str());
                WiFi.begin(_listSSID[j].c_str(),_listPWD[j].c_str());
                connRes = waitForConnectResult();
                DEBUG_WM(" -->  ");
                DEBUG_WM_LN(connRes);
                break;
              }
            }
            if (connRes == WL_CONNECTED) break;
          }
          
       } 
    }
  }
  // int connRes = waitForConnectResult();
  DEBUG_WM ("Connection result: ");
  DEBUG_WM_LN ( connRes );
  //not connected, WPS enabled, no pass - first attempt
  if (_tryWPS && connRes != WL_CONNECTED && pass == "") {
    startWPS();
    //should be connected at the end of WPS
    connRes = waitForConnectResult();
  }
  return connRes;
}

uint8_t myWiFiManager::waitForConnectResult() {
  if (_connectTimeout == 0) {
    return WiFi.waitForConnectResult();
  } else {
   // DEBUG_WM_LN (F("Waiting for connection result with time out"));
    unsigned long start = millis();
    boolean keepConnecting = true;
    uint8_t status;
    while (keepConnecting) {
      status = WiFi.status();
      if (millis() > start + _connectTimeout) {
        keepConnecting = false;
        DEBUG_WM_LN (F("Connection timed out"));
      }
      if (status == WL_CONNECTED || status == WL_CONNECT_FAILED) {
        keepConnecting = false;
      }
      DEBUG_WM (F("."));
      delay(100);
      yield();
    }
    return status;
  }
}

void myWiFiManager::startWPS() {
  DEBUG_WM_LN("START WPS");
  WiFi.beginWPSConfig();
  DEBUG_WM_LN("END WPS");
}
/*
  String myWiFiManager::getSSID() {
  if (_ssid == "") {
    DEBUG_WM_LN(F("Reading SSID"));
    _ssid = WiFi.SSID();
    DEBUG_WM_LN(F("SSID: "));
    DEBUG_WM_LN(_ssid);
  }
  return _ssid;
  }

  String myWiFiManager::getPassword() {
  if (_pass == "") {
    DEBUG_WM_LN(F("Reading Password"));
    _pass = WiFi.psk();
    DEBUG_WM_LN("Password: " + _pass);
    //DEBUG_WM_LN(_pass);
  }
  return _pass;
  }
*/
String myWiFiManager::getConfigPortalSSID() {
  return _apName;
}

void myWiFiManager::resetSettings() {
  DEBUG_WM_LN(F("settings invalidated"));
  DEBUG_WM_LN(F("THIS MAY CAUSE AP NOT TO START UP PROPERLY. YOU NEED TO COMMENT IT OUT AFTER ERASING THE DATA."));
  WiFi.disconnect(true);
  //delay(200);
}
void myWiFiManager::setTimeout(unsigned long seconds) {
  setConfigPortalTimeout(seconds);
}

void myWiFiManager::setConfigPortalTimeout(unsigned long seconds) {
  _configPortalTimeout = seconds * 1000;
}

void myWiFiManager::setConnectTimeout(unsigned long seconds) {
  _connectTimeout = seconds * 1000;
}

void myWiFiManager::setDebugOutput(boolean debug) {
  _debug = debug;
}

void myWiFiManager::setAPStaticIPConfig(IPAddress ip, IPAddress gw, IPAddress sn) {
  _ap_static_ip = ip;
  _ap_static_gw = gw;
  _ap_static_sn = sn;
}

void myWiFiManager::setSTAStaticIPConfig(IPAddress ip, IPAddress gw, IPAddress sn) {
  _sta_static_ip = ip;
  _sta_static_gw = gw;
  _sta_static_sn = sn;
}

void myWiFiManager::setMinimumSignalQuality(int quality) {
  _minimumQuality = quality;
}

void myWiFiManager::setBreakAfterConfig(boolean shouldBreak) {
  _shouldBreakAfterConfig = shouldBreak;
}

void myWiFiManager::setSleepAfterAutoConnect(boolean shouldSleep) {
  _shouldSleep = shouldSleep;
}

//// =============================================================================================
//// webserver handlers
//// =============================================================================================
/** Handle root or redirect to captive portal */
void myWiFiManager::handleRoot() {
  DEBUG_WM_LN(F("Handle root"));
  if (captivePortal()) { // If caprive portal redirect instead of displaying the page.
    return;
  }

  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Options");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += "<h1>";
  page += FPSTR(HTTP_LOGO_SVG_SMALL);
  page += FPSTR(HTTP_LOGO_PATH);
  page += _apName;
  page += "</h1>";
 // page += F("<h3>myWiFiManager</h3>");
  page += FPSTR(HTTP_PORTAL_OPT_STRT);

  bool param = false;

  for (int i = 0; i < _paramsCount; i++) {
    if (_params[i] == NULL) {
      break;
    }
    if (_params[i]->isExternal()){
      if (_params[i]->getPlaceholder() != NULL) {    // only parameters with placeholders show up as external webpages via button on root page
        String pitem = FPSTR(HTTP_FORM_EXT_BUTTON);  // <form action=\"/{a}\" method=\"get\"><button>{i}</button></form><br/>
        pitem.replace("{i}", _params[i]->getPlaceholder());   // getPlaceholder
        pitem.replace("{a}", _params[i]->getID());
        page += pitem;
        pitem = "/";
        pitem += _params[i]->getID();
        /// TOOD add external button with ID text
        // server->on(pitem,_params[i]._actioncallback)
        // /<ID> is automatically handled via the not found call
      }
    }
    else
      param = true;
  }
  if (param){
    String pitem = FPSTR(HTTP_FORM_BUTTON);
    pitem.replace("{i}", "Config Parameters");
    pitem.replace("{a}", "Param");
    page += pitem;
    server->on("/Param", std::bind(&myWiFiManager::handleParam, this));
    /// TODO add parameter button
  }

  page += FPSTR(HTTP_PORTAL_OPT_END);
  page += FPSTR(HTTP_END);

  server->send(200, "text/html", page);

}

/** Wifi config page handler */
void myWiFiManager::handleWifi(boolean scan) {

  
  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Config ESP");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);

  if (scan) {
    DEBUG_WM(F("Scan access points ..."));
    Serial.printf("client heap size: %u\n", ESP.getFreeHeap());
    int n = WiFi.scanNetworks();
    DEBUG_WM_LN(F(" done"));
    if (n == 0) {
      DEBUG_WM_LN(F("No networks found"));
      page += F("No networks found. Refresh to scan again.");
    } else {

      //sort networks
      int indices[n];
      for (int i = 0; i < n; i++) {
        indices[i] = i;
      }

      // RSSI SORT

      // old sort
      for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
          if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
            std::swap(indices[i], indices[j]);
          }
        }
      }

      /*std::sort(indices, indices + n, [](const int & a, const int & b) -> bool
        {
        return WiFi.RSSI(a) > WiFi.RSSI(b);
        });*/

      // remove duplicates ( must be RSSI sorted )
      if (_removeDuplicateAPs) {
        String cssid;
        for (int i = 0; i < n; i++) {
          if (indices[i] == -1) continue;
          cssid = WiFi.SSID(indices[i]);
          for (int j = i + 1; j < n; j++) {
            if (cssid == WiFi.SSID(indices[j])) {
              DEBUG_WM_LN("DUP AP: " + WiFi.SSID(indices[j]));
              indices[j] = -1; // set dup aps to index -1
            }
          }
        }
      }

      //display networks in page
      for (int i = 0; i < n; i++) {
        if (indices[i] == -1) continue; // skip dups
        DEBUG_WM(WiFi.SSID(indices[i]));
        DEBUG_WM(" : ");
        DEBUG_WM(WiFi.RSSI(indices[i]));
        int quality = getRSSIasQuality(WiFi.RSSI(indices[i]));

        if (_minimumQuality == -1 || _minimumQuality < quality) {
          String item = FPSTR(HTTP_ITEM);
          String rssiQ;
          rssiQ += quality;
          item.replace("{v}", WiFi.SSID(indices[i]));
          item.replace("{r}", rssiQ);
          if (WiFi.encryptionType(indices[i]) != ENC_TYPE_NONE) {
            item.replace("{i}", "l");
          } else {
            item.replace("{i}", "");
          }
          DEBUG_WM_LN("");
          page += item;
          delay(0);
        } else {
          DEBUG_WM_LN(F(" Skipping due to quality"));
        }

      }
      page += "<br/>";
    }
  }

  page += FPSTR(HTTP_FORM_START);
  page += "<input id='SSID' name='SSID' length=1 placeholder='0' value='0' type='hidden'>";
  for (int i = 1;i <= STORAGE_SIZE; i++){
    String pitem = FPSTR(HTTP_WIFIxx_SAVE);
    pitem.replace("{i}",String(i));
    pitem.replace("{s}", _listSSID[i]);
    page += pitem;
  }

  if (_sta_static_ip) {

    String item = FPSTR(HTTP_FORM_PARAM);
    item.replace("{i}", "ip");
    item.replace("{n}", "ip");
    item.replace("{p}", "Static IP");
    item.replace("{l}", "15");
    item.replace("{v}", _sta_static_ip.toString());

    page += item;

    item = FPSTR(HTTP_FORM_PARAM);
    item.replace("{i}", "gw");
    item.replace("{n}", "gw");
    item.replace("{p}", "Static Gateway");
    item.replace("{l}", "15");
    item.replace("{v}", _sta_static_gw.toString());

    page += item;

    item = FPSTR(HTTP_FORM_PARAM);
    item.replace("{i}", "sn");
    item.replace("{n}", "sn");
    item.replace("{p}", "Subnet");
    item.replace("{l}", "15");
    item.replace("{v}", _sta_static_sn.toString());

    page += item;

    page += "<br/>";
  }

  page += FPSTR(HTTP_FORM_END);
  page += FPSTR(HTTP_SCAN_LINK);

  page += FPSTR(HTTP_END);
Serial.printf("client heap size: %u\n", ESP.getFreeHeap());
  server->send(200, "text/html", page);

Serial.printf("client heap size: %u\n", ESP.getFreeHeap());
  DEBUG_WM_LN(F("Sent config page"));
  DEBUG_WM_LN(page.c_str());
}

/** Handle the WLAN save form and redirect to WLAN config page again */
void myWiFiManager::handleWifiSave() {
  DEBUG_WM_LN(F("WiFi save"));

  //SAVE/connect here
  DEBUG_WM(F("SSID "));
  _ssid = server->arg("s").c_str();
  DEBUG_WM_LN(_ssid);
  _pass = server->arg("p").c_str();

  DEBUG_WM(F("save button # "));
  int id =  atoi(server->arg("SSID").c_str());
  DEBUG_WM_LN(id);

  _listSSID[id] = _ssid;
  _listPWD[id] = _pass;
  
  if (id != 0) {
    DEBUG_WM_LN("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["SSID1"] = _listSSID[1];
    json["SSID2"] = _listSSID[2];
    json["SSID3"] = _listSSID[3];
    json["SSID4"] = _listSSID[4];

    json["PWD1"] = _listPWD[1];
    json["PWD2"] = _listPWD[2];
    json["PWD3"] = _listPWD[3];
    json["PWD4"] = _listPWD[4];

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      DEBUG_WM_LN("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
/* parameters are handled in a separate page
  //parameters
  for (int i = 0; i < _paramsCount; i++) {
    if (_params[i] == NULL) {
      break;
    }
    //read parameter
    String value = server->arg(_params[i]->getID()).c_str();
    //store it in array
    if (i == id){
      value = _ssid;
    }
    value.toCharArray(_params[i]->_value, _params[i]->_length);
    DEBUG_WM(F("Parameter "));
    DEBUG_WM(_params[i]->getID());
    DEBUG_WM(F(" = "));
    DEBUG_WM_LN(value);
  }
*/
  if (server->arg("ip") != "") {
    DEBUG_WM(F("static ip "));
    DEBUG_WM_LN(server->arg("ip"));
    //_sta_static_ip.fromString(server->arg("ip"));
    String ip = server->arg("ip");
    optionalIPFromString(&_sta_static_ip, ip.c_str());
  }
  if (server->arg("gw") != "") {
    DEBUG_WM(F("static gateway "));
    DEBUG_WM_LN(server->arg("gw"));
    String gw = server->arg("gw");
    optionalIPFromString(&_sta_static_gw, gw.c_str());
  }
  if (server->arg("sn") != "") {
    DEBUG_WM(F("static netmask "));
    DEBUG_WM_LN(server->arg("sn"));
    String sn = server->arg("sn");
    optionalIPFromString(&_sta_static_sn, sn.c_str());
  }

  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Credentials Saved");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += FPSTR(HTTP_SAVED);
  page += FPSTR(HTTP_END);

  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);

  DEBUG_WM_LN(F("Sent wifi save page"));

  connect = true; //signal ready to connect/reset
}

/** Parameter config page handler */
void myWiFiManager::handleParam() {

  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Config ESP");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += "<h1>";
  page += FPSTR(HTTP_LOGO_SVG_SMALL);
  page += FPSTR(HTTP_LOGO_PATH);
  page += "Parameters" ;
  page += "</h1>";
  String pitem = FPSTR(HTTP_FORM_PSTRT);
  pitem.replace("{a}","ParamSave");
  page += pitem;

  server->on("/ParamSave", std::bind(&myWiFiManager::handleParamSave, this));

  char parLength[2];
  // add the extra parameters to the form
  for (int i = 0; i < _paramsCount; i++) {
    if (_params[i] == NULL) {
      break;
    }
    if (! _params[i]->isExternal()){

      pitem = FPSTR(HTTP_FORM_PARAM);
      if (_params[i]->getID() != NULL) {
        pitem.replace("{i}", _params[i]->getID());
        pitem.replace("{n}", _params[i]->getID());
        pitem.replace("{p}", _params[i]->getPlaceholder());
        snprintf(parLength, 2, "%d", _params[i]->getValueLength());
        pitem.replace("{l}", parLength);
        pitem.replace("{v}", _params[i]->getValue());
        pitem.replace("{c}", _params[i]->getCustomHTML());
      } else {
        pitem = _params[i]->getCustomHTML();
      }

      page += pitem;
    }
  }
  page += FPSTR(HTTP_FORM_END);
  
  page += FPSTR(HTTP_END);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);

  
  DEBUG_WM_LN(F("Sent config page"));
  Serial.println(page);
}

/** Handle the parameter save button */
void myWiFiManager::handleParamSave() {
  
  DEBUG_WM_LN(F("Parameter save"));

  //parameters
  for (int i = 0; i < _paramsCount; i++) {
    if (_params[i] == NULL) {
      break;
    }
    //read parameter
    String value = server->arg(_params[i]->getID()).c_str();
    
    value.toCharArray(_params[i]->_value, _params[i]->_length);
    DEBUG_WM(F("Parameter "));
    DEBUG_WM(_params[i]->getID());
    DEBUG_WM(F(" = "));
    DEBUG_WM_LN(value);
  }
  if ( _actioncallback != NULL) {
          _actioncallback();
  }
  else
  {
    ///todo call save json
  }
  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Parameters Saved");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += "<h1>";
  page += FPSTR(HTTP_LOGO_SVG_SMALL);
  page += FPSTR(HTTP_LOGO_PATH);
  page += "</h1>";
  page += FPSTR(HTTP_SAVED_PARAM);
  page.replace("{i}","Parameters");
  page += FPSTR(HTTP_OK);
  page += FPSTR(HTTP_END);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);

  DEBUG_WM_LN(F("Sent parameter save page"));

}


/** Handle the clear page */
void myWiFiManager::handleDemo() {
  
  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Demo");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += F("<dl>");
  page += F("<dt>Demo Mode selected</dt>");
  page += F("</dl>");
  page += FPSTR(HTTP_END);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);

  DEBUG_WM_LN(F("Sent Demo page"));
  Serial.println(page);
}

/** Handle the custom page */
void myWiFiManager::handleCustom() {

  DEBUG_WM_LN(F("Handle Custom "));
//// TODO make custom parameter input based on wifi page  
  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Custom parameters");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += FPSTR(HTTP_LOGO_SVG_SMALL);
  page += FPSTR(HTTP_LOGO_PATH);
  page += FPSTR(HTTP_FORM_CUSTOM);
  char parLength[2];
  // add the extra parameters to the form
  DEBUG_WM(F("add parameters "));
  DEBUG_WM_LN(_paramsCount);
  for (int i = 0; i < _paramsCount; i++) {
    if (_params[i] == NULL) {
      break;
    }

    String pitem = FPSTR(HTTP_FORM_PARAM);
    if (_params[i]->getID() != NULL) {
      DEBUG_WM(_params[i]->getID());
      pitem.replace("{i}", _params[i]->getID());
      pitem.replace("{n}", _params[i]->getID());
      pitem.replace("{p}", _params[i]->getPlaceholder());
      snprintf(parLength, 2, "%d", _params[i]->getValueLength());
      pitem.replace("{l}", parLength);
      pitem.replace("{v}", _params[i]->getValue());
      pitem.replace("{c}", _params[i]->getCustomHTML());
    } else {
      DEBUG_WM("get custom HTML");
      pitem = _params[i]->getCustomHTML();
    }
    DEBUG_WM(F(" ,"));
    page += pitem;
  }
  DEBUG_WM_LN(F(""));
  if (_params[0] != NULL) {
    page += "<br/>";
  }
  page += FPSTR(HTTP_FORM_END);
  page += FPSTR(HTTP_END);
  Serial.println(page);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);

  DEBUG_WM_LN(F("Sent custom page"));
  Serial.println(page);
}
/** Handle the custom page */
void myWiFiManager::handleCustomSave() {

  Serial.println("URL is: " + server->uri());
  Serial.print("HTTP Method on request was: ");
  switch(server->method()) {
  case HTTP_GET:
    Serial.println("GET");
    break ;
  case HTTP_POST:
    Serial.println( "POST");
    break ;
  case HTTP_PUT:
    Serial.println( "PUT");
    break ;
  case HTTP_PATCH:
    Serial.println( "PATCH");
    break ;
  case HTTP_DELETE:
    Serial.println( "DELETE");
    break ;
  }
  // Print how many properties we received and then print their names
  // and values.
  Serial.println("Number of query properties: " + String(server->args()));
  int i;
  for (i=0; i<server->args(); i++) {
    Serial.println(" - " + server->argName(i) + " = " + server->arg(i));
  }

  DEBUG_WM_LN(F("Sent custom save page"));
  
  //SAVE/connect here
  DEBUG_WM(F("value "));
  DEBUG_WM_LN( server->arg("config").c_str());
  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Custom Save");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += F("<dl>");
  page += F("<dt>Custom save executed</dt>");
  page += F("</dl>");
  page += FPSTR(HTTP_OK);
  page += FPSTR(HTTP_END);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);
  DEBUG_WM_LN(F("Sent wifi save page"));

}

/** Handle the clear page */
void myWiFiManager::handleClear() {
  //clean FS, for testing
  SPIFFS.format();
  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Clear");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += "<h1>";
  page += FPSTR(HTTP_LOGO_SVG_SMALL);
  page += FPSTR(HTTP_LOGO_PATH);
  page += "</h1>";
  page += F("<dl>");
  page += F("<dt>Config file cleared</dt>");
  page += F("</dl>");
  page += FPSTR(HTTP_OK);
  page += FPSTR(HTTP_END);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
  server->send(200, "text/html", page);

  DEBUG_WM_LN(F("Sent Clear page"));

}
/** Handle the info page */
void myWiFiManager::handleInfo() {
  DEBUG_WM_LN(F("Info"));
  BufferedResponse response((ESP8266WebServer &) *server, "text/html", 200, 128);

  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Info");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += "<h1>";
  page += FPSTR(HTTP_LOGO_SVG_SMALL);
  page += FPSTR(HTTP_LOGO_PATH);
  page += "Info</h1>";

  page += F("<dl>");
  page += F("<dt>Chip ID</dt><dd>");
  page += ESP.getChipId();
  page += F("</dd>");
  page += F("<dt>Flash Chip ID</dt><dd>");
  page += ESP.getFlashChipId();
  page += F("</dd>");
  page += F("<dt>IDE Flash Size</dt><dd>");
  page += ESP.getFlashChipSize();
  page += F(" bytes</dd>");
  page += F("<dt>Real Flash Size</dt><dd>");
  page += ESP.getFlashChipRealSize();
  page += F(" bytes</dd>");
  page += F("<dt>Soft AP IP</dt><dd>");
  page += WiFi.softAPIP().toString();
  page += F("</dd>");
  page += F("<dt>Soft AP MAC</dt><dd>");
  page += WiFi.softAPmacAddress();
  page += F("</dd>");
  page += F("<dt>Station MAC</dt><dd>");
  page += WiFi.macAddress();
  page += F("</dd>");
  page += F("</dl>");
  page += FPSTR(HTTP_OK);
  page += FPSTR(HTTP_END);
  Serial.println(page);
  //server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations  
  //server->send(200, "text/html", page);
 
  response.print(page);
  response.flush();

  DEBUG_WM_LN(F("Sent info page"));
  Serial.println(page);
}

/** Handle the reset page */
void myWiFiManager::handleReset() {
  DEBUG_WM_LN(F("Reset"));

  page = FPSTR(HTTP_HEAD);
  page.replace("{v}", "Reset");
  page += FPSTR(HTTP_SCRIPT);
  page += FPSTR(HTTP_STYLE);
  page += _customHeadElement;
  page += FPSTR(HTTP_HEAD_END);
  page += F("Module will reset in a few seconds.");
  page += FPSTR(HTTP_END);
  server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations  
  server->send(200, "text/html", page);

  DEBUG_WM_LN(F("Sent reset page"));
  Serial.println(page);
  delay(5000);
  ESP.reset();
  delay(2000);
}



//removed as mentioned here https://github.com/tzapu/myWiFiManager/issues/114
/*void myWiFiManager::handle204() {
  DEBUG_WM_LN(F("204 No Response"));
  server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->send ( 204, "text/plain", "");
}*/

void myWiFiManager::handleNotFound() {
  
  String uri = server->uri();
  DEBUG_WM("Handle Not Found :");
  DEBUG_WM_LN(uri);
  
  for (int i = 0; i < _paramsCount; i++) {
    if (_params[i] == NULL) {
      break;
    }
    if (_params[i]->isExternal()){
      DEBUG_WM("check for parameter ");DEBUG_WM_LN(_params[i]->getID());
      if (uri.endsWith(_params[i]->getID())) {
        if (uri.equals("/"+ String(_params[i]->getID())))
        {
          DEBUG_WM( "call parameter external setup function ");
          page = _params[i]->_setup();
          DEBUG_WM_LN( "--> OK");
          DEBUG_WM("setup function returned : ");
          if(String(_params[i]->getID()).endsWith("json")){
            DEBUG_WM_LN("json data ");
            Serial.println(page);
            server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations  
            server->send(200, "application/json", page);
          }
          else{
            DEBUG_WM_LN("html page ");
            Serial.println(page);
            BufferedResponse response((ESP8266WebServer &) *server, "text/html", 200, 128);
            response.print(page);
            response.flush();
            // server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations
            // server->send(200,"text/html",page);
          }
          return;
        }
        if (uri.equals("/save "+ String(_params[i]->getID())))
        {
          DEBUG_WM_LN( "call parameter custom save function ");
          if ( _params[i]->_actioncallback != NULL)
            _params[i]->_actioncallback();
          page = FPSTR(HTTP_HEAD);
          page.replace("{v}", "Custom Save");
          page += FPSTR(HTTP_SCRIPT);
          page += FPSTR(HTTP_STYLE);
          page += _customHeadElement;
          page += FPSTR(HTTP_HEAD_END);
          page += "<h1>";
          page += FPSTR(HTTP_LOGO_SVG_SMALL);
          page += FPSTR(HTTP_LOGO_PATH);
          page += "</h1>";
          page += F("<dl><dt>");
          page += _params[i]->getID();
          page += " saved </dt></dl>";
          page += FPSTR(HTTP_OK);
          page += FPSTR(HTTP_END);
          server->setContentLength(CONTENT_LENGTH_UNKNOWN); // force chunked response to overcome the heap limitations  
          server->send(200, "text/html", page);

          DEBUG_WM_LN(F("Sent Custom save page"));
          return;
        }
      }
    }
  }
  
  if (captivePortal()) { // If captive portal redirect instead of displaying the error page.
    return;
  }
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server->uri();
  message += "\nMethod: ";
  message += ( server->method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server->args();
  message += "\n";

  for ( uint8_t i = 0; i < server->args(); i++ ) {
    message += " " + server->argName ( i ) + ": " + server->arg ( i ) + "\n";
  }
  server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server->sendHeader("Pragma", "no-cache");
  server->sendHeader("Expires", "-1");
  server->send ( 404, "text/plain", message );
}


/** Redirect to captive portal if we got a request for another domain. Return true in that case so the page handler do not try to handle the request again. */
boolean myWiFiManager::captivePortal() {

  if (!isIp(server->hostHeader()) ) {
    DEBUG_WM_LN(F("Request redirected to captive portal"));
    server->sendHeader("Location", String("http://") + toStringIp(server->client().localIP()), true);
    server->send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    server->client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

//start up config portal callback
void myWiFiManager::setAPCallback( void (*func)(myWiFiManager* mymyWiFiManager) ) {
  _apcallback = func;
}

//start up save config callback
void myWiFiManager::setSaveConfigCallback( void (*func)(void) ) {
  _actioncallback = func;
}

//sets a custom element to add to head, like a new style tag
void myWiFiManager::setCustomHeadElement(const char* element) {
  _customHeadElement = element;
}

//if this is true, remove duplicated Access Points - defaut true
void myWiFiManager::setRemoveDuplicateAPs(boolean removeDuplicates) {
  _removeDuplicateAPs = removeDuplicates;
}



template <typename Generic>
void myWiFiManager::DEBUG_WM(Generic text) {
  if (_debug) {
    if(_newline_debug){
      Serial.print("*WM: ");
      _newline_debug = false;
    }
    Serial.print(text);
  }
}

template <typename Generic>
void myWiFiManager::DEBUG_WM_LN(Generic text) {
  if (_debug) {
    if(_newline_debug){
      Serial.print("*WM: ");
      _newline_debug = false;
    }
    Serial.println(text);
    _newline_debug = true;
  }
}

int myWiFiManager::getRSSIasQuality(int RSSI) {
  int quality = 0;

  if (RSSI <= -100) {
    quality = 0;
  } else if (RSSI >= -50) {
    quality = 100;
  } else {
    quality = 2 * (RSSI + 100);
  }
  return quality;
}

/** Is this an IP? */
boolean myWiFiManager::isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

/** IP to String? */
String myWiFiManager::toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}
