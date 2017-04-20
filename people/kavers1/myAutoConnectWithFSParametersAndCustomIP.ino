#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <myWiFiManager.h>          //https://github.com/tzapu/myWiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

//define your default values here, if there are different values in config.json, they are overwritten.
//length should be max size + 1 

const char HTTP_PARAM[] PROGMEM = "<input id='config' name='config' length=1 placeholder='0' value='0' type='hidden'><input type='checkbox' onclick=\"var e = document.getElementById('config'); e.value = updatebit(e.value,this.checked,0); \">GPS</br><input type='checkbox' onclick=\"var e = document.getElementById('config'); e.value = updatebit(e.value,this.checked,1); \">barometer</br><input type='checkbox' onclick=\"var e = document.getElementById('config'); e.value = updatebit(e.value,this.checked,2); \">humidity</br><input type='checkbox' onclick=\"var e = document.getElementById('config'); e.value = updatebit(e.value,this.checked,3); \">accelero<br/>";

const char demo_html[] PROGMEM =
"<!DOCTYPE html>\n"
"<html>\n"
"  <head>\n"
"    <title>ADEM Demo</title>\n"
"    <meta charset=\"UTF-8\">\n"
"    <meta name=\"apple-mobile-web-app-capable\" content=\"yes\">\n"
"    <script type=\"application/javascript\">\n"
"      var density = 1.5 * Math.pow(10, 12);\n"
"      var pi = 3.14159627;\n"
"      var K = 3531.5;\n"
"      var r1 = 0.15 * Math.pow(10, -6);\n"
"      var vol1 = (4.0 / 3.0) * pi * Math.pow(r1, 3);\n"
"      var mass1 = density * vol1;\n"
"      var r25 = 0.44 * Math.pow(10, -6);\n"
"      var vol25 = (4.0 / 3.0) * pi * Math.pow(r25, 3);\n"
"      var mass25 = density * vol25;\n"
"\n"
"      function PMcount(PPM) {\n"
"        var ratioPPM = PPM / 60000 * 10.0;\n"
"        return 1.1 * Math.pow(ratioPPM, 3) - 3.8 * Math.pow(ratioPPM, 2) + 520 * ratioPPM + 0.62;\n"
"      }\n"
"\n"
"      // Assues density,  shape,  and size of dust  to estimate mass concentration  from particle\n"
"      // count. This method was described in a 2009 paper by Uva, M., Falcone, R., McClellan, A.,\n"
"      // and Ostapowicz, E.            http://wireless.ece.drexel.edu/research/sd_air_quality.pdf\n"
"\n"
"      // begins PM1 mass concentration algorithm\n"
"      function PM1mass(PM1) {\n"
"        return Math.round(PMcount(PM1) * K * mass1 * 100) / 100;\n"
"     }\n"
"\n"
"      function PM25mass(PM25) {\n"
"        return Math.round(PMcount(PM25) * K * mass25 * 100) / 100;\n"
"      }\n"
"\n"
"      function update_all() {\n"
"        var request = new XMLHttpRequest();\n"
"        request.open('GET', '/sensors.json', true);\n"
"        // This following line is needed to tell the server this is an ajax request\n"
"        request.setRequestHeader('X-Requested-With', 'XMLHttpRequest');\n"
"        request.onload = function () {\n"
"          var json = JSON.parse(this.response);\n"
"          document.getElementById('bar').innerHTML = json['barometer']['Pressure'];\n"
"          document.getElementById('tmp').innerHTML = ((json['barometer']['Temperature'] + json['humidity']['Temperature']) / 2).toFixed(2);\n"
"          document.getElementById('hum').innerHTML = json['humidity']['Humidity'];\n"
"          document.getElementById('pm1').innerHTML = (json['particulate']['PM1'] / 100).toFixed(2);\n"
"          document.getElementById('pm2').innerHTML = (json['particulate']['PM2.5'] / 100).toFixed(2);\n"
"          document.getElementById('pm1c').innerHTML = (PMcount(json['particulate']['PM1']) / 100).toFixed(2);\n"
"          document.getElementById('pm2c').innerHTML = (PMcount(json['particulate']['PM2.5']) / 100).toFixed(2);\n"
"          // Count PM2.5 mass together with PM1 mass with its own formula\n"
"          document.getElementById('pm1m').innerHTML = (PM1mass(json['particulate']['PM1'] - json['particulate']['PM2.5']) + PM25mass(json['particulate']['PM2.5'])).toFixed(2);\n"
"          document.getElementById('pm2m').innerHTML = (PM25mass(json['particulate']['PM2.5'])).toFixed(2);\n"
"        };\n"
"        request.send();\n"
"      };\n"
"      setInterval(update_all, 20000);\n"
"    </script>\n"
"    <style>\n"
"      *{ -webkit-box-sizing:border-box; -moz-box-sizing:border-box; box-sizing:border-box }\n"
"      html,body { height:100%; margin:0 }\n"
"      body { padding-top:20px; font-family:sans-serif; font-weight:700; color:#30a5e7; font-size:28px; line-height:1.5 }\n"
"      table.outer { width:100%; height: 100%; }\n"
"      td.outer { align: center; vertical-align: middle; }\n"
"      table.inner { width:100%; }\n"
"      td { display:table-cell; vertical-align:top; width:33%; text-align:right }\n"
"      td.col2 { border-bottom:1px solid #30a5e7; padding-right:10px }\n"
"      td.col3 { text-align:left; padding-left:10px }\n"
"      a.top { border-bottom:1px solid #30a5e7 }\n"
"      .sub { font-size:.65em }\n"
"      .top { position:absolute }\n"
"      svg.top { top:5%; left:6% }\n"
"      a.top { font-size:.75em; color:#30a5e7; bottom:7%; left:50%; -webkit-transform:translateX(-50%); transform:translateX(-50%); text-decoration:none }\n"
"      @media(max-height:700px) {\n"
"        body { font-size:22px }\n"
"        svg.top { top:3%; width:64px }\n"
"      }\n"
"      @media(max-height:450px) {\n"
"        .top { display:none }\n"
"      }\n"
"      @media(max-width:750px) {\n"
"        svg.top { left:50%; -webkit-transform:translateX(-50%); transform:translateX(-50%) }\n"
"      }\n"
"    </style>\n"
"  </head>\n"
"  <body onload=\"update_all();\">\n"
"    <a href=\"http://ik-adem.be/\" target=\"_blank\" class=\"top\">ik-adem.be</a>\n"
"    <table class=\"outer\"><tr><td class=\"outer\">\n"
"      <table class=\"inner\">"
"        <tbody>\n"
"          <tr><td rowspan=\"2\">&#128684;</td><td class=\"col2\" id=\"pm1\">N/A</div></td> <td class=\"col3\">PM<span class=\"sub\">1.0</span></td></tr>\n"
"                                          <tr><td class=\"col2\" id=\"pm1c\">N/A</div></td><td class=\"col3\">pc/.01ft³</td></tr>\n"
"                        <tr><td>&#128168;</td><td class=\"col2\" id=\"pm1m\">N/A</div></td><td class=\"col3\">µg/m³</td></tr>\n"
"          <tr><td rowspan=\"2\">&#127981;</td><td class=\"col2\" id=\"pm2\">N/A</div></td> <td class=\"col3\">PM<span class=\"sub\">2.5</span></td></tr>\n"
"                                          <tr><td class=\"col2\" id=\"pm2c\">N/A</div></td><td class=\"col3\">pc/.01ft³</td></tr>\n"
"                        <tr><td>&#128168;</td><td class=\"col2\" id=\"pm2m\">N/A</div></td><td class=\"col3\">µg/m³</td></tr>\n"
"                                <tr><td>☀️</td><td class=\"col2\" id=\"tmp\">N/A</div></td> <td class=\"col3\">°C</td></tr>\n"
"                        <tr><td>&#128167;</td><td class=\"col2\" id=\"hum\">N/A</div></td> <td class=\"col3\">%</td></tr>\n"
"                          <tr><td>&#9729;</td><td class=\"col2\" id=\"bar\">N/A</div></td> <td class=\"col3\">mbar</td></tr>\n"
"        </tbody>\n"
"      </table>\n"
"    </td></tr></table>\n"
"    <svg class=\"top\" xmlns=\"http://www.w3.org/2000/svg\" x=\"0\" y=\"0\" width=\"94px\" height=\"97px\" viewBox=\"0 0 94 97\"><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M59.707,38.168c0,1.609-1.303,2.912-2.91,2.912 s-2.909-1.303-2.909-2.912c0-1.605,1.302-2.908,2.909-2.908S59.707,36.562,59.707,38.168z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M46.91,91.997c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S46.91,90.392,46.91,91.997z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M67.66,38.711 74.08,42.564 66.034,47.178\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M47,92 c24.855,0,45-20.145,45-45.002C92,22.146,71.855,2,47,2C22.146,2,2,22.146,2,46.998c0,7.953,2.312,18.233,8.667,25.995 c4.848,5.908,15.083,9.917,25.167,5.25c6.155-2.849,12.924-9.339,15.916-12.503c3.741-3.955,10.948-12.77,13.088-16.424 c7.185-12.273-0.257-23.352-14.627-22.324c-7.94,0.566-12.831,7.312-16.68,16.422c-2.546,6.025-3.522,13.453-2.061,18.061\"/><path fill=\"#FFF\" stroke=\"#30A5E7\" stroke-width=\"2\" stroke-linecap=\"round\" d=\"M35.848,47.939 c0,0-7.844,0.533-11.19-2.15c-1.387-1.112-1.408-2.963-1.123-3.996c0.575-2.083,4.444-2.963,4.444-2.963s-4.574-3.471-3.21-6.623 c0.917-2.121,2.964-2.918,4.728-2.906c4.31,0.028,11.14,6.617,12.1,7.938\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M35.707,64.168c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S35.707,62.562,35.707,64.168z\"/></svg>\n"
"  </body>\n"
"</html>";


//flag for saving data
bool shouldSaveConfig = false;
char SSID[32];
int config = 7;
myWiFiManager myWifiManager;
myWiFiManagerParameter * custom_config;
myWiFiManagerParameter * demo;
char pagebuf[1024];
String page = "";

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  sprintf(SSID, "ADEM-%d", ESP.getChipId());
  //clean FS, for testing
  //SPIFFS.format();


  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  //myWiFiManagerParameter custom_config(HTTP_PARAM);
  page += "<table>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<0 & config) != 0 ? "checked": "unchecked";
  page += " value='1'/>";
  page += "</td><td>GPS</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<1 & config) != 0 ? "checked": "unchecked";
  page += " value='2'/>";
  page += "</td><td>barometer</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<2 & config) != 0 ? "checked": "unchecked";
  page += " value='4'/>";
  page += "</td><td>accelero</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<3 & config) != 0 ? "checked": "unchecked";
  page += " value='8'/>";
  page += "</td><td>humidity</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<4 & config) != 0 ? "checked": "unchecked";
  page += " value='16'/>";
  page += "</td><td>neopixel</td></tr>";
  page += "<tr><td><input type='checkbox' name='options'";
  page += (1<<5 & config) != 0 ? "checked": "unchecked";
  page += " value='32'/>";
  page += "</td><td>buzzer</td></tr>";
  page += "</table>";

  const char * ptr = page.c_str();

  custom_config = new myWiFiManagerParameter("config","config","0",2,ptr );
  //myWiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around

  //set config save notify callback
  myWifiManager.setSaveConfigCallback(saveConfigCallback);

  demo = new myWiFiManagerParameter("Demo",SetupDemoPage());
  //add all your parameters here
  myWifiManager.addParameter(custom_config);
  myWifiManager.addParameter(demo);
/*  myWifiManager.addParameter(&custom_SSID2);
  myWifiManager.addParameter(&custom_SSID3);
  myWifiManager.addParameter(&custom_SSID4);
  */
  //reset settings - for testing
  //myWifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  myWifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  myWifiManager.setConnectTimeout(10);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration

  if (!myWifiManager.autoConnect(SSID, "3B#n#nen")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  Serial.print(F("Reading SSID : "));
  Serial.println(WiFi.SSID());
}

String SetupDemoPage(void){
  return FPSTR(demo_html);
}
void loop() {
  
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c != '\n' and c != '\r') {
      switch (c) {
        case 'o':
              Serial.print("Turning WiFi off... ");

              // Disable WIFI
              //wifi_set_sleep_type(LIGHT_SLEEP_T);
              WiFi.disconnect();
              WiFi.mode(WIFI_OFF);
              //KOV WiFi.forceSleepBegin();
              //KOV delay(1); // needed to go to sleep

              Serial.println("OK");
              delay(3000);
              break;
        case 'c':
              WiFi.mode(WIFI_STA);
              // WiFi.begin();
              if (myWifiManager.autoConnect()) {
                Serial.println("client connected = true ");
                // Log that wifi-connection worked
              } 
              else {
                // Log that wifi-connection failed
                Serial.println("client connected = false");
              }
              break;
        case 's' :
              //WiFi.begin();
              if (myWifiManager.startConfigPortal(SSID)) {
                Serial.println("station connected = true ");
              } 
              else {
                 Serial.println("station connected = false ");
              }
              break;
        case 'r' :
              Serial.print(F("Reading SSID : "));
              Serial.println(WiFi.SSID());
              Serial.print(F("Reading Password :"));
              Serial.println(WiFi.psk());
              break;
        case 'v' :
              Serial.print(F("get Value : "));
              Serial.println(custom_config->getValue());
              break;
        case 'i' :
              Serial.print(F("get ID : "));
              Serial.println(custom_config->getID());
              break;
        case 'h' :
              Serial.print(F("get HTML : "));
              Serial.println(custom_config->getCustomHTML());
              break;
      }
    }
  }
}
