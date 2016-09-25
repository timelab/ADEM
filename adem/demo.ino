extern ESP8266WebServer webserver;

static const char *demo_html =
"<!DOCTYPE html>\n"
"<html>\n"
"  <head>\n"
"    <title>ADEM Demo</title>\n"
"    <meta charset=\"UTF-8\">\n"
"    <meta name=\"apple-mobile-web-app-capable\" content=\"yes\">\n"
"    <script type=\"application/javascript\">\n"
"      function update_all() {\n"
"        var request = new XMLHttpRequest();\n"
"        request.open('GET', '/sensors.json', true);\n"
"        // This following line is needed to tell the server this is an ajax request\n"
"        request.setRequestHeader('X-Requested-With', 'XMLHttpRequest');\n"
"        request.onload = function () {\n"
"          var json = JSON.parse(this.response);\n"
"          document.getElementById('bar').innerHTML = json['barometer']['Pressure'];\n"
"          document.getElementById('tmp').innerHTML = json['barometer']['Temperature'];\n"
"          document.getElementById('hum').innerHTML = json['humidity']['Humidity'];\n"
"          document.getElementById('pm1').innerHTML = json['particulate']['PM1.0'];\n"
"          document.getElementById('pm2').innerHTML = json['particulate']['PM2.5'];\n"
"        };\n"
"        request.send();\n"
"      };\n"
"      setInterval(update_all, 20000);\n"
"    </script>\n"
"    <style> *{-webkit-box-sizing:border-box;-moz-box-sizing:border-box;box-sizing:border-box}html,body{height:100%;margin:0}body{padding-top:20px;font-family:sans-serif;font-weight:700;color:#30a5e7;font-size:28px;line-height:1.5}.table{display:table;width:100%;height:100%}.cell{display:table-cell;vertical-align:middle;width:100%;height:100%}.col{float:left;width:33%}.col-2 p,a.top{border-bottom:1px solid #30a5e7}.txt-r{text-align:right;padding-right:10px}.sub{font-size:.65em}.top{position:absolute}svg.top{top:5%;left:6%}a.top{font-size:.75em;color:#30a5e7;bottom:7%;left:50%;-webkit-transform:translateX(-50%);transform:translateX(-50%);text-decoration:none}@media(max-height:700px){body{font-size:22px}svg.top{top:3%;width:64px}}@media(max-height:450px){.top{display:none}}@media(max-width:750px){svg.top{left:50%;-webkit-transform:translateX(-50%);transform:translateX(-50%)}} </style>\n"
"  </head>\n"
"  <body onload=\"update_all();\">\n"
"    <a href=\"http://ik-adem.be/\" target=\"_blank\" class=\"top\">ik-adem.be</a>\n"
"    <div class=\"table\">\n"
"      <div class=\"cell\">\n"
"        <div class=\"col txt-r\"><p>&#128684;</p><p>&#127981;</p><p>☀️</p><p>&#128167;</p><p>&#9729;</p></div>\n"
"        <div class=\"col col-2 txt-r\"><p id=\"pm1\">N/A</p><p id=\"pm2\">N/A</p><p id=\"tmp\">N/A</p><p id=\"hum\">N/A</p><p id=\"bar\">N/A</p></div>\n"
"        <div class=\"col col-3\"><p>PM<span class=\"sub\">1.0</span></p><p>PM<span class=\"sub\">2.5</span></p><p>°C</p><p>%</p><p>mbar</p></div>\n"
"      </div>\n"
"    </div>\n"
"    <svg class=\"top\" xmlns=\"http://www.w3.org/2000/svg\" x=\"0\" y=\"0\" width=\"94px\" height=\"97px\" viewBox=\"0 0 94 97\"><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M59.707,38.168c0,1.609-1.303,2.912-2.91,2.912 s-2.909-1.303-2.909-2.912c0-1.605,1.302-2.908,2.909-2.908S59.707,36.562,59.707,38.168z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M46.91,91.997c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S46.91,90.392,46.91,91.997z\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M67.66,38.711 74.08,42.564 66.034,47.178\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M47,92 c24.855,0,45-20.145,45-45.002C92,22.146,71.855,2,47,2C22.146,2,2,22.146,2,46.998c0,7.953,2.312,18.233,8.667,25.995 c4.848,5.908,15.083,9.917,25.167,5.25c6.155-2.849,12.924-9.339,15.916-12.503c3.741-3.955,10.948-12.77,13.088-16.424 c7.185-12.273-0.257-23.352-14.627-22.324c-7.94,0.566-12.831,7.312-16.68,16.422c-2.546,6.025-3.522,13.453-2.061,18.061\"/><path fill=\"#FFF\" stroke=\"#30A5E7\" stroke-width=\"2\" stroke-linecap=\"round\" d=\"M35.848,47.939 c0,0-7.844,0.533-11.19-2.15c-1.387-1.112-1.408-2.963-1.123-3.996c0.575-2.083,4.444-2.963,4.444-2.963s-4.574-3.471-3.21-6.623 c0.917-2.121,2.964-2.918,4.728-2.906c4.31,0.028,11.14,6.617,12.1,7.938\"/><path fill=\"none\" stroke=\"#30A5E7\" stroke-width=\"2\" d=\"M35.707,64.168c0,1.609-1.302,2.912-2.91,2.912 s-2.91-1.303-2.91-2.912c0-1.605,1.302-2.908,2.91-2.908S35.707,62.562,35.707,64.168z\"/></svg>\n"
"  </body>\n"
"</html>";

boolean isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

void website() {
  // If we get another request, redirect to http://adem.local/
  if (!isIp(webserver.hostHeader()) && webserver.hostHeader() != String("adem.local")) {
//    webserver.sendHeader("Location", String("http://") + toStringIp(webserver.client().localIP()), true);
    webserver.sendHeader("Location", "http://adem.local/", true);
    webserver.send ( 302, "text/plain", "Redirecting to http://adem.local/");
  } else {
    webserver.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webserver.sendHeader("Pragma", "no-cache");
    webserver.sendHeader("Expires", "-1");
    webserver.send(200, "text/html", demo_html);
  }
}

void sensors() {
  StaticJsonBuffer<256> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  StaticJsonBuffer<256> jsonBufferBar;
  root["barometer"] = jsonBufferBar.parseObject(barometer.report());
  StaticJsonBuffer<256> jsonBufferHum;
  root["humidity"] = jsonBufferHum.parseObject(humidity.report());
  StaticJsonBuffer<256> jsonBufferPar;
  root["particulate"] = jsonBufferPar.parseObject(particulate.report());
  char data[256];
  root.printTo(data, sizeof(data));
  webserver.send(200, "application/json", data);
}