#ifndef _BUFFEREDRESPONSE_H
#define _BUFFEREDRESPONSE_H

#include <Arduino.h>
#include <ESP8266WebServer.h>

class BufferedResponse : public Print {
private:
    uint8_t * buf;
    size_t buffer_size;
    ESP8266WebServer &server;
    size_t pos = 0;

public:
    BufferedResponse(ESP8266WebServer &_server, const char *content_type = "text/html", int code = 200, size_t buffer_size = 4096);
    
    ~BufferedResponse();

    size_t write(const uint8_t *data, size_t len);

    size_t write(uint8_t data);

    template <typename stringType>
    void write(const stringType &str);    

    using Print::write;

    void flush();

    void close();
};


#endif
