#include "BufferedResponse.h"

BufferedResponse::BufferedResponse(ESP8266WebServer &_server, const char *content_type, int code, size_t _buffer_size)
    :server(_server),  buffer_size(_buffer_size) {
  buf = (uint8_t  *) malloc(buffer_size);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  if (content_type != NULL)
    server.sendHeader("Content-Type", content_type, true);

  server.sendHeader("Cache-Control", "no-cache");
  server.send(code);
}

BufferedResponse::~BufferedResponse() {
  if (pos != 0)
    close();
  delete buf;
}

size_t BufferedResponse::write(const uint8_t *data, size_t len) {
  const size_t buffer_write_size = buffer_size - 1;
  size_t data_pos = 0;
  Serial.println("buffered write data");
  while(len) {
    //no more space? -> flush
    if((buffer_write_size - pos) == 0){
      flush();
      Serial.println(" flush buffer");
    }
    yield();
    //how much can we write?
    size_t copy_len = std::min(buffer_write_size - pos, len);
    memcpy((void *) (buf + pos), (const void *) (data + data_pos), copy_len);
    pos += copy_len;
    data_pos += copy_len;
    len -= copy_len;
  }
}

void BufferedResponse::flush() {
  //Flush buffer
  if (pos > 0) {
    buf[pos] = 0;  // end of string mark
    server.sendContent(
        (const char *) &buf[0]); //Actually this is a conversion to String - not sure if this is a good idea?
    pos = 0;
  }
}

void BufferedResponse::close() {
  //Flush buffer
  flush();
  //End connection
  server.sendContent("");
}

size_t BufferedResponse::write(uint8_t data) {
  return write(&data, 1);
}

template <typename stringType>
void BufferedResponse::write(const stringType &str)
{
  write(str.c_str(), str.length());
}