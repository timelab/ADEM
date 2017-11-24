

#include "store_and_forward.h"

#define SERIAL_BAUD 74880
#define DEBUG

#ifdef DEBUG
#define __LOG(msg) Serial.print(msg)
#define __LOGLN(msg) Serial.println(msg)
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif


storeAndForwardBuf buffer(1000);
long n;
int lID;
bool filling = true;
bool emptying = false;

void setup() {
  
  Serial.begin(SERIAL_BAUD);
  __LOGLN();
  delay(1000);
  __LOGLN("Serial communication... OK");

  __LOGLN();
  __LOGLN("Setup started in DEBUG mode.");

//  buzzer.begin();
  if (buffer.full()) {
    Serial.println("ERROR no memory to buffer data");
  }


  __LOG("Buffer size :"); __LOGLN(buffer.size());
  __LOG("Buffer room :"); __LOGLN(buffer.room());
  __LOG("Buffer available :"); __LOGLN(buffer.available());
  __LOG("Buffer empty :"); __LOGLN(buffer.empty());
  __LOG("Buffer full :"); __LOGLN(buffer.full());
  __LOG("Add size 100");
  buffer.resizeAdd(100);
  __LOG("Buffer size :"); __LOGLN(buffer.size());
  __LOG("Buffer room :"); __LOGLN(buffer.room());
  __LOG("Buffer available :"); __LOGLN(buffer.available());
  __LOG("Buffer empty :"); __LOGLN(buffer.empty());
  __LOG("Buffer full :"); __LOGLN(buffer.full());
  __LOG("Resize -100");
  buffer.resize(1000);
  __LOG("Buffer size :"); __LOGLN(buffer.size());
  __LOG("Buffer room :"); __LOGLN(buffer.room());
  __LOG("Buffer available :"); __LOGLN(buffer.available());
  __LOG("Buffer empty :"); __LOGLN(buffer.empty());
  __LOG("Buffer full :"); __LOGLN(buffer.full());
  __LOG("Resize -100");
  delay(5000);


}

// Main loop takes care of state and transition management
void loop() {
    if (buffer.room() > 100)
        n = random(10,100);
    else
        n = random(10,buffer.room()/2);
    int data[100];
    if (filling){
        __LOG("Add data of size ");__LOGLN(n);
        data[0] = n;
        for (int i=1;i<=n;i++)
        {
            data[i] = i;
        }
        buffer.write((const char*) &data[0],n+1);
        __LOG("Buffer size :"); __LOGLN(buffer.room());
        delay(300);
    }
    if (buffer.room() <= 30){
        emptying = true;
        filling = false;
    }
    if (emptying){
        __LOG("Read buffered report ");
        lID = buffer.peek();
        __LOG(" for ");__LOG(lID);__LOGLN(" bytes");
        
        __LOG("Buffer size :"); __LOGLN(buffer.room());
        int s = buffer.peek((char *) &data[0], (size_t)( lID+1));
        if (s > 0 ) {
            __LOG("buffer read for ");__LOG(s);__LOGLN(" bytes");
        } 
        buffer.ack(); 
        __LOG("Buffer size :"); __LOGLN(buffer.room());
        delay(300);
    }
    if (buffer.empty()){
        emptying = false;
        filling = true;
    }
}