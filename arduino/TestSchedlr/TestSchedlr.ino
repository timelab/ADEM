
#include "TickerSchedlr.h"

TickerSchedlr schedule(200);


int count = 0;
int count2 = 0;
int count3 = 0;
TickerTask * T1 = NULL;
TickerTask * T2 = NULL;
TickerTask * T3 = NULL;
int pin1 = 12;
int pin2 = 14;
int pin3 = 13;

void flip(void *)
{
  int state = digitalRead(12);  // get the current state of GPIO1 pin
  digitalWrite(12, !state);     // set pin to the opposite state
  Serial.print(count);
  Serial.print(" :flip1: ");
  Serial.println(state);
  ++count;
}

void flip2(void *ptr)
{
  int pin = 14;
  int *iptr = (int *)ptr;
  int state = digitalRead(*iptr);  // get the current state of GPIO1 pin
  digitalWrite(14, !state);     // set pin to the opposite state
  Serial.print(count2);
  Serial.print(" :flip2: ");
  Serial.println(state);
  ++count2;
}

void flip3(void *)
{
  int state = digitalRead(13);  // get the current state of GPIO1 pin
  digitalWrite(13, !state);     // set pin to the opposite state
  Serial.print(count3);
  Serial.print(" :flip3: ");
  Serial.println(state);
    Serial.print("last ");Serial.print(T3->_last_time);Serial.print(" next ");    Serial.print(T3->_next_time);Serial.print("  period "); Serial.println(T3->_next_time - T3->_last_time);
  ++count3;
  if (count3 > 10)
  {
    Serial.print(" period for task3  is ");
    Serial.println(T3->_period);
    T3->cont(&flip3,T3->_period*20,PERIODIC);
    Serial.print("last ");Serial.print(T3->_last_time);Serial.print(" next ");    Serial.print(T3->_next_time);Serial.print("  period "); Serial.println(T3->_next_time - T3->_last_time);
    count3 = 0;
  }
 }

void setup() {
  
  Serial.begin(115200);
  
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  T1 =  schedule.Add(&flip, PERIODIC,100);
  T2 =  schedule.Add(&flip2,&pin2, PERIODIC,500);
  T3 =  schedule.Add(&flip3,PERIODIC,1000);
  T1->name = "Task 1";
  T2->name = "Task 2";
  T3->name = "Task 3";
  Serial.print("Task 1 next time is "); Serial.println(T1->_next_time);
  Serial.print("Task 1 period is "); Serial.println(T1->_period);
  Serial.print("Schedule time "); Serial.println(schedule.getTickTime());
  Serial.print("Task 2 next time is "); Serial.println(T2->_next_time);
  Serial.print("Task 2 period is "); Serial.println(T2->_period);
  Serial.print("Schedule time "); Serial.println(schedule.getTickTime());
  Serial.print("Task 3 next time is "); Serial.println(T3->_next_time);
  Serial.print("Task 3 period is "); Serial.println(T3->_period);
  Serial.print("Schedule time "); Serial.println(schedule.getTickTime());
  Serial.println("schedule added");
 
}

// the loop function runs over and over again until power down or reset
void loop() {
schedule.tick();
}
