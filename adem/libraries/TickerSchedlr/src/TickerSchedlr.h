/* TickerSchedlr: a task scheduler written for an ESP8266 project in cpp
* Copyright (C) 2016 KAVERS
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* 1. The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
*
* similar ideas found in:
*  https://github.com/ivanseidel/ArduinoThread
*  https://github.com/Toshik/TickerScheduler
*  https://github.com/cyphar/sched
*
*/

//#include <Ticker.h>
#include <stdint.h>
#include <Arduino.h>
#include <inttypes.h>

#if !defined(__TICKERSCHED_H__)
#define __TICKERSCHED_H__

/* feel free to change this, but it is proportional to time and space complexity for a bunch of operations */
#if !defined(SCHED_BUFFER_SIZE)
#  define SCHED_BUFFER_SIZE 100
#endif

// #define DEBUG_SCHED  // uncomment this line to get debug output; be aware of the volume of serial logging it can overflow the buffers
#if defined(DEBUG_SCHED)
#define __LOG(msg) (Serial.print(msg))
#define __LOGLN(msg) (Serial.println(msg))
#else
#define __LOG(msg)
#define __LOGLN(msg)
#endif

/* task type enum */
enum tickertasktype {
	NOOP,      /* do not execute, it may be overwritten by the next register call */
	IDLE,      /* run at the next idle tick (time related variables are ignored, but should be set to 0) */
	ONCE,      /* run once at (the specified time + the time of registering the task) milliseconds */
	PERIODIC,  /* run every (specified time) milliseconds (starting at the first tick after being registered). */
};

typedef void(*task_fp)(void *);


class TickerTask
{
private:
	task_fp Callback = NULL;  // callback points to the function to execute in the task
	bool _updated;
	// make sure that tasks are not created directly so that we can guarantee dynamic object for tasks
	// only pointers to tasks objects should exist otherwise tasks will be destroyed when going out of scope
	TickerTask();
	TickerTask(task_fp f, void *t_arg, uint32_t interval, int _tasktype);

	/* a time-related variable, which has a different meaning depending on the type */
	/* represents the "next" scheduled time it is meant to run (allows for periodic tasks) */
	long _next_time;     // when do we have to run next
	long _last_time;     // last execution time given in millis() units
	long _duration;      // how long did it take to run the task
	
public:
	volatile bool _enabled = true;

	~TickerTask();
	// factory methods to create tasks
	static TickerTask * createPeriodic(task_fp f, void *t_arg, uint32_t interval);
	static TickerTask * createPeriodic(task_fp f, uint32_t interval);
	static TickerTask * createIdle(task_fp f, void *t_arg);
	static TickerTask * createIdle(task_fp f);
	static TickerTask * createDelayed(task_fp f, void *t_arg, uint32_t span);
	static TickerTask * createDelayed(task_fp f, uint32_t span);

	long interval;      // what is the periodicity or the delay in milli seconds
	int ID;         // what is the ID in the scheduler table
	void *task_arg = NULL;  // arguments for the callback funtion
	String name = "";
	int tasktype = NOOP;  // no operation is the default value

	void kill();      // end task and remove from scheduler
	void replace(task_fp f);       // continue scheduling replacing callback with give function 
	void replace(task_fp f, void *t_arg); // continue scheduling replacing callback with give function 
	void replaceDelayed(task_fp f, uint32_t span);        // continue scheduling replacing callback with give function 
	void replaceDelayed(task_fp f, void *t_arg, uint32_t span); // continue scheduling replacing callback with give function 

	void delay(uint32_t span);
	void trigger();   // set the task for next round of execution
	void exec();    // execute current task outside scheduler 
	void clear();
	void setNextTime();
	void suspend();
	void resume();
	inline uint32_t getNextTime() { return _next_time; };
	inline uint32_t getDuration() { return _duration; };
	inline uint32_t getLastTime() { return _last_time; };
};

typedef  TickerTask * ptrTickerTask;

class TickerSchedlr
{
private:
	size_t _size;
	size_t _maxTsk = 0;
	TickerTask **tasks = NULL;
	static TickerSchedlr * _schedlr;
	static uint32_t _ticktime;
	uint32_t _nextSchedule = 0;
	int16_t _idlePtr = 0;

	uint32_t updateTickTime();
	TickerSchedlr();
	TickerSchedlr(uint8_t size);

public:
	static TickerSchedlr* Instance();
	static TickerSchedlr* Instance(int size);
	~TickerSchedlr();

	static uint32_t getTickTime();

	//  ptrTickerTask Add(task_fp f, int type, uint32_t period);
	//  ptrTickerTask Add(task_fp f, void *t_arg, int type, uint32_t period);
	//  ptrTickerTask Add(task_fp f, int type);
	//  ptrTickerTask Add(task_fp f, void *t_arg, int type);

	void Add(TickerTask *task);

	void tick();
};

#endif
