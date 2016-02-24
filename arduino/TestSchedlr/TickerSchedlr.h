#include <Ticker.h>
#include <stdint.h>


#if !defined(__TICKERSCHED_H__)
#define __TICKERSCHED_H__

/* feel free to change this, but it is proportional to time and space complexity for a bunch of operations */
#if !defined(SCHED_BUFFER_SIZE)
#	define SCHED_BUFFER_SIZE 100
#endif

/* the amount of time in milliseconds that the scheduler must be idle in order for idle tasks to be run */
#if !defined(SCHED_IDLE_WINDOW)
#	define SCHED_IDLE_WINDOW 200
#endif

/* task type enum */
enum tasktype {
	NOOP,      /* do not execute, it may be overwritten by the next register call */
	IDLE,      /* run at the next idle tick (time related variables are ignored, but should be set to 0) */
	ONCE,      /* run once at (the specified time + the time of registering the task) milliseconds */
	PERIODIC,  /* run every (specified time) milliseconds (starting at the first tick after being registered). */
};

typedef void(*task_fp)(void *);


class TickerTask
{
	
public:
	volatile bool is_used = false;

	TickerTask();
	~TickerTask();
	/* a time-related variable, which has a different meaning depending on the flag */
	long _last_time;

	/* represents the "next" scheduled time it is meant to run (allows for period tasks) */
	long _next_time;	// when do we have to run next
	long _period;		// what is the periodicity
	int _ID;			// what is the ID in the scheduler table
	long _duration;		// how long did it take to run the task
  boolean updated;
	int	flag = NOOP;	// no operation is the default value
	task_fp Callback = NULL;	// callback points to the function to execute in the task
	void *task_arg = NULL;		// arguments for the callback funtion
  String name = "";
  
	//boolean is_used;	// is this task in use ?

	boolean end();		// end task and remove from scheduler
	boolean cont(task_fp f, uint32_t delay, tasktype flag);				// continue scheduling replacing callback with give function 
	boolean cont(task_fp f, void *t_arg, uint32_t delay, tasktype flag); // continue scheduling replacing callback with give function 

	void trigger();		// set the task for next round of execution
	void exec();		// execute current task outside scheduler 
	void clear();
};

class TickerSchedlr
{
private:
	uint _size;
	TickerTask *tasks = NULL;
  

	void handleTicker(void(*f)(void), volatile bool * flag);
	void updateTickTime();

public:
	TickerSchedlr();
	TickerSchedlr(uint8_t size);
	~TickerSchedlr();

  static long _ticktime  ;  
	static long getTickTime();
	TickerTask * Add(task_fp f, int flag, uint32_t period);
	TickerTask * Add(task_fp f, void *t_arg, int flag, uint32_t period);
	TickerTask * Add(task_fp f, int flag);
	TickerTask * Add(task_fp f, void *t_arg, int flag);

	void tick();
};


#endif
