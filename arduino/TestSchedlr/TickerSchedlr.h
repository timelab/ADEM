//#include <Ticker.h>
#include <stdint.h>


#if !defined(__TICKERSCHED_H__)
#define __TICKERSCHED_H__

/* feel free to change this, but it is proportional to time and space complexity for a bunch of operations */
#if !defined(SCHED_BUFFER_SIZE)
#  define SCHED_BUFFER_SIZE 100
#endif

/* the amount of time in milliseconds that the scheduler must be idle in order for idle tasks to be run */
#if !defined(SCHED_IDLE_WINDOW)
# define SCHED_IDLE_WINDOW 200
#endif
//#define DEBUG
#if defined(DEBUG)
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
  boolean updated;
  // make sure that tasks are not created directly so that we can guarantee dynamic object for tasks
  // only pointers to taks objects should exist otherwise tasks will be destroyed when going out of scope
  TickerTask();
  TickerTask(task_fp f, void *t_arg, uint32_t period, int _tasktype);

public:
  volatile bool is_used = false;

  ~TickerTask();
  // factory methods to create tasks
  static TickerTask * createPeriodic(task_fp f, void *t_arg, uint32_t period);
  static TickerTask * createPeriodic(task_fp f, uint32_t period);
  static TickerTask * createIdle(task_fp f, void *t_arg);
  static TickerTask * createIdle(task_fp f);
  static TickerTask * createDelayed(task_fp f, void *t_arg, uint32_t span);
  static TickerTask * createDelayed(task_fp f, uint32_t span);

  /* a time-related variable, which has a different meaning depending on the type */
  long last_time; // last execution time given in millis() units

  /* represents the "next" scheduled time it is meant to run (allows for period tasks) */
  long next_time; // when do we have to run next
  long period;  // what is the periodicity or the delay in milli seconds
  int ID;     // what is the ID in the scheduler table
  long duration;  // how long did it take to run the task
  void *task_arg = NULL;    // arguments for the callback funtion
  String name = "";
  int tasktype = NOOP;  // no operation is the default value

  boolean kill();    // end task and remove from scheduler
  boolean replace(task_fp f);       // continue scheduling replacing callback with give function 
  boolean replace(task_fp f, void *t_arg); // continue scheduling replacing callback with give function 
  boolean replaceDelayed(task_fp f, uint32_t span);        // continue scheduling replacing callback with give function 
  boolean replaceDelayed(task_fp f, void *t_arg, uint32_t span); // continue scheduling replacing callback with give function 

  void delay(uint32_t span);
  void trigger();   // set the task for next round of execution
  void exec();    // execute current task outside scheduler 
  void clear();

};

typedef  TickerTask * ptrTickerTask;

class TickerSchedlr
{
private:
  uint _size;
  TickerTask **tasks = NULL;
  static TickerSchedlr * _schedlr;
  static long _ticktime;
  long next_schedule = 0;
  int16_t idlePtr = 0;

  long updateTickTime();
  TickerSchedlr();
  TickerSchedlr(uint8_t size);

public:
  static TickerSchedlr* Instance();
  static TickerSchedlr* Instance(int size);
  ~TickerSchedlr();

  static long getTickTime();

  //  ptrTickerTask Add(task_fp f, int type, uint32_t period);
  //  ptrTickerTask Add(task_fp f, void *t_arg, int type, uint32_t period);
  //  ptrTickerTask Add(task_fp f, int type);
  //  ptrTickerTask Add(task_fp f, void *t_arg, int type);

  void Add(TickerTask *task);

  void tick();
};

#endif
