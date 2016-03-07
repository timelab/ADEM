#include <Arduino.h>
#include "TickerSchedlr.h"

long TickerSchedlr::_ticktime = 0;
TickerSchedlr *TickerSchedlr::_schedlr = 0;

/* type checking macros */
#define __TASK_IS_NOOP(task) ((task).tasktype == NOOP)
#define __TASK_IS_IDLE(task) ((task).tasktype == IDLE)
#define __TASK_IS_ONCE(task) ((task).tasktype == ONCE)
#define __TASK_IS_REGULAR(task) (!(__TASK_IS_NOOP(task) || __TASK_IS_IDLE(task)))

// singleton pattern for scheduler
TickerSchedlr *TickerSchedlr::Instance() {
  __LOGLN("Get scheduler instance");
  if (_schedlr == 0){
    _schedlr = new TickerSchedlr;
  }
  return _schedlr;
}

TickerSchedlr *TickerSchedlr::Instance(int size) {
  __LOGLN("Get scheduler sized instance");
  if (_schedlr == 0){
    _schedlr = new TickerSchedlr(size);
  }
  return _schedlr;
}

long TickerSchedlr::updateTickTime() {
  /* just use the arduino epoch */
  //  __LOG("Update tick time ");
  _ticktime = millis();
  //  __LOGLN( _ticktime);
  return _ticktime;
}

long TickerSchedlr::getTickTime(){
  __LOG("Get tick time ");__LOGLN(_ticktime);
  return _ticktime;
}

TickerSchedlr::TickerSchedlr(){
  __LOGLN("Create scheduler");
  //TickerSchedlr(SCHED_BUFFER_SIZE);
  this->tasks = new TickerTask*[SCHED_BUFFER_SIZE];
  this->_size = SCHED_BUFFER_SIZE;
  int i;
  for ( i = 0; i< SCHED_BUFFER_SIZE; i++)
    tasks[i] = 0;
  updateTickTime();
};

TickerSchedlr::TickerSchedlr(uint8_t size){
  __LOG("Create sized scheduler :");__LOGLN(size);
  this->tasks = new TickerTask*[size];
  int i;
  for (i = 0; i< size; i++)
    tasks[i] = 0;
  this->_size = size;
  updateTickTime();
};

TickerSchedlr::~TickerSchedlr() {
  __LOGLN("Destroy scheduler");
  // clear all tasks
  int i;
  for (i = 0; i < _size; i++) {
    if (tasks[i] != 0)
      tasks[i]->clear();
    yield();
  }
  // delete the tasks array
  delete[] this->tasks;
  this->tasks = 0;
  this->_size = 0;
};

void TickerSchedlr::tick(){
  updateTickTime();
  bool idle = true;
  long task_time = 0;
  __LOG("Scheduler tick "); __LOGLN(_ticktime);
  /* run through regular tasks */
  for (int i = 0; i < _size; i++) {
    TickerTask *task = tasks[i];
    if (task){
      __LOG(" "); __LOG(task->name);__LOG("->");__LOG(task->tasktype);
      /* check if the task has been scheduled to run now or in the past */
      if (task && __TASK_IS_REGULAR(*task)) {
        __LOG(" is a regular task"); 
        /* task has been scheduled to execute */
        if (task->next_time <= _ticktime){
          yield();
          task->exec();
          yield();
          if (__TASK_IS_ONCE(*task))
            task->tasktype = NOOP;
          task_time = task->next_time;
        }
        // store the minimal ticktime for the next task to schedule
        if (task->next_time < next_schedule || next_schedule <= _ticktime)
            next_schedule = task->next_time;
      }
      else
        if (task && __TASK_IS_NOOP(*task)){
          // ready to delete the task
          __LOG(" clear task ");__LOG(task->name);
          task->clear();
          delete task;
          tasks[i] = 0;
        }
        __LOGLN();
    }
  }
  // when next_schedule < current time then we can execute the IDLE tasks
  // we keep track what IDLE task was executed last to prevent starvation of tasks
  int i = 0;
  __LOG(" next_schedule "); __LOG(next_schedule); __LOG(" < ? "); __LOGLN(updateTickTime());
  while (next_schedule > updateTickTime() && i < _size) {
    TickerTask * task = tasks[idlePtr++];
    // execute idle tasks 
    if (task && __TASK_IS_IDLE(*task)){
        __LOGLN(" IDLE tasks");
        yield();
        task->exec();
        yield();
    }
    i++;
    idlePtr = (idlePtr % _size); // circular buffer pointer
  }
};

void TickerSchedlr::Add(TickerTask *task){
  __LOG("Add task to schedule : ");
  for (int i = 0; i < _size; i++) {
    if (tasks[i] == 0){
      tasks[i] = task;
      task->ID = i;
      if (task->next_time < next_schedule || next_schedule <= _ticktime)
            next_schedule = task->next_time;
      __LOGLN(i);
      return;
    }
  }
  task->ID = -1;  // mark the task to flag the error. There is no space to add a task.
}

// ============================================================================================
//
//
//
// ============================================================================================

TickerTask::TickerTask(){
  __LOGLN("Create task");
  period = 0;
  ID = 0;
  duration = 0;
  Callback = NULL;
  task_arg = NULL;
  is_used = false;
  tasktype = NOOP;
  TickerSchedlr::Instance()->Add(this);
  last_time = TickerSchedlr::getTickTime();
  next_time = last_time;
}

TickerTask::TickerTask(task_fp f, void *t_arg, uint32_t Period, int _tasktype){
  __LOGLN("Create task with function pointer");
  period = Period;
  ID = 0;
  duration = 0;
  Callback = f;
  task_arg = t_arg;
  is_used = true;
  tasktype = _tasktype;
  TickerSchedlr::Instance()->Add(this);
  last_time = TickerSchedlr::getTickTime();
  next_time = last_time + Period;
}

TickerTask::~TickerTask(){
  is_used = false;
  //delete task_arg;
};

TickerTask * TickerTask::createPeriodic(task_fp f, void *t_arg, uint32_t period){
  __LOGLN("Create PERIODIC task");
  __LOG("PERIODIC ");__LOGLN(PERIODIC);
  TickerTask * ptr = new TickerTask(f, t_arg, period, PERIODIC);
  __LOG("task type ");__LOGLN(ptr->tasktype);
  __LOG("task ID ");__LOGLN(ptr->ID);
  if (ptr->ID < 0) {
    delete (ptr);
    return 0;
  }
  else
    return ptr;
};

TickerTask * TickerTask::createPeriodic(task_fp f, uint32_t period){
  __LOGLN("Create PERIODIC task");
  __LOG("PERIODIC ");__LOGLN(PERIODIC);
  TickerTask * ptr = new TickerTask(f, 0, period, PERIODIC);
  __LOG("task type ");__LOGLN(ptr->tasktype);
  __LOG("task ID ");__LOGLN(ptr->ID);
  if (ptr->ID < 0) {
    delete (ptr);
    return 0;
  }
  else
    return ptr;
};

TickerTask * TickerTask::createIdle(task_fp f, void *t_arg){
  __LOGLN("Create IDLE task");
  __LOG("IDLE ");__LOGLN(IDLE);
  TickerTask * ptr = new TickerTask(f, t_arg, 0, IDLE);
  __LOG("task type ");__LOGLN(ptr->tasktype);
  __LOG("task ID ");__LOGLN(ptr->ID);
  if (ptr->ID < 0) {
    delete (ptr);
    return 0;
  }
  else
    return ptr;
};

TickerTask * TickerTask::createIdle(task_fp f){
  __LOGLN("Create IDLE task");
  __LOG("IDLE ");__LOGLN(IDLE);
  TickerTask * ptr = new TickerTask(f, 0, 0, IDLE);
  __LOG("task type ");__LOGLN(ptr->tasktype);
  __LOG("task ID ");__LOGLN(ptr->ID);
  if (ptr->ID < 0) {
    delete (ptr);
    return 0;
  }
  else
    return ptr;
};

TickerTask * TickerTask::createDelayed(task_fp f, void *t_arg, uint32_t span){
  __LOGLN("Create DELAYED task");
  TickerTask * ptr = new TickerTask(f, t_arg, span, ONCE);
  if (ptr->ID < 0) {
    delete (ptr);
    return 0;
  }
  else
    return ptr;
};

TickerTask * TickerTask::createDelayed(task_fp f, uint32_t span){
  __LOGLN("Create DELAYED task");
  TickerTask * ptr = new TickerTask(f, 0, span, ONCE);
  if (ptr->ID < 0) {
    delete (ptr);
    return 0;
  }
  else
    return ptr;
};


boolean TickerTask::kill(){
  // end this task. clear all data in scheduler table
  clear();
  yield();
};

void TickerTask::delay(uint32_t delay){
  __LOG("delay task "); __LOG(name); __LOG(" for "); __LOGLN(delay);
  long unow = TickerSchedlr::getTickTime();
  last_time = unow;
  next_time = unow + delay;
  updated = true; // indicate we already updated the scheduling properties
};

boolean TickerTask::replace(task_fp f){
  return replaceDelayed(f, 0);
};

boolean TickerTask::replace(task_fp f, void *t_arg){
  // continue next time with new callback function
  __LOG("replace task "); __LOGLN(name);
  Callback = f;
  task_arg = t_arg;
};

boolean TickerTask::replaceDelayed(task_fp f, uint32_t delay){
  return replaceDelayed(f, 0, delay);
};

boolean TickerTask::replaceDelayed(task_fp f, void *t_arg, uint32_t delay){
  __LOG("replace task "); __LOG(name); __LOG(" and delay for "); __LOGLN(delay);
  // continue next time with new callback function
  long unow = TickerSchedlr::getTickTime();
  last_time = unow;
  next_time = unow + delay;
  Callback = f;
  task_arg = t_arg;
  this->updated = true; // indicate we already updated the scheduling properties
};


void TickerTask::exec(){
  // execute the task by calling the callback function.
  // we yield first to do some background tasks in the ESP8266
  Serial.print("execute task "); Serial.println(name);
  __LOG("execute task "); __LOGLN(name);
  yield(); // take care of background tasks in the ESP8266
  (*Callback)(task_arg);  // execute task
  yield(); // take care of background tasks in the ESP8266
  if (tasktype == ONCE){
    clear();  // remove task
    return;
  }
  // update timing
  long unow = TickerSchedlr::getTickTime();
  this->duration = unow - this->last_time;
  if (!(this->updated)){  // update the scheduling properties if not done yet
    this->last_time = unow;
    this->next_time = unow + this->period;
  }
  this->updated = false;  // clear update flag for next itteration
};

void TickerTask::clear() {
  // reset all data in the task
  last_time = 0;
  next_time = 0;
  period = 0;
  ID = 0;
  duration = 0;
  Callback = NULL;
  task_arg = NULL;
  is_used = false;
  tasktype = NOOP;  // mark the task as "no operation", the scheduler will delete it and remove it from the scheduler table
};

