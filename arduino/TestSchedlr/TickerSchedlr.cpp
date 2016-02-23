#include <Arduino.h>
#include "TickerSchedlr.h"

  long TickerSchedlr::_ticktime = 0;
  
	void TickerSchedlr::updateTickTime() {
	/* just use the arduino epoch */
		_ticktime = millis();
	}

	long TickerSchedlr::getTickTime(){
		 return _ticktime;
	}
	
	TickerSchedlr::TickerSchedlr(){
		this->tasks = new TickerTask[SCHED_BUFFER_SIZE];
		this->_size = SCHED_BUFFER_SIZE;
		updateTickTime();
	};

	TickerSchedlr::TickerSchedlr(uint8_t size){
		this->tasks = new TickerTask[size];
		this->_size = size;
   _ticktime = 0;
		updateTickTime();
	};

	TickerSchedlr::~TickerSchedlr()	{
		// clear all tasks
		int i;
		for (i = 0; i < _size; i++) {
			tasks[i].clear();
			yield();
		}
		// delete the tasks array
		delete[] this->tasks;
		this->tasks = NULL;
		this->_size = 0;
	};

/* flag checking macros */
#define __TASK_IS_NOOP(task) ((task).flag == NOOP)
#define __TASK_IS_IDLE(task) ((task).flag == IDLE)
#define __TASK_IS_REGULAR(task) (!(__TASK_IS_NOOP(task) || __TASK_IS_IDLE(task)))

	void TickerSchedlr::tick(){
		updateTickTime();
		bool idle = true;

		/* run through regular tasks */
		int i;
		for (i = 0; i < _size; i++) {
			TickerTask *task = &tasks[i];

			/* check if the task has been scheduled to run now or in the past */
			if (__TASK_IS_REGULAR(*task)) {
				/* check if the task has been scheduled to execute in the idle window */
				if (task->_next_time <= _ticktime + SCHED_IDLE_WINDOW) {
					idle = false;
				}

				/* task has been scheduled to execute */
				if (task->_next_time <= _ticktime){
					yield();
					task->exec();
					yield();
				}
			}
		}

		/* this is an idle tick -- run through idle tasks */
		if (idle) {
			for (i = 0; i < SCHED_BUFFER_SIZE; i++) {
				TickerTask *task = &tasks[i];

				/* execute idle tasks */
				if (__TASK_IS_IDLE(*task)){
					yield();
					task->exec();
					yield();
				}
			}
		}
	};

	TickerTask * TickerSchedlr::Add(task_fp f, void *t_arg, int flag, uint32_t period){
		int i;
		for (i = 0; i < _size; i++) {
			TickerTask *task = &tasks[i];
			if (~task->is_used && task->Callback == NULL && task->flag == NOOP){
				task->Callback = f;
				task->task_arg = t_arg;
				task->flag = flag;
				task->_period = period;
				task->is_used = true;
				task->_ID = i;
				task->_next_time = _ticktime;
				return task;
			};
		};
		return NULL;
	};
	TickerTask * TickerSchedlr::Add(task_fp f, int flag, uint32_t period){
		Add(f, NULL, flag, period);
	};
	TickerTask * TickerSchedlr::Add(task_fp f, int flag){
		Add(f, NULL, flag, 0);
	};
	TickerTask * TickerSchedlr::Add(task_fp f, void *t_arg, int flag){
		Add(f, t_arg, flag, 0);
	};


	// ============================================================================================

	TickerTask::TickerTask(){
		_last_time = 0;
		_next_time = 0;
		_period = 0;
		_ID = 0;
		_duration = 0;
		Callback = NULL;
		task_arg = NULL;
		is_used = false;
		flag = NOOP;
	};

	TickerTask::~TickerTask(){

	};

	boolean TickerTask::end(){
		// end this task. clear all data in scheduler table
		clear();
		yield();
	};
	
	boolean TickerTask::cont(task_fp f, uint32_t delay, tasktype flag){
		return cont(f, NULL, delay, flag);
	};

	boolean TickerTask::cont(task_fp f, void *t_arg, uint32_t delay, tasktype flag){
		// continue next time with new callback function
		long unow = TickerSchedlr::getTickTime();
		_last_time = unow;
		_next_time = unow + delay;
		Callback = f;
		task_arg = t_arg;
		this->flag = flag;
    this->updated = true;
	};
  
 

	void TickerTask::exec(){
		// execute the task by calling the callback function.
		// we yield first to do some background tasks in the ESP8266
		yield(); // take care of background tasks in the ESP8266
		(*Callback)(task_arg);  // execute task
		yield(); // take care of background tasks in the ESP8266
		if (flag == ONCE){
			clear();  // remove task
			return;
		}
		// update timing
		long unow = TickerSchedlr::getTickTime();
    if (!(this->updated)){
		  this->_last_time = unow;
		  this->_next_time = unow + this->_period; 
    }
    this->updated = false;
		this->_duration = unow - this->_last_time;
	};

	void TickerTask::clear() {
		// reset all data in the task
		_last_time = 0;
		_next_time = 0;
		_period = 0;
		_ID = 0;
		_duration = 0;
		Callback = NULL;
		task_arg = NULL;
		is_used =false;
		flag = NOOP;
	};
