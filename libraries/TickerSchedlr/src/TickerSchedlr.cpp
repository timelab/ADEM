#include <Arduino.h>
#include "TickerSchedlr.h"

uint32_t TickerSchedlr::_ticktime = 0;
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

uint32_t TickerSchedlr::getTickTime(){
	__LOG("Get tick time "); __LOGLN(_ticktime);
	return _ticktime;
}

TickerSchedlr::TickerSchedlr(){
	__LOGLN("Create scheduler");
	//TickerSchedlr(SCHED_BUFFER_SIZE);
	this->tasks = new TickerTask*[SCHED_BUFFER_SIZE];
	this->_size = SCHED_BUFFER_SIZE;
	int i;
	for (i = 0; i< SCHED_BUFFER_SIZE; i++)
		tasks[i] = 0;
	updateTickTime();
};

TickerSchedlr::TickerSchedlr(uint8_t size){
	__LOG("Create sized scheduler :"); __LOGLN(size);
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
	__LOG("Scheduler tick "); __LOGLN(_ticktime);
	/* run through regular tasks */
	for (int i = 0; i <= _maxTsk; i++) {
		TickerTask *task = tasks[i];
		if (task){
			__LOG(" "); __LOG(task->name); __LOG("->"); __LOG(task->tasktype);
			/* check if the task has been scheduled to run now or in the past */
			if (task && __TASK_IS_REGULAR(*task) && task._enabled) {
				__LOG(" is a regular task");
				/* task has been scheduled to execute */
				if (task->getNextTime() <= _ticktime){
					yield();
					task->exec();
					yield();
					if (__TASK_IS_ONCE(*task))
						task->tasktype = NOOP;
				}
				// store the minimal ticktime for the next task to schedule
				if (task->getNextTime() < _nextSchedule || _nextSchedule <= _ticktime)
					_nextSchedule = task->getNextTime();
			}
			else
				if (task && __TASK_IS_NOOP(*task)){
					// ready to delete the task
					__LOG(" clear task "); __LOG(task->name);
					task->clear();
					delete task;
					tasks[i] = 0;
				}
			__LOGLN();
		}
		else
			if (_maxTsk == i + 1)
				_maxTsk--; // shrink itteration size of scheduler since last task is empty
	}
	// when _nextSchedule < current time then we can execute the IDLE tasks
	// we keep track what IDLE task was executed last to prevent starvation of tasks
	int i = 0;
	__LOG(" next_schedule "); __LOG(_nextSchedule); __LOG(" < ? "); __LOGLN(updateTickTime());
	while (_nextSchedule > updateTickTime() && i < _maxTsk) {
		TickerTask * task = tasks[_idlePtr++];
		// execute idle tasks 
		if (task && __TASK_IS_IDLE(*task) && task._enabled){
			__LOGLN(" IDLE tasks");
			yield();
			task->exec();
			yield();
		}
		i++;
		_idlePtr = (_idlePtr % _size); // circular buffer pointer
	}
};

void TickerSchedlr::Add(TickerTask *task){
	__LOG("Add task to schedule : ");
	for (int i = 0; i < _size; i++) {
		if (tasks[i] == 0){
			tasks[i] = task;
			task->ID = i;
			if (task->getNextTime() < _nextSchedule || _nextSchedule <= _ticktime)
				_nextSchedule = task->getNextTime();
			__LOGLN(i);
			if (_maxTsk < i)
				_maxTsk = i;
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
	interval = 0;
	ID = 0;
	_duration = 0;
	Callback = NULL;
	task_arg = NULL;
	tasktype = NOOP;
	TickerSchedlr::Instance()->Add(this);
	_last_time = TickerSchedlr::getTickTime();
	_next_time = _last_time;
}

TickerTask::TickerTask(task_fp f, void *t_arg, uint32_t period, int _tasktype){
	__LOGLN("Create task with function pointer");
	interval = period;
	ID = 0;
	_duration = 0;
	Callback = f;
	task_arg = t_arg;
	tasktype = _tasktype;
	TickerSchedlr::Instance()->Add(this);
	_last_time = TickerSchedlr::getTickTime();
	_next_time = _last_time + interval;
}

TickerTask::~TickerTask(){
	// TODO do we have to delete the task_arg  object ?
	// if (task_arg)
	//    delete task_arg;
};

TickerTask * TickerTask::createPeriodic(task_fp f, void *t_arg, uint32_t period){
	__LOGLN("Create PERIODIC task");
	__LOG("PERIODIC "); __LOGLN(PERIODIC);
	TickerTask * ptr = new TickerTask(f, t_arg, period, PERIODIC);
	__LOG("task type "); __LOGLN(ptr->tasktype);
	__LOG("task ID "); __LOGLN(ptr->ID);
	if (ptr->ID < 0) {
		delete (ptr);
		return 0;
	}
	else
		return ptr;
};

TickerTask * TickerTask::createPeriodic(task_fp f, uint32_t period){
	__LOGLN("Create PERIODIC task");
	__LOG("PERIODIC "); __LOGLN(PERIODIC);
	TickerTask * ptr = new TickerTask(f, 0, period, PERIODIC);
	__LOG("task type "); __LOGLN(ptr->tasktype);
	__LOG("task ID "); __LOGLN(ptr->ID);
	if (ptr->ID < 0) {
		delete (ptr);
		return 0;
	}
	else
		return ptr;
};

TickerTask * TickerTask::createIdle(task_fp f, void *t_arg){
	__LOGLN("Create IDLE task");
	__LOG("IDLE "); __LOGLN(IDLE);
	TickerTask * ptr = new TickerTask(f, t_arg, 0, IDLE);
	__LOG("task type "); __LOGLN(ptr->tasktype);
	__LOG("task ID "); __LOGLN(ptr->ID);
	if (ptr->ID < 0) {
		delete (ptr);
		return 0;
	}
	else
		return ptr;
};

TickerTask * TickerTask::createIdle(task_fp f){
	__LOGLN("Create IDLE task");
	__LOG("IDLE "); __LOGLN(IDLE);
	TickerTask * ptr = new TickerTask(f, 0, 0, IDLE);
	__LOG("task type "); __LOGLN(ptr->tasktype);
	__LOG("task ID "); __LOGLN(ptr->ID);
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


bool TickerTask::kill(){
	// end this task. clear all data in scheduler table
	clear();
	yield();
};

void TickerTask::suspend(){
	_enabled = false;
}

void TickerTask::enable(){
	_enabled = true;
}

void TickerTask::delay(uint32_t delay){
	__LOG("delay task "); __LOG(name); __LOG(" for "); __LOGLN(delay);
	long unow = TickerSchedlr::getTickTime();
	_last_time = unow;
	_next_time = unow + delay;
	_updated = true; // indicate we already updated the scheduling properties
};

bool TickerTask::replace(task_fp f){
	return replaceDelayed(f, 0);
};

bool TickerTask::replace(task_fp f, void *t_arg){
	// continue next time with new callback function
	__LOG("replace task "); __LOGLN(name);
	Callback = f;
	task_arg = t_arg;
};

bool TickerTask::replaceDelayed(task_fp f, uint32_t delay){
	return replaceDelayed(f, 0, delay);
};

bool TickerTask::replaceDelayed(task_fp f, void *t_arg, uint32_t delay){
	__LOG("replace task "); __LOG(name); __LOG(" and delay for "); __LOGLN(delay);
	// continue next time with new callback function
	uint32_t unow = TickerSchedlr::getTickTime() + delay;
	_last_time = unow;
	_next_time = unow + delay;
	Callback = f;
	task_arg = t_arg;
	this->_updated = true; // indicate we already updated the scheduling properties
};

void TickerTask::setNextTime(){
	long unow = TickerSchedlr::getTickTime();
	_duration = unow - _last_time;
	_last_time = _next_time;
	_next_time = unow + interval;
}
void TickerTask::trigger(){
	_next_time = TickerSchedlr::getTickTime(); //execute next round in the scheduler
}

void TickerTask::exec(){
	// execute the task by calling the callback function.
	// we yield first to do some background tasks in the ESP8266
	//Serial.print("execute task "); Serial.println(name);
	__LOG("execute task "); __LOGLN(name);
	yield(); // take care of background tasks in the ESP8266
	(*Callback)(task_arg);  // execute task
	yield(); // take care of background tasks in the ESP8266
	if (tasktype == ONCE){
		clear();  // clear the task and mark for deletion
		return;
	}
	// update timing
	if (!(this->_updated))  // update the scheduling properties if not done yet
		setNextTime();

	this->_updated = false;  // clear update flag for next itteration
};

void TickerTask::clear() {
	// reset all data in the task
	_last_time = 0;
	_next_time = 0;
	interval = 0;
	ID = 0;
	_duration = 0;
	Callback = NULL;
	tasktype = NOOP;  // mark the task as "no operation", the scheduler will delete it and remove it from the scheduler table
};

