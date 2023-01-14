/*
 * File:   my_scheduling_lib.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 3 novembre 2022, 11.15
 */

#include "my_scheduling_lib.h"
#include "my_timer_lib.h"

typedef struct
{
    int n;  // number of periods elapsed from last task call
    int N;  // after how many periods should the task be called
    void (*task)(void);  // the task
} Heartbeat;

// The scheduling data that allows to perform the scheduling routine
Heartbeat _scheduling_info[TASK_COUNT];
// The next index for adding a task
int _current_index = 0;

// The timer ms set for the scheduling
int _scheduling_timer_ms;
// The timer that will be used for scheduling
short _timer;


void _scheduler()
{
    for(int i=0; i<TASK_COUNT; ++i)
    {
        if(++_scheduling_info[i].n < _scheduling_info[i].N)
            continue;
        _scheduling_info[i].task();
        _scheduling_info[i].n = 0;
    }
}


void scheduling_init(short timer, int loop_ms)
{
    _timer = timer;
    _scheduling_timer_ms = loop_ms;
    _current_index = 0;
}


void scheduling_add_task(void (*task)(void), int period, int offset)
{
    schedInfo[_current_index++] = (Heartbeat){offset, period/_scheduling_timer_ms, task};
}


void scheduling_loop()
{
    // Initialize the timer used for the schefuling loop
    tmr_setup_period(_timer, _scheduling_timer_ms);

    while (1) 
    {
        scheduler();
        tmr_wait_period(_timer);
    }
}