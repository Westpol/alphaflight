
#include "common.h"

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

typedef enum{
    SCHEDULER_OKAY,
    SCHEDULER_REGISTERING_MAX_TASKS_REACHED,
    SCHEDULER_TIMER_PROBLEM

} SCHEDULER_RETURN_TYPE;

typedef struct{
    uint32_t average_exec_time;     // time a process takes in us
    uint32_t max_exec_time;     // max time a process takes in us (gets cleared every n seconds by special function)
    uint32_t total_exec_time;       // all execution deltas added together
    uint32_t run_count;     // gets increased every time task gets executed
    uint32_t deadline_miss_count;       // misses execution time by more than 10 us
    uint32_t max_lateness;      // absolute maximum overdue time
    uint32_t overrun_count;     // how often a task violates their max and min deltas
    uint8_t cpu_percent;
    uint32_t filter_value;      // averaging filter value [calculated: ((base delta value) * filter_value + new delta value) / filter_value + 1]
} task_stat_t;

typedef struct{
    uint32_t next_execution_timestamp;
    
    uint32_t call_delta_max;        // given while registering task
    uint32_t call_delta_min;
    uint32_t call_delta_norm;

    volatile bool dynamic_execution_management;     // given while registering task, can be changed by wwdg, activates using task given deltas

    volatile bool running;      // for wwdg function, true while task executes
    volatile bool exit_loops;       // standard false. **EVERY LOOP SHOULD HAVE IT APPENDED WITH AN AND TO EXIT LOOPS IN WORST CASE**
    volatile bool activated;        // activate / deactivate task
    uint32_t task_index;
    char* task_name;
} task_info_t;

typedef uint32_t (*task_func_t)(const task_info_t *task);

typedef struct{
    task_stat_t stat;
    task_info_t info;
    task_func_t func;
} task_t;

#endif