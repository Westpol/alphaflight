#include "scheduler.h"
#include "timer.h"

#define num_tasks 50
#define min_call_delta 100      // in us

static struct{
    task_t task[num_tasks];
    uint8_t num_registered_tasks;
} scheduler = {0};


void SCHEDULER_LOOP(void){
    uint32_t now = MICROS32();

    for(uint8_t i = 0; i < scheduler.num_registered_tasks; i++){

        if((int32_t)(scheduler.task[i].info.next_execution_timestamp - now) <= 0 && scheduler.task[i].info.activated){      // if task is active and overdue

            task_t* task = &scheduler.task[i];      // save extract task from array to own pointer (less confusion)

            uint32_t timestamp_func_start = MICROS32();
            task->info.running = true;
            uint32_t delta = task->func(&task->info);       // run task
            task->info.running = false;
            // v averaging task execution times v
            task->stat.average_exec_time = (task->stat.average_exec_time * task->stat.filter_value + (uint32_t)(MICROS32() - timestamp_func_start)) / (task->stat.filter_value + 1);

            if(!task->info.dynamic_execution_management){     // task decides next execution time (clamped) or default is being applied
                task->info.next_execution_timestamp += task->info.call_delta_norm;
            }
            else{
                delta = UTILS_MIN_I(UTILS_MAX_I(delta, task->info.call_delta_min), task->info.call_delta_max);      // clamp values to user min / maxes
                
                task->info.next_execution_timestamp += delta;
            }
        }
        
    }
}

int32_t SCHEDULER_REGISTER_TASK(task_func_t func, uint32_t delta_norm, bool dynamic_delta, uint32_t delta_min, uint32_t delta_max, uint32_t max_execution_time, char* task_name){

    if(scheduler.num_registered_tasks >= num_tasks) return -1;

    uint32_t task_num = scheduler.num_registered_tasks++;
    scheduler.task[task_num].info.task_index = task_num;
    scheduler.task[task_num].func = func;
    scheduler.task[task_num].info.dynamic_execution_management = dynamic_delta;
    scheduler.task[task_num].info.call_delta_min = delta_min;
    scheduler.task[task_num].info.call_delta_max = delta_max;
    scheduler.task[task_num].info.call_delta_norm = delta_norm;
    scheduler.task[task_num].info.task_name = task_name;
    scheduler.task[task_num].info.activated = true;

    return task_num;
}