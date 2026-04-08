#include "scheduler.h"
#include "timer.h"
#include "usb.h"

#define num_tasks 100
#define min_call_delta 100      // in us

static struct{
    task_t task[num_tasks];
    uint32_t num_registered_tasks;
} scheduler = {0};

#define avgr_delta_filter 10
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
            uint32_t exec_time = MICROS32() - timestamp_func_start;
            task->stat.average_exec_time = (task->stat.average_exec_time * task->stat.filter_value + exec_time) / (task->stat.filter_value + 1);
            task->stat.total_exec_time += exec_time;

            task->stat.average_delta_time = (task->stat.average_delta_time * (avgr_delta_filter - 1) + (timestamp_func_start - task->stat.last_exec_time + (avgr_delta_filter / 2))) / avgr_delta_filter;
            task->stat.last_exec_time = timestamp_func_start;

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

const task_stat_t* SCHEDULER_GET_TASK_STAT_BY_INDEX(uint32_t index){
    if(index > scheduler.num_registered_tasks) return 0;
    return &scheduler.task[index].stat;
}

int32_t SCHEDULER_DISABLE_TASK_BY_INDEX(uint32_t index){
    if(index > scheduler.num_registered_tasks) return -1;
    if(!scheduler.task[index].info.activated){
        return 1;
    }
    scheduler.task[index].info.activated = false;
    return 0;
}

int32_t SCHEDULER_ENABLE_TASK_BY_INDEX(uint32_t index){
    if(index > scheduler.num_registered_tasks) return -1;
    if(scheduler.task[index].info.activated){
        return 1;
    }
    scheduler.task[index].info.activated = true;
    scheduler.task[index].info.next_execution_timestamp = MICROS32();
    return 0;
}

int32_t SCHEDULER_REGISTER_TASK(task_func_t func, uint32_t delta_norm, bool dynamic_delta, uint32_t delta_min, uint32_t delta_max, uint32_t max_execution_time_us, char* task_name){

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

uint32_t SCHEDULER_PRINT_TASK_PAGE(const task_info_t* task){
    if(MICROS32() == 0) return 0; // avoid division by zero
    USB_PRINTLN_BLOCKING("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");   // a few newlines
    USB_PRINTLN_BLOCKING("%-16s | %-15s | %-14s | %-10s |",
                     "Task Name",
                     "Next Execution",
                     "Exec Delta",
                     "CPU Usage");   // print header

    USB_PRINTLN_BLOCKING("------------------------------------------------------------------");  // header seperator
    float cpu_usage_total = 0;

    for(uint32_t i = 0; i < scheduler.num_registered_tasks; i++){
        //if(!scheduler.task[i].info.activated) continue;
        if(task->task_index == i) continue;
        task_t task = scheduler.task[i];
        float cpu_usage = (float)task.stat.total_exec_time / MICROS32();
        cpu_usage_total += cpu_usage;
        USB_PRINTLN_BLOCKING("%-16s | %12lu us | %11d us | %8.2f %% |",
                     task.info.task_name,
                     task.info.next_execution_timestamp,
                     task.stat.average_delta_time,
                     cpu_usage * 100.0f);
    }

    USB_PRINTLN_BLOCKING("------------------------------------------------------------------");  // total seperator
    USB_PRINTLN_BLOCKING("%-16s | %-15s | %-14s | %8.2f %% |",
                    "Total",
                    "",
                    "",
                    cpu_usage_total * 100.0f);   // print header
    return 0;
}