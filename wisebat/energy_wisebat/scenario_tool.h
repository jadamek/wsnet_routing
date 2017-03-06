#ifndef SCENARIO_TOOL_H
#define SCENARIO_TOOL_H

#include "wisebat.h"

typedef struct _sequence_info
{
    callback_t callback;
    int label;
} sequence_info_t;


#define scenario_declare(func)\
int func(call_t *c, void *data)\
{ \
    sequence_info_t* __sequinfo;\
    if(!data) { \
        __sequinfo = malloc(sizeof(sequence_info_t)); \
        __sequinfo->label = 0;\
        __sequinfo->callback = func;\
    } else {\
        __sequinfo = (sequence_info_t*)data;\
    }

#define scenario_start()\
    switch(__sequinfo->label) { \
    case 0:

#define scenario_restart() \
    __sequinfo->label = 0; \
    scheduler_add_callback(get_time(), c, __sequinfo->callback, __sequinfo); \
    return 0; } }

#define scenario_end() } } free(__sequinfo);



#define scenario_wait_ms(time) scenario_wait(time*1000000U) 
#define scenario_wait_s(time) scenario_wait(time*1000000000U) 

#define scenario_wait(time) \
    scheduler_add_callback(get_time() + time, c, __sequinfo->callback, __sequinfo); \
    __sequinfo->label = __LINE__; return 0; case __LINE__:

#define scenario_launch(func,c) func(c, 0)


#endif // SCENARIO_TOOL_H
