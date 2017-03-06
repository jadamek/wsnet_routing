/**
 *  \file   linear.c
 *  \brief  linearly decreasing battery
 *  \author Guillaume Chelius
 *  \date   2007
 **/

#include "wisebat.h"
#include "stdarg.h"
#include "wisebat_das.h"


#define S_TO_NS(s) ((s)*1000000000)
#define MS_TO_NS(ms) ((ms)*1000000)
#define US_TO_NS(us) ((us)*1000)

/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "WiSeBat battery",
    "Quentin BRAMAS",
    "0.1",
    MODELTYPE_ENERGY, 
    {NULL, 0}
};



/* ************************************************** */
/* ************************************************** */
typedef struct _wisebat_data {
    double initial;
    double nominal_discharge_current;
    interpolation_points_t internal_resistance;
    interpolation_points_t voltage_characteristic;
    interpolation_points_t capacity_characteristic;
    double cut_off_voltage;
    uint64_t voltage_latency;

} wisebat_data_t;

typedef struct _nodedata {
    double residual;
    double last_current;
    double last_voltage;
    double voltage_min;
    double voltage_max;
    double voltage_average;
    int    debug;
    FILE * logFile;
    int log_level;
    uint64_t log_interval;
    uint64_t last_log;
    uint64_t last_update;
    component_t radio_component;
    void* components;
    void* components_private;
    void* after_components_updates_callbacks;
    void* before_component_consume_callbacks;
    event_t * next_event;
    int is_dying;
} nodedata_t;


/** das_component_t - to save in a das which sensor is being used **/
typedef struct _das_component{
    component_t* component_ptr;
    component_mode_t mode;
    call_t call;
} das_component_t;

typedef struct _das_data_ptr{
    void * data;
} das_data_ptr_t;


const uint64_t ONE_HOUR = 3600000000000;
const double ONE_HOUR_D = 3600000000000.0;
const uint64_t AUTO_UPDATE_INTERVALLE = 100000;

void print_human_time(uint64_t time)
{
    printf("%lum %lus %lums %.3fus ", (time/60000000000) , (time/1000000000) % 60, (time/1000000) % 1000, fmod((time/1000.0), 1000.0));
}
void print_debug(call_t * c, const char* format, ...)
{
    va_list args;
    printf("%d ", c->node);
    print_human_time(get_time());
    printf(" ");
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf("\n");
}


/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {

    wisebat_data_t * wisebat_data = malloc(sizeof(wisebat_data_t));
    param_t *param;

    wisebat_data->initial = 10;
    wisebat_data->nominal_discharge_current = 0.5;
    wisebat_data->internal_resistance.n = 0;
    wisebat_data->cut_off_voltage = 2.0;
    wisebat_data->voltage_latency = 1;

    /* get parameters */
    das_init_traverse(params);
    while ((param = (param_t *) das_traverse(params)) != NULL) {
        if (!strcmp(param->key, "nominal-discharge-current")) {
            if (get_param_double(param->value, &(wisebat_data->nominal_discharge_current))) {
                fprintf(stderr, "[WiseBat] init(), Error: Unable to resolve \"nominal-discharge-current\" parameter\n");
                goto error;
            }
        }
        if (!strcmp(param->key, "voltage-latency")) {
            if (get_param_time(param->value, &(wisebat_data->voltage_latency))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "internal-resistance")) {
            if (get_param_interpolation_points(param->value, &(wisebat_data->internal_resistance))) {
                fprintf(stderr, "[WiseBat] init(), Error: Unable to resolve \"internal-resistance\" parameter\n");
                goto error;
            }
        }
        if (!strcmp(param->key, "voltage-characteristic")) {
            if (get_param_interpolation_points(param->value, &(wisebat_data->voltage_characteristic))) {
                fprintf(stderr, "[WiseBat] init(), Error: Unable to resolve \"voltage-characteristic\" parameter\n");
                goto error;
            }
        }
        if (!strcmp(param->key, "capacity-characteristic")) {
            if (get_param_interpolation_points(param->value, &(wisebat_data->capacity_characteristic))) {
                fprintf(stderr, "[WiseBat] init(), Error: Unable to resolve \"capacity-characteristic\" parameter\n");
                goto error;
            }
        }
        if (!strcmp(param->key, "cut-off-voltage")) {
            if (get_param_double(param->value, &(wisebat_data->cut_off_voltage))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "energy")) {
            if (get_param_double(param->value, &(wisebat_data->initial))) {
                goto error;
            }
        }
    }

    if(wisebat_data->voltage_latency < 1) {
        wisebat_data->voltage_latency = 1;
    }

    set_entity_private_data(c, wisebat_data);
    return 0;

error:
   free(wisebat_data);
   return -1;
}

int destroy(call_t *c) {
    return 0;
}

void register_component(call_t *c, component_t *component, component_mode_t mode);
component_mode_t get_component_mode(call_t *c, component_t *component);

/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
    nodedata_t * nodedata = malloc(sizeof(nodedata_t));
    param_t *param;

    wisebat_data_t * wisebat_data = get_entity_private_data(c);

    /* default values */
    nodedata->residual  = wisebat_data->initial;
    nodedata->debug   = 0;
    nodedata->logFile = 0;
    nodedata->log_level = 3;
    nodedata->log_interval = 0;
    nodedata->last_log = 0;
    nodedata->last_update = 0;
    nodedata->last_current = 0;
    nodedata->last_voltage = 3.0;
    nodedata->voltage_min = INFINITY;
    nodedata->voltage_max = 0;
    nodedata->voltage_average = 0;
    nodedata->next_event = NULL;
    nodedata->components   = das_create();
    nodedata->components_private   = das_create();
    nodedata->after_components_updates_callbacks = das_create();
    nodedata->before_component_consume_callbacks = das_create();
    nodedata->is_dying = 0;
   /* get parameters */
    das_init_traverse(params);
    while ((param = (param_t *) das_traverse(params)) != NULL) {
        if (!strcmp(param->key, "debug")) {
            if (get_param_integer(param->value, &(nodedata->debug))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "log")) {
            nodedata->logFile = fopen(param->value, "w+");
            if(!nodedata->logFile)
            {
                fprintf(stderr, "Warning: Unable to open the log file %s\n", param->value);
            }
            fprintf(nodedata->logFile, "type time node value\n");
            fflush(nodedata->logFile);
        }
        if (!strcmp(param->key, "log-level")) {
            if (get_param_integer(param->value, &(nodedata->log_level))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "log-interval")) {
            if (get_param_time(param->value, &(nodedata->log_interval))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "energy")) {
            if (get_param_double(param->value, &(nodedata->residual))) {
                goto error;
            }
        }
    }
    if(nodedata->logFile)
    {
        fprintf(nodedata->logFile,"residual 0 %d %.9f \n", c->node, nodedata->residual);
        fflush(nodedata->logFile);
    }

    if(nodedata->residual > wisebat_data->initial)
    {
        fprintf(stderr, "Warning: the initial energy of node %d is greater than the battery capacity (battery capacity: %f)\n", c->node, wisebat_data->initial);
    }

    nodedata->radio_component.name = "radio";
    nodedata->radio_component.consume = NULL;


    set_node_private_data(c, nodedata);

    register_component(c, &nodedata->radio_component, MODE_OFF);


    return 0;
    
 error:
    free(nodedata);
    return -1;
}
double energy_status(call_t *c);
double energy_remaining(call_t * c);
void _update(call_t *c, uint64_t time);
int unsetnode(call_t *c) {

    nodedata_t * nodedata = get_node_private_data(c);
    nodedata->is_dying = 1;
    _update(c, get_time());
    printf("[Energy] ");
    print_human_time(get_time());
    printf(" node %d, remaining %f\n", c->node, nodedata->residual);
    printf("[Energy] unset node %d\n", c->node);


    //free(get_node_private_data(c)); DONT FREE because other layer may ask for battery information

    return 0;
}


/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
    return 0;
}

/* ************************************************** */
/* ************************************************** */


double energy_remaining(call_t *c);


int update_callback(call_t* c, void * args)
{
    nodedata_t * nodedata = get_node_private_data(c);
    nodedata->next_event = NULL;
    _update(c, get_time());
    return 0;
}


void _consume(call_t *c, double energy) {
    nodedata_t * nodedata = get_node_private_data(c);
    nodedata->residual -= energy;
    if (nodedata->residual <= 0) {
        nodedata->residual = 0;
        //node_kill(c->node);
    }
    return;
}
void schedule_update(call_t * c, uint64_t clock)
{
    nodedata_t * nodedata = get_node_private_data(c);
    if(nodedata->is_dying)
    {
        return;
    }
    if(nodedata->next_event && nodedata->next_event->clock > get_time())
    {
        scheduler_delete_callback(c, nodedata->next_event);
        nodedata->next_event = NULL;
    }
    if(clock >= get_time())
    {
        nodedata->next_event = scheduler_add_callback(clock, c, update_callback, NULL);
    }
}

double get_equivalent_capacity(call_t * c, double current)
{
    wisebat_data_t * wisebat_data = get_entity_private_data(c);
    return linear_interpolation(&wisebat_data->capacity_characteristic, current);
}

void update_voltage(call_t * c)
{
    nodedata_t * nodedata = get_node_private_data(c);
    wisebat_data_t * wisebat_data = get_entity_private_data(c);

    // voltage depending on the discharge ratio
    double voltage = linear_interpolation(&wisebat_data->voltage_characteristic, wisebat_data->initial - nodedata->residual);

    // the internal resistance reduce the voltage
    double internal_resitance = linear_interpolation(&wisebat_data->internal_resistance, wisebat_data->initial - nodedata->residual);
    //printf("Voltage %f Resistance %f\n", voltage, internal_resitance);
    voltage -= nodedata->last_current * 0.001 * internal_resitance;
    //                             Ampere     *      Ohms

    nodedata->last_voltage = voltage;
}

component_context_t get_battery_context(call_t * c)
{
    nodedata_t * nodedata = get_node_private_data(c);
    component_context_t context;
    context.voltage = nodedata->last_voltage;
    return context;
}


/**
*   Main funcion: update the battery since the last time we call it. Take into account all componenents (radio included)
*/
void _update(call_t *c, uint64_t time)
{
    if(!is_node_alive(c->node))
    {
        return;
    }
    nodedata_t * nodedata = get_node_private_data(c);
    wisebat_data_t * wisebat_data = get_entity_private_data(c);
    uint64_t elapsed = time - nodedata->last_update;

    /*
     *  Update the battery residual
     */
    if(elapsed > 0)
    {
        double current = nodedata->last_current;

        // if the current is greater than the nominal discharge current, then there are some loss
        if(current > wisebat_data->nominal_discharge_current)
        {
            double capacity = get_equivalent_capacity(c, current);
            current *= wisebat_data->initial / capacity;
        }
        double consumed_since_last_update = elapsed * current / ONE_HOUR_D;
        nodedata->residual -= consumed_since_last_update;
    }
    //----------------------------------------------

    /*
     * Update the voltage
     */
    update_voltage(c);
    //----------------------------------------------


    /*
     * Update the current of each component
     */
    component_context_t context = get_battery_context(c);
    double total_current = 0.0; //<- represent the instantaneous current


    void * components_iterator = das_create_iterator(nodedata->components_private);
    das_component_t* das_component = 0;
    while((das_component = (das_component_t*)das_iterator_next(nodedata->components_private, &components_iterator)))
    {
        component_t * component = das_component->component_ptr;
        if(!component->consume)
        {
            //printf("component %s dont have a consume function\n", component->name);
            continue;
        }

        component_context_t local_context = context;
        // callback BEFORE_COMPONENT_CONSUME
        das_init_traverse(nodedata->before_component_consume_callbacks);
        battery_callback_wrapper_t *wrapper;
        while((wrapper = das_traverse(nodedata->before_component_consume_callbacks)) != NULL)
        {
            wrapper->callback(&(wrapper->call), &local_context, (void*)component);
        }

        double consumption = component->consume(&das_component->call, local_context, das_component->mode);
        if(nodedata->logFile && nodedata->log_level >= 3)
        {
            fprintf(nodedata->logFile,"consumption_%s %"PRId64" %d %.9f \n", component->name, time, c->node, consumption);
        }
        total_current += consumption;
    }

    // callback AFTER_COMPONENTS_UPDATE
    das_init_traverse(nodedata->after_components_updates_callbacks);
    battery_callback_wrapper_t *wrapper;
    while((wrapper = das_traverse(nodedata->after_components_updates_callbacks)) != NULL)
    {
        wrapper->callback(&(wrapper->call), &context, (void*)&total_current);
    }

    //----------------------------------------------


    //if the current has changed, schedule an update
    if(fabs(nodedata->last_current - total_current) > 0.0001)
    {
        schedule_update(c, time + wisebat_data->voltage_latency);
    }
    nodedata->last_current = total_current;

    /*
     * Log
     */
    if(nodedata->logFile)
    {
        nodedata->voltage_min = fmin(nodedata->voltage_min, context.voltage);
        nodedata->voltage_max = fmax(nodedata->voltage_max, context.voltage);
        nodedata->voltage_average = nodedata->voltage_average * (nodedata->last_update - nodedata->last_log);
        nodedata->voltage_average += context.voltage * elapsed;
        if(time - nodedata->last_log > 0)
        {
            nodedata->voltage_average /= time - nodedata->last_log;
        }
        else
        {
            nodedata->voltage_average = context.voltage;
        }
    }
    if(nodedata->logFile && (nodedata->log_level >= 2 || (nodedata->log_level == 1 && (time - nodedata->last_log) >= nodedata->log_interval)))
    {
        nodedata->last_log = time;
        fprintf(nodedata->logFile,"residual %"PRId64" %d %.9f \n", time, c->node, nodedata->residual);
        fprintf(nodedata->logFile,"voltage %"PRId64" %d %.9f \n", time, c->node, nodedata->voltage_average);
        fprintf(nodedata->logFile,"voltage_min %"PRId64" %d %.9f \n", time, c->node, nodedata->voltage_min);
        fprintf(nodedata->logFile,"voltage_max %"PRId64" %d %.9f \n", time, c->node, nodedata->voltage_max);
        //fflush(nodedata->logFile);
        nodedata->voltage_min = INFINITY;
        nodedata->voltage_max = 0;
    }
    if((nodedata->residual <= 0 || nodedata->last_voltage < wisebat_data->cut_off_voltage))
    {
        if(nodedata->logFile)
        {
            fprintf(nodedata->logFile,"residual %"PRId64" %d %.9f \n", time, c->node, nodedata->residual);
            fflush(nodedata->logFile);
        }
        printf("[Energy] %"PRId64" kill_node %d voltage=%f\n", time, c->node, nodedata->last_voltage);
        if(!nodedata->is_dying)
		node_kill(c->node);
        return;
    }
    //----------------------------------------------

    nodedata->last_update = time;
}



void consume(call_t *c, double energy) {
    if(!is_node_alive(c->node))
    {
        return;
    }
    //fprintf(stderr, "Depreciated function: consume()\n");
    _consume(c, energy);
    _update(c, get_time());
    return;
}



int das_component_equals(void*das_component, void*component)
{
    return !strcasecmp( ((das_component_t*)das_component)->component_ptr->name, ((component_t*)component)->name);
}

void set_radio_mode_without_update(call_t *c, component_mode_t mode)
{
    nodedata_t * nodedata = get_node_private_data(c);
    das_component_t * das_component = 0;
    if(!das_selective_get(nodedata->components_private, das_component_equals, &nodedata->radio_component, (void**)&das_component))
    {
        printf("Error: The radio has not been registered\n");
        return;
    }
    das_component->mode = mode;
}

void consume_tx(call_t *c, uint64_t duration, double txdBm) {
    if(!is_node_alive(c->node))
    {
        return;
    }
    //fprintf(stderr, "Depreciated function: consume_tx()\n");
    return;
/*
    printf("consume_tx\n");
    nodedata_t * nodedata = get_node_private_data(c);
    component_mode_t previous_mode = get_component_mode(c, &nodedata->radio_component);

    set_radio_mode_without_update(c, MODE_TX);
    _update(c, get_time());
    if(!is_node_alive(c->node))
    {
        return;
    }
    set_radio_mode_without_update(c, previous_mode);
    _update(c, get_time() - duration);
    return;*/
}

void consume_rx(call_t *c, uint64_t duration) {

    nodedata_t * nodedata = get_node_private_data(c);
    if(nodedata->debug)
    {
        fprintf(stderr, "Depreciated function: consume_rx()\n");
    }
    return;
    /*
    nodedata_t * nodedata = get_node_private_data(c);

    component_mode_t previous_mode = get_component_mode(c, &nodedata->radio_component);
    set_radio_mode_without_update(c, MODE_RX);

    _update(c, get_time());
    if(!is_node_alive(c->node))
    {
        return;
    }
    set_radio_mode_without_update(c, previous_mode);
    _update(c, get_time() - duration);
    return;*/
}

void consume_idle(call_t *c, uint64_t duration) {
    fprintf(stderr, "do'nt use the function consume_idle\n");
    return;
}


double energy_consumed(call_t *c) {
    _update(c, get_time());
    nodedata_t * nodedata = get_node_private_data(c);
    wisebat_data_t * wisebat_data = get_entity_private_data(c);
    return wisebat_data->initial - nodedata->residual;
}

double energy_remaining(call_t *c) {
    _update(c, get_time());
    nodedata_t * nodedata = get_node_private_data(c);
    return nodedata->residual;
}

double energy_status(call_t *c) {
    _update(c, get_time());
    nodedata_t * nodedata = get_node_private_data(c);
    wisebat_data_t * wisebat_data = get_entity_private_data(c);
    double status = nodedata->residual / wisebat_data->initial;
    if ((status >= 0) && (status <= 1)) {
        return status;
    } else {
        return 0;
    }
}





/* Battery Custom Method */
component_mode_t get_component_mode(call_t *c, component_t *component)
{
    nodedata_t * nodedata = get_node_private_data(c);
    das_component_t * das_component = 0;
    if(!das_selective_get(nodedata->components_private, das_component_equals, component, (void**)&das_component))
    {
        printf("%lu Error: The sensor %s has not been registered (get_component_mode)\n", get_time(), component->name);
        return MODE_UNKNOWN;
    }
    return das_component->mode;
}

/* Battery Custom Method */
void set_component_mode(call_t *c, component_t *component, component_mode_t mode)
{
    nodedata_t * nodedata = get_node_private_data(c);
    das_component_t * das_component = 0;
    if(!das_selective_get(nodedata->components_private, das_component_equals, component, (void**)&das_component))
    {
        printf("%lu Error: The sensor %s has not been registered (set_component_mode)\n", get_time(), component->name);
        return;
    }
    if(das_component->mode != mode)
    {
        das_component->mode = mode;
        _update(c, get_time());
        if(!is_node_alive(c->node))
        {
            return;
        }
    }
}

/* Battery Custom Method */
void register_component(call_t *c, component_t *component, component_mode_t mode)
{

    nodedata_t * nodedata = get_node_private_data(c);

    das_insert(nodedata->components, component);

    das_component_t* das_component = malloc(sizeof(das_component_t));
    das_component->component_ptr = component;
    das_component->mode = mode;
    das_component->call.entity = c->from;
    das_component->call.node = c->node;
    das_component->call.from = c->entity;
    das_insert(nodedata->components_private, das_component);

    if(nodedata->logFile)
    {
        fprintf(nodedata->logFile,"consumption_%s %"PRId64" %d %d \n", component->name, get_time(), c->node, 0);
    }
}

/* Battery Custom Method */
void set_radio_comsumption(call_t *c, component_consume_t consume)
{
    nodedata_t * nodedata = get_node_private_data(c);

    // change the call_t
    das_component_t * das_component = 0;
    if(!das_selective_get(nodedata->components_private, das_component_equals, &nodedata->radio_component, (void**)&das_component))
    {
        das_component->call.entity = c->from;
        das_component->call.from   = c->entity;
        das_component->call.node   = c->node;
    }
    nodedata->radio_component.consume = consume;
}
/* Battery Custom Method */
void set_radio_mode(call_t *c, component_mode_t mode)
{
    nodedata_t * nodedata = get_node_private_data(c);
    set_component_mode(c, &nodedata->radio_component, mode);
}
/* Battery Custom Method */
component_mode_t get_radio_mode(call_t *c)
{
    nodedata_t * nodedata = get_node_private_data(c);
    return get_component_mode(c, &nodedata->radio_component);
}



/* ************************************************** */
/* ************************************************** */
energy_methods_t methods = {consume_tx, 
                            consume_rx,
                            consume_idle,
                            consume,
                            energy_consumed,
                            energy_remaining,
                            energy_status};

/**
 * @brief ioctl used to communicate with the components handle by the battery. don't use those functione directly, see the header file for simpler function
 * @param c
 * @param option
 * @param in
 * @param out
 * @return
 */

int ioctl(call_t *c, int option, void *in, void **out)
{
    nodedata_t * nodedata = get_node_private_data(c);
    switch(option)
    {
    case REGISTER_COMPONENT:
    {
        register_component(c, (component_t*) in, *(component_mode_t*)(*out));
        return 0;
    } break;
    case SET_COMPONENT_MODE:
    {
        set_component_mode(c, (component_t*) in, *(component_mode_t*)(*out));
        return 0;
    } break;
    case GET_COMPONENT_MODE:
    {
         *(component_mode_t*)(*out) = get_component_mode(c, (component_t*) in);
        return 0;
    } break;
    case SET_RADIO_CONSUMPTION:
    {
        set_radio_comsumption(c, *(component_consume_t*) in);
        return 0;
    } break;
    case SET_RADIO_MODE:
    {
        set_radio_mode(c, *(component_mode_t*) in);
        return 0;
    } break;
    case GET_RADIO_MODE:
    {
        *((component_mode_t*) in) = get_radio_mode(c);
        return 0;
    } break;
    case COMPONENTS_INIT_TRAVERSE:
    {
        das_init_traverse(nodedata->components);
        return 0;
    } break;
    case COMPONENTS_TRAVERSE:
    {
        *((component_t**)in) = das_traverse(nodedata->components);   
        //if(c->node == 0) printf("c: %p\n",*((component_t**)in));
        return 0;
    } break;
    case GET_BATTERY_CONTEXT:
    {
        *((component_context_t*)in) = get_battery_context(c);
        return 0;
    } break;
    case REGISTER_CALLBACK:
    {
        battery_callback_type_t type = *(battery_callback_type_t*)in;
        switch(type)
        {
        case BATTERY_AFTER_COMPONENTS_UPDATE:
            das_insert(nodedata->after_components_updates_callbacks, *out);
            break;
        case BATTERY_BEFORE_COMPONENT_CONSUME:
            das_insert(nodedata->before_component_consume_callbacks, *out);
            break;
        }

        return 0;
    } break;
    }
    return -1;
}

