#ifndef REALISTIC_BATTERY_H
#define REALISTIC_BATTERY_H
#include <include/modelutils.h>
#include "wisebat_tools.h"



typedef enum _component_mode{
    MODE_UNKNOWN,
    MODE_OFF,
    MODE_ON,
    MODE_SLEEP,
    MODE_RUN,
    MODE_TX,
    MODE_RX,
    MODE_IDLE,
    MODE_INIT,
    MODE_READ
} component_mode_t;

typedef enum _battery_ioctl_command {
    REGISTER_COMPONENT,
    SET_COMPONENT_MODE,
    GET_COMPONENT_MODE,
    SET_RADIO_CONSUMPTION ,
    SET_RADIO_MODE,
    GET_RADIO_MODE,
    GET_BATTERY_CONTEXT ,
    COMPONENTS_INIT_TRAVERSE,
    COMPONENTS_TRAVERSE,
    REGISTER_CALLBACK
} battery_ioctl_command_t;

typedef enum _battery_callback_type {
    BATTERY_AFTER_COMPONENTS_UPDATE,
    BATTERY_BEFORE_COMPONENT_CONSUME
} battery_callback_type_t;

#define IOCTL_OK 0

typedef struct _component_context{

    /**
     * @brief voltage, the voltage of the battery
     */
    double voltage;
} component_context_t;


typedef struct _battery_callback_wrapper{
    call_t call;
    int (*callback)(call_t*, component_context_t*, void*);
} battery_callback_wrapper_t;


typedef double (*component_consume_t)(call_t* c, component_context_t context, component_mode_t mode);


typedef struct _component {
    char * name;
    component_consume_t consume;
} component_t;

typedef struct _wisebat_voltage_characteristic
{
    double current;
    double (*callback)(double);
} wisebat_voltage_characteristic_t;

typedef struct _wisebat_capacity_characteristic
{
    double current;
    double capacity;
} wisebat_capacity_characteristic_t;


double cpu_consumption_default(call_t* c, component_context_t context, component_mode_t mode)
{
    if(mode == MODE_OFF)
    {
        return 0;
    }
    return 10;
}

int battery_get_component_mode (call_t *c, component_t* component, component_mode_t* mode);
int battery_components_traverse (call_t *c, component_t** component);
int battery_components_init_traverse (call_t *c);
double cpu_consumption_auto_off(call_t* c, component_context_t context, component_mode_t mode)
{
    int nb_components_on = 0;
    battery_components_init_traverse(c);
    component_t * component;
    while(IOCTL_OK == battery_components_traverse(c, &component) && NULL != component)
    {
        component_mode_t mode;
        if(IOCTL_OK != battery_get_component_mode(c, component, &mode))
        {
            break;
        }
        if(mode != MODE_OFF && mode != MODE_SLEEP)
        {
            ++nb_components_on;
        }
    }
    if(mode == MODE_OFF)
    {
        return 0;
    }
    if(nb_components_on == 1) //there is only the cpu running
    {
        return 0.01;
    }
    return 10;
}
double radio_consumption_default(call_t* c, component_context_t context, component_mode_t mode)
{
    switch(mode)
    {
    case MODE_OFF:
    case MODE_SLEEP:
        return 0;
    case MODE_RX:
        return 6;
    case MODE_TX:
        return 11;
    default:
        return 2;
    }
    
}

double default_voltage_characteristic_1(double x)
{
    return -3.01*x*x*x + 5.13*x*x - 3.03*x + 3.13;
}
double default_voltage_characteristic_2(double x)
{
    return -3.92*x*x*x + 5.5*x*x - 2.9*x +3;
}
double default_voltage_characteristic_3(double x)
{
    return -10.74*x*x*x + 11.18*x*x - 4.13*x +2.8;
}


#define RETURN_ENERGY_IOCTL(option, in, out) \
    do {\
        call_t    c0;\
        c0.node   = c->node;\
        c0.from   = c->entity;\
        c0.entity = get_energy_entity(c);\
        if(c0.entity == -1) return -1; \
        return IOCTL(&c0, option, in, out);\
    } while(0)


/**
 *  Register a component with a initial mode
 */
int battery_register_component (call_t *c, component_t *component, component_mode_t mode)
{
    void *pmode = &mode;
    RETURN_ENERGY_IOCTL(REGISTER_COMPONENT, component, &pmode);
}

/**
 *  Set the mode of a component
 */
int battery_set_component_mode (call_t *c, component_t *component, component_mode_t mode)
{
    void *pmode = &mode;
    RETURN_ENERGY_IOCTL(SET_COMPONENT_MODE, component, &pmode);
}

/**
 *  set the variable pointed by 'out_mode' with the mode of the component, return 0 if the function succed, -1 otherwise
 */
int battery_get_component_mode (call_t *c, component_t *component, component_mode_t* out_mode)
{
    RETURN_ENERGY_IOCTL(GET_COMPONENT_MODE, component, (void**)&out_mode);
}

/**
 *  Set the consume function for the radio component
 */
int battery_set_radio_consumption(call_t *c, component_consume_t consume)
{
    RETURN_ENERGY_IOCTL(SET_RADIO_CONSUMPTION, &consume, NULL);
}

/**
 *  Set the radio mode
 */
int battery_set_radio_mode (call_t *c, component_mode_t mode)
{
    RETURN_ENERGY_IOCTL(SET_RADIO_MODE, &mode, NULL);
}

/**
 *  call battery_get_component_mode with the radio component
 */
int battery_get_radio_mode (call_t *c, component_mode_t* mode)
{
    RETURN_ENERGY_IOCTL(GET_RADIO_MODE, mode, NULL);
}

/**
 *  init the traverse of the das of components
 */
int battery_components_init_traverse (call_t *c)
{
    RETURN_ENERGY_IOCTL(COMPONENTS_INIT_TRAVERSE, NULL, NULL);
}

/**
 *  traverse the das of components
 */
int battery_components_traverse (call_t *c, component_t** component)
{
    RETURN_ENERGY_IOCTL(COMPONENTS_TRAVERSE, component, NULL);
}

/**
 *  get the context (voltage...)
 */
int battery_get_context (call_t *c, component_context_t* context)
{
    RETURN_ENERGY_IOCTL(GET_BATTERY_CONTEXT, context, NULL);
}

/**
 *  register a callback
 */
int battery_register_callback(call_t *c, battery_callback_type_t callback_type, int (*callback)(call_t*, component_context_t*, void*))
{
    battery_callback_wrapper_t *wrapper = malloc(sizeof(battery_callback_wrapper_t));
    wrapper->callback = callback;
    wrapper->call.entity = c->entity;
    wrapper->call.node = c->node;
    wrapper->call.from = get_energy_entity(c);
    RETURN_ENERGY_IOCTL(REGISTER_CALLBACK, &callback_type, (void**)&wrapper);
}


#endif // REALISTIC_BATTERY_H

