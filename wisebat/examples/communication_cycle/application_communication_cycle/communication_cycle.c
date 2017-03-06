/**
 *  \file   test.c
 *  \brief  TEST application
 *  \author Quentin BRAMAS
 *  \date   2013
 **/
#include <stdio.h>

#include <include/modelutils.h>
#include "../../../energy_wisebat/wisebat.h"
#include "../../../energy_wisebat/scenario_tool.h"

#define ONE_HOUR 3600000000000
/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "Communication Cycle",
    "Quentin BRAMAS",
    "0.1",
    MODELTYPE_APPLICATION,
    {NULL, 0}
};



/* ************************************************** */
/* ************************************************** */
struct nodedata {
    component_t sensor_component;
    component_t cpu_component;
    int overhead;
};


double sensor_consume(call_t* c, component_context_t context, component_mode_t mode)
{
    if(mode == MODE_INIT)
    {
        return 20;
    }
    if(mode == MODE_READ)
    {
        return 8;
    }
    return 0.001;
}
double cpu_consume(call_t* c, component_context_t context, component_mode_t mode)
{
    if(mode == MODE_SLEEP)
    {
        return 0.1;
    }
    return 10;
}

double radio_consume(call_t* c, component_context_t context, component_mode_t mode)
{
    if(mode == MODE_SLEEP)
    {
        return 0.1;
    }
    if(mode == MODE_IDLE)
    {
        return 2;
    }
    if(mode == MODE_RX)
    {
        return 9;
    }
    return 11;
}


// Change the global consumption
int power_manager_after_update(call_t* c, component_context_t* context, void* arg)
{
    double* current = (double*)arg;
    if(*current < 0.02)
    {
        *current/=0.96;
    }
    else
    {
        *current/=0.99;
    }
    return 0;
}

// normalize the voltage
int power_manager_before_consume(call_t* c, component_context_t* context, void* arg)
{
    //component_t* component = (component_t*)arg;
    context->voltage = 3;
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {
    return 0;
}


int destroy(call_t *c) {
    return 0;
}



/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
    struct nodedata *nodedata = malloc(sizeof(struct nodedata));
 

    set_node_private_data(c, nodedata);
    return 0;
}

int unsetnode(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);

    free(nodedata);
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int cycle(call_t *c, void *args);

int bootstrap(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);

    array_t *down = get_entity_bindings_down(c);
    call_t c0 = {down->elts[0], c->node, c->entity};
    /* get overhead */
    nodedata->overhead = GET_HEADER_SIZE(&c0);

    nodedata->cpu_component.name = "cpu";
    nodedata->cpu_component.consume = cpu_consume;
    battery_register_component(c, &nodedata->cpu_component, MODE_SLEEP);

    nodedata->sensor_component.name = "sensor";
    nodedata->sensor_component.consume = sensor_consume;
    battery_register_component(c, &nodedata->sensor_component, MODE_OFF);

    battery_set_radio_consumption(c, radio_consume);
    battery_set_radio_mode(c, MODE_SLEEP);

    battery_register_callback(c, BATTERY_AFTER_COMPONENTS_UPDATE, power_manager_after_update);
    battery_register_callback(c, BATTERY_BEFORE_COMPONENT_CONSUME, power_manager_before_consume);


    /* eventually schedule callback */
    if(c->node == 0)
    {
        scenario_launch(cycle,c);
    }

    return 0;
}

int ioctl(call_t *c, int option, void *in, void **out) {
    return 0;
}

void send_a_packet(call_t *c)
{
    struct nodedata *nodedata = get_node_private_data(c);
    array_t *down = get_entity_bindings_down(c);
    packet_t *packet = packet_create(c, nodedata->overhead, -1);

    call_t c0 = {down->elts[0], c->node, c->entity};
    destination_t destination = {1, {0,0,0}};
        
    if (SET_HEADER(&c0, packet, &destination) == -1) {
        packet_dealloc(packet);
        return;
    }
    printf("[APP] %lu send packet to 1\n", get_time());
    TX(&c0, packet);
}


/* ************************************************** */
/* ************************************************** */
scenario_declare(cycle)
{
    struct nodedata *nodedata = get_node_private_data(c);


    scenario_start();

    battery_set_component_mode(c, &nodedata->cpu_component, MODE_RUN);
    scenario_wait_ms(5);

    battery_set_component_mode(c, &nodedata->sensor_component, MODE_INIT);
    scenario_wait_ms(10);

    battery_set_component_mode(c, &nodedata->sensor_component, MODE_READ);
    scenario_wait_ms(30);

    battery_set_component_mode(c, &nodedata->sensor_component, MODE_OFF);
    battery_set_component_mode(c, &nodedata->cpu_component, MODE_SLEEP);
    send_a_packet(c);
    scenario_wait_s(30);

    scenario_restart();
    return 0;
}

/* ************************************************** */
/* ************************************************** */

/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {
    printf("[APP] %lu received packet from 0\n", get_time());
}


/* ************************************************** */
/* ************************************************** */
application_methods_t methods = {rx};
