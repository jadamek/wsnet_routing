/**
 *  \file   test.c
 *  \brief  TEST application
 *  \author Quentin BRAMAS
 *  \date   2013
 **/
#include <stdio.h>

#include <include/modelutils.h>
#include "../energy_wisebat/wisebat.h"

#define ONE_HOUR 3600000000000
/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "TEST Battery",
    "Quentin BRAMAS",
    "0.1",
    MODELTYPE_APPLICATION,
    {NULL, 0}
};



/* ************************************************** */
/* ************************************************** */
struct nodedata {
    int debug;
    uint64_t start;
    uint64_t on_period;
    uint64_t off_period;
    component_t consumer_component;
    double current;
};


double consumer_consume(call_t* c, component_context_t context, component_mode_t mode)
{
    struct nodedata *nodedata = get_node_private_data(c);
    if(mode == MODE_RUN)
    {
        return nodedata->current;
    }
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
    param_t *param;

    nodedata->start = 15000000;
    nodedata->on_period = 15000000;
    nodedata->off_period = 200000000;
    nodedata->current = 1.0;

/* get parameters */
    das_init_traverse(params);
    while ((param = (param_t *) das_traverse(params)) != NULL) {
        if (!strcmp(param->key, "debug")) {
            if (get_param_integer(param->value, &(nodedata->debug))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "start")) {
            if (get_param_time(param->value, &(nodedata->start))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "on-period")) {
            if (get_param_time(param->value, &(nodedata->on_period))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "off-period")) {
            if (get_param_time(param->value, &(nodedata->off_period))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "current")) {
            if (get_param_double(param->value, &(nodedata->current))) {
                goto error;
            }
        }
    }

    set_node_private_data(c, nodedata);
    return 0;

 error:
    free(nodedata);
    return -1;
}

int unsetnode(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);

    free(nodedata);

    return 0;
}


int on(call_t *c, void *args);
int off(call_t *c, void *args);
int print(call_t *c, void *args)
{
    if(!is_node_alive(c->node))
    {
        return -1;
    }
    component_context_t context;
    if(IOCTL_OK != battery_get_context(c, &context))
    {
        return -1;
    }
    printf("%d remaining %f voltage %f\n", c->node, battery_remaining(c), context.voltage);
    scheduler_add_callback(get_time()+ONE_HOUR*24, c, print, NULL);
    return 0;
}

/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);

    uint64_t start = nodedata->start;

    nodedata->consumer_component.name = "consumer";
    nodedata->consumer_component.consume = consumer_consume;

    battery_register_component(c, &nodedata->consumer_component, MODE_OFF);



    /* eventually schedule callback */
    scheduler_add_callback(start, c, on, NULL);

    scheduler_add_callback(get_time()+ONE_HOUR, c, print, NULL);

    return 0;
}

int ioctl(call_t *c, int option, void *in, void **out) {
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int on(call_t *c, void *args) {
    struct nodedata *nodedata = get_node_private_data(c);
    scheduler_add_callback(get_time() + nodedata->on_period, c, off, NULL);
    battery_set_component_mode(c, &nodedata->consumer_component, MODE_RUN);
    return 0;
}
int off(call_t *c, void *args) {
    struct nodedata *nodedata = get_node_private_data(c);
    scheduler_add_callback(get_time() + nodedata->off_period, c, on, NULL);
    battery_set_component_mode(c, &nodedata->consumer_component, MODE_OFF);
    return 0;
}


/* ************************************************** */
/* ************************************************** */

/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {

}


/* ************************************************** */
/* ************************************************** */
application_methods_t methods = {rx};
