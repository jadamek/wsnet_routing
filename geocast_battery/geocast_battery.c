// geocast_battery.c
// adaption of geocasting model which tests battery consumption over repeated transmission
// Jordan Adamek
// Spring 2017

#include <stdio.h>
#include <include/modelutils.h>

#include "../wisebat/energy_wisebat/wisebat.h"
#include "../wisebat/energy_wisebat/scenario_tool.h"

////////////////////////////////////////////////////////////////////////////////
// Model Information

model_t model =
{
    "Single Geocast Model + Battery Consumption Scenario",
    "Jordan Adamek & James Robinson",
    "0.1",
    MODELTYPE_APPLICATION,
    {NULL, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Defines

#define DEFAULT_PACKET_SIZE 10
#define DEFAULT_START_TIME 2000000000ull

#define ERROR -1
#define EMPTY_DESTINATION {-2, {-1, -1, -1}}

// Data message interval of 1 second
static const int DMI = 1000000000U;

// Total messages sent throughought scenario
static const int BOUND = 5;

static int ROUND = -1;

#define CALL_DOWN(call) {get_entity_bindings_down(call)->elts[0], call->node,\
	call->entity}

#define TRUE 1
#define FALSE 0

#define TIME_PRECISION 1000000000
////////////////////////////////////////////////////////////////////////////////
// Structs and Typedefs

typedef int bool;

struct node_data
{
    int overhead;
    int packet_size;
    bool received_packet;
    component_t cpu_component;
};

struct entity_data
{
    destination_t source;
    destination_t destination;
    int num_targets;
    int data_tx;
    int data_rx;
    int* round_tx;
    int* round_rx;
    uint64_t latency;
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes

//init
int init(call_t *call, void *params);

//destroy
int destroy(call_t* call);

int call_back(call_t *call, void *args);
int tx(call_t *call, void *args);

int compare_destinations(destination_t *dest1, destination_t *dest2);
double cpu_consume(call_t* c, component_context_t context, component_mode_t mode);
double radio_consume(call_t* c, component_context_t context, component_mode_t mode);
int power_manager_after_update(call_t* c, component_context_t* context, void* arg);
int power_manager_before_consume(call_t* c, component_context_t* context, void* arg);

////////////////////////////////////////////////////////////////////////////////
// Init

int init(call_t *call, void *params)
{
    struct entity_data *entity_data = malloc(sizeof(struct entity_data));
    param_t *param;
    destination_t empty_destination = EMPTY_DESTINATION;
    double size = -1;
    int i;

    //init values
    entity_data->num_targets = 0;
    entity_data->data_tx = 0;
    entity_data->data_rx = 0;
    entity_data->round_tx = (int*)malloc(BOUND * sizeof(int));
    entity_data->round_rx = (int*)malloc(BOUND * sizeof(int));
    for(i = 0; i < BOUND; i++){
	 entity_data->round_tx[i] = 0;
	 entity_data->round_rx[i] = 0;
    }
    entity_data->latency = 0;
    entity_data->source = empty_destination;
    entity_data->destination = empty_destination;

    //get parameters
    das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
	if(!strcmp(param->key, "source"))
	    get_param_nodeid(param->value, &(entity_data->source.id),
		entity_data->source.id);
	if(!strcmp(param->key, "x_pos"))
	    get_param_double(param->value,
		&(entity_data->destination.position.x));
	if(!strcmp(param->key, "y_pos"))
	    get_param_double(param->value,
		&(entity_data->destination.position.y));
	if(!strcmp(param->key, "z_pos"))
	    get_param_double(param->value,
		&(entity_data->destination.position.z));
	if(!strcmp(param->key, "length"))
	    get_param_double(param->value, &size);
	if(!strcmp(param->key, "radius"))
	    get_param_double(param->value, &size);
    }

    if(compare_destinations(&entity_data->source, &empty_destination))
	get_param_nodeid("random", &(entity_data->source.id),
	    entity_data->source.id);
    if(compare_destinations(&entity_data->destination, &empty_destination))
    {
	double tmp1 = 0, tmp2 = 0;

	while(((tmp1 = get_random_x_position()) + (size / 2)) >
	    get_topology_area()->x || (tmp1 - (size / 2) < 0));
	while(((tmp2 = get_random_y_position()) + (size / 2)) >
	    get_topology_area()->y || (tmp2 -(size / 2) < 0));
	entity_data->destination.position.x = tmp1;
	entity_data->destination.position.y = tmp2;
    }
    entity_data->destination.position.z = 0;

    //save private data
    set_entity_private_data(call, entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    struct entity_data *entity_data = get_entity_private_data(call);
    FILE *results;
    char *name = (char*)malloc(11 * sizeof(int));
    sprintf(name, "results.txt");
    int i;

    fprintf(stderr, "Application Statistics:\n");
    fprintf(stderr, "  message sent from %d\n", entity_data->source.id);
    fprintf(stderr, "  message sent to %f, %f, %f\n",
	entity_data->destination.position.x,
	entity_data->destination.position.y,
	entity_data->destination.position.z);
    fprintf(stderr, "  Number of targets: %d\n", entity_data->num_targets);
    fprintf(stderr, "  num messages sent: %d\n", entity_data->data_tx);
    fprintf(stderr, "  num messages received: %d\n", entity_data->data_rx);
    fprintf(stderr, "  message delivery rate: %f",
	(double)entity_data->data_rx / (double)entity_data->data_tx);
    if(entity_data->data_rx < entity_data->num_targets || entity_data->data_rx < 1)
        fprintf(stderr, " <failed>");
    fprintf(stderr, "\n  latency: %lld nanoseconds", entity_data->latency);

    for(i = 0; i < BOUND; i++)
        fprintf(stderr, "\n  Delivery for round %d: %d / %d (%0.2f\%)", i + 1, entity_data->round_rx[i], entity_data->round_tx[i] * entity_data->num_targets, (double)entity_data->round_rx[i] / ((double)(entity_data->round_tx[i] * entity_data->num_targets)) * 100.0);

    fprintf(stderr, "\n");

    if((results = fopen(name, "a")) == NULL)
	fprintf(stderr, "[ERR] Couldn't open file: results.txt\n");
    else{
	fprintf(results, "%d\t%d\t%lld\t", entity_data->num_targets,
	    (int)entity_data->data_rx, entity_data->latency);
    }

    fclose(results);
    free(entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

int setnode(call_t *call, void *params)
{
    struct node_data *node_data = malloc(sizeof(struct node_data));
    /*param_t *param;*/

    //set default values
    node_data->packet_size = DEFAULT_PACKET_SIZE;
    node_data->received_packet = FALSE;

    /*
    //get params
    das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
	if(!strcmp(param->key, "packet_size"))
	    get_param_integer(param->value, &(node_data->packet_size));
    }
    */

    set_node_private_data(call, node_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Unset node

int unsetnode(call_t *call)
{
    struct node_data *node_data = get_node_private_data(call);

    free(node_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap

int bootstrap(call_t *call)
{
    struct entity_data *entity_data = get_entity_private_data(call);
    struct node_data *node_data = get_node_private_data(call);
    int i = get_entity_links_down_nbr(call);
    entityid_t *down = get_entity_links_down(call);
    node_data->overhead = 0;

if(call->node == 0)
fprintf(stderr, "[DBG] source: %d, destination: %f, %f, %f\n", entity_data->source.id, entity_data->destination.position.x, entity_data->destination.position.y, entity_data->destination.position.z);

    //get number of targets
    if(call->node == 0)
    {
	int tmp = get_node_count(), j;
	for(j = 0; j < tmp; ++j)
	{
	    if(j == entity_data->source.id)
		continue;
	    destination_t tmp_dest = {j, *get_node_position(j)};

	    if(tmp_dest.position.x >= entity_data->destination.position.x - 1 &&
	       tmp_dest.position.x <= entity_data->destination.position.x + 1 &&
	       tmp_dest.position.y >= entity_data->destination.position.y - 1 &&
	       tmp_dest.position.y <= entity_data->destination.position.y + 1)
	    {
		entity_data->num_targets++;
	    }
	}
    }

    //get overhead
    while(i--)
    {
	call_t call_down = {down[i], call->node, call->entity};

	if(get_entity_type(&call_down) == MODELTYPE_ROUTING ||
	    get_entity_type(&call_down) == MODELTYPE_MAC)
	    node_data->overhead += GET_HEADER_SIZE(&call_down);
    }
    
    if(call->node == entity_data->source.id){
        for(i = 0; i < BOUND; i++){
	    scheduler_add_callback(DEFAULT_START_TIME + i * DMI, call, call_back, NULL);
	}
    }
    return 0;
}

int call_back(call_t *call, void *args)
{
    struct node_data *node_data = get_node_private_data(call);
    struct entity_data *entity_data = get_entity_private_data(call);
    call_t call_down = CALL_DOWN(call);

    entity_data->source.position = *get_node_position(entity_data->source.id);
    packet_t *packet = packet_create(call, node_data->packet_size +
	node_data->overhead, -1);
    if(SET_HEADER(&call_down, packet, &(entity_data->destination)) == ERROR)
    {
	fprintf(stderr, "Couldn't route, error in down module from unicast\n");
	packet_dealloc(packet);
	return 0;
    }

    ROUND++;

    entity_data->data_tx++;
    entity_data->round_tx[ROUND]++;
    TX(&call_down, packet);

    return 0;
}

int ioctl(call_t *call, int option, void *in, void **out)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// RX

void rx(call_t *call, packet_t *packet)
{
    struct entity_data *entity_data = get_entity_private_data(call);
    struct node_data *node_data = get_node_private_data(call); 

    if(!node_data->received_packet)
    {
	entity_data->latency = get_time() - DEFAULT_START_TIME;
	node_data->received_packet = TRUE;
    }
    entity_data->data_rx++;
    entity_data->round_rx[ROUND]++;
    packet_dealloc(packet);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Helper Functions

bool compare_destinations(destination_t *dest1, destination_t *dest2)
{
    if(dest1->id != dest2->id)
	return FALSE;
    if(dest1->position.x != dest2->position.x)
	return FALSE;
    if(dest1->position.y != dest2->position.y)
	return FALSE;
    if(dest1->position.z != dest2->position.z)
	return FALSE;
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
// Power Consumption Functions
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

////////////////////////////////////////////////////////////////////////////////
// Application Methods

application_methods_t methods =
{
    rx
};
