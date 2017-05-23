// multicast.c
// generic application that selects one sender and tells the routing module
// to select the correct number of targets for multicasting
// James Robinson
// 4/30/2017

#include <stdio.h>
#include <stdbool.h>
#include <include/modelutils.h>

////////////////////////////////////////////////////////////////////////////////
// Model information

model_t model =
{
	"Multicast Application Module",
	"James Robinson",
	"0.1",
	MODELTYPE_APPLICATION,
	{NULL, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Defines

#define TRUE 1
#define FALSE 2

#define DEFAULT_PACKET_SIZE 10
#define TX_START_TIME 2000000000
#define DEFAULT_START_TIME 0

#define ERROR -1
#define EMPTY_DESTINATION {-2, {-1, -1, -1}}

#define CALL_DOWN(call) {get_entity_bindings_down(call)->elts[0], call->node,\
	call->entity}
#define NEW(type) malloc(sizeof(type))

#define TIME_PRECISION 1000000000

////////////////////////////////////////////////////////////////////////////////
// Structs and Typedefs

typedef struct
{
	int overhead;
	int packet_size;
	bool received_packet;
}node_data_t;

typedef struct
{
	destination_t source;
	int num_targets;
	int num_tx;
	int num_rx;
	uint64_t latency;
}entity_data_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes

int init(call_t *call, void *params);

int destroy(call_t *call);

int call_back(call_t *call, void *args);
int tx_start(call_t *call, void *args);

bool compare_destinations(destination_t *l, destination_t *r);

////////////////////////////////////////////////////////////////////////////////
// Init

int init(call_t *call, void *params)
{
    entity_data_t *entity_data = NEW(entity_data_t);
    param_t *param;
    destination_t empty_destination = EMPTY_DESTINATION;

    //init values
    int percentage = 0;
    entity_data->source = empty_destination;
    entity_data->num_targets = 0;
    entity_data->num_tx = 0;
    entity_data->num_rx = 0;
    entity_data->latency = 0;

    //get parameters
    das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
	if(!strcmp(param->key, "source"))
	    get_param_nodeid(param->value, &(entity_data->source.id),
		entity_data->source.id);
	if(!strcmp(param->key, "num_targets"))
	    get_param_integer(param->value, &(entity_data->num_targets));
	if(!strcmp(param->key, "percent"))
	    get_param_integer(param->value, &percentage);
    }

    if(compare_destinations(&entity_data->source, &empty_destination))
	get_param_nodeid("random", &(entity_data->source.id),
	    entity_data->source.id);

    if(percentage != 0)
	entity_data->num_targets = ceil(get_node_count() *
	    ((double)percentage / 100));

    set_entity_private_data(call, entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    entity_data_t *entity_data = get_entity_private_data(call);
    FILE *results;
    char *name = (char*)malloc(11 * sizeof(int));
    sprintf(name, "results.txt");

    fprintf(stderr, "Application Statistics:\n");
    fprintf(stderr, "  message sent from %d\n", entity_data->source.id);
    fprintf(stderr, "  number of targets: %d\n", entity_data->num_targets);
    fprintf(stderr, "  num messages sent: %d\n", entity_data->num_tx);
    fprintf(stderr, "  num messages received: %d\n", entity_data->num_rx);
    fprintf(stderr, "  message delivery rate: %f\n",
	(double)entity_data->num_rx / (double)entity_data->num_tx);
    fprintf(stderr, "  latency: %lld nanoseconds\n", entity_data->latency);

    if((results = fopen(name, "a")) == NULL)
	fprintf(stderr, "[ERR] Couldn't open file: results.txt\n");
    else
	fprintf(results, "%d\t%d\t%lld\t", entity_data->num_targets,
	    entity_data->num_rx, entity_data->latency);
    fclose(results);

    free(entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

int setnode(call_t *call, void *params)
{
    node_data_t *node_data = NEW(node_data_t);
    /*param_t *param*/

    node_data->packet_size = DEFAULT_PACKET_SIZE;
    node_data->received_packet = false;

    /*
    //get params
    das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
    }
    */

    set_node_private_data(call, node_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Unset Node

int unsetnode(call_t *call)
{
    node_data_t *node_data = get_node_private_data(call);

    free(node_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap

int bootstrap(call_t *call)
{
    entity_data_t *entity_data = get_entity_private_data(call);
    node_data_t *node_data = get_node_private_data(call);
    int i = get_entity_links_down_nbr(call);
    entityid_t *down = get_entity_links_down(call);
    node_data->overhead = 0;

    //get overhead
    while(i--)
    {
	call_t call_down = {down[i], call->node, call->entity};

	if(get_entity_type(&call_down) == MODELTYPE_ROUTING ||
	    get_entity_type(&call_down) == MODELTYPE_MAC)
	    node_data->overhead += GET_HEADER_SIZE(&call_down);
    }

    if(call->node == entity_data->source.id)
	scheduler_add_callback(DEFAULT_START_TIME, call, call_back, NULL);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// TX functions

int call_back(call_t *call, void *args)
{
    entity_data_t *entity_data = get_entity_private_data(call);
    node_data_t *node_data = get_node_private_data(call);
    call_t call_down = CALL_DOWN(call);
    destination_t num_targets = EMPTY_DESTINATION;

    entity_data->source.position = *get_node_position(entity_data->source.id);
    packet_t *packet = packet_create(call, node_data->packet_size +
	node_data->overhead, -1);
    num_targets.id = entity_data->num_targets;

    if(SET_HEADER(&call_down, packet, &num_targets) == ERROR)
    {
	fprintf(stderr, "[ERR] Couldn't route, error in module down from "
	    "multicast\n");
	packet_dealloc(packet);
	return 0;
    }

    entity_data->num_tx++;
    scheduler_add_callback(TX_START_TIME, call, tx_start, (void*)packet);
    return 0;
}

int tx_start(call_t *call, void *args)
{
    call_t call_down = CALL_DOWN(call);
    TX(&call_down, (packet_t*)args);
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
    entity_data_t *entity_data = get_entity_private_data(call);
    node_data_t *node_data = get_node_private_data(call);


    if(!node_data->received_packet)
    {
	entity_data->latency = get_time() - TX_START_TIME;
	node_data->received_packet = TRUE;
	entity_data->num_rx++;
    }
    packet_dealloc(packet);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Helper Functions

bool compare_destinations(destination_t *l, destination_t *r)
{
    if(l->id != r->id ||
	l->position.x != r->position.x ||
	l->position.y != r->position.y ||
	r->position.z != r->position.z)
	return false;
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Application Methods

application_methods_t methods =
{
	rx
};
