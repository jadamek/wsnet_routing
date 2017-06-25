// msb.c
// generic multi-source broadcast application module
// James Robinson
// 6/23/2017

#include <stdio.h>
#include <stdbool.h>
#include <include/modelutils.h>

////////////////////////////////////////////////////////////////////////////////
// Model Information

model_t model =
{
	"MSB",
	"James Robinson",
	"0.1",
	MODELTYPE_APPLICATION,
	{NULL, 0}
};

///////////////////////////////////////////////////////////////////////////////
// Defines

#define DEFAULT_PACKET_SIZE 10
#define TX_START_TIME 0
#define DEFAULT_START_TIME 0

#define ERROR -1
#define NONE -2
#define EMPTY_DESTINATION {NONE, {-1, -1, -1}}

#define NODE_DATA(call) get_node_private_data(call)
#define ENTITY_DATA(call) get_entity_private_data(call)
#define CALL_DOWN(call) {get_entity_bindings_down(call)->elts[0], call->node,\
	call->entity}
#define NEW(type) malloc(sizeof(type))

#define TIME_PRECISION 1000000000

////////////////////////////////////////////////////////////////////////////////
// Struct and Typedefs

typedef struct
{
	int overhead;
	int packet_size;
	bool received_packet;
}node_data_t;

typedef struct
{
	void *source;
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
    entity_data_t *entity_data;
    //param_t *param;

    if((entity_data = NEW(entity_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate entity data in MSB module\n");
	return ERROR;
    }

    if((entity_data->source = das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate destination list in MSB"
	    " module\n");
	free(entity_data);
	return ERROR;
    }
    entity_data->num_tx = 0;
    entity_data->num_rx = 0;
    entity_data->latency = 0;

    /*das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
    }*/

    set_entity_private_data(call, entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    destination_t *tmp = NULL;
    FILE *results;
    char *name = (char*)malloc(11 *sizeof(int));
    sprintf(name, "results.txt");

    fprintf(stderr, "Application Statistics:\n");
    fprintf(stderr, "  messages sent from: ");
    das_init_traverse(entity_data->source);
    while((tmp = (destination_t*)das_traverse(entity_data->source)) != NULL)
	fprintf(stderr, "%d, ", tmp->id);
    fprintf(stderr, "\n  num messages sent: %d\n", entity_data->num_tx);
    fprintf(stderr, "  num messages received: %d\n", entity_data->num_rx);
    fprintf(stderr, "  message delivery rate: %f\n",
	(double)entity_data->num_rx / (double)entity_data->num_tx);
    fprintf(stderr, "  latency: %lld nanoseconds\n", entity_data->latency);

    if((results = fopen(name, "a")) == NULL)
	fprintf(stderr, "[ERR] Can't open file: %s\n", name);
    else
    {
	fprintf(results, "%d\t%d\t%lld\t", entity_data->num_tx,
	    entity_data->num_rx, entity_data->latency);
	fclose(results);
    }

    das_init_traverse(entity_data->source);
    while((tmp = (destination_t*)das_traverse(entity_data->source)) != NULL)
	free(tmp);
    das_destroy(entity_data->source);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

int setnode(call_t *call, void *params)
{
    node_data_t *node_data;
    //param_t *param;*/
    if((node_data = NEW(node_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate node data in MSB module\n");
	destroy(call);
	return ERROR;
    }

    node_data->overhead = 0;
    node_data->packet_size = DEFAULT_PACKET_SIZE;
    node_data->received_packet = false;

    /*das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
    }*/

    set_node_private_data(call, node_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Unset Node

int unsetnode(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    free(node_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap

int bootstrap(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    int i = get_entity_links_down_nbr(call);
    entityid_t *down = get_entity_links_down(call);

    while(i--)
    {
	call_t call_down = {down[i], call->node, call->entity};

	if(get_entity_type(&call_down) == MODELTYPE_ROUTING ||
	    get_entity_type(&call_down) == MODELTYPE_MAC)
		node_data->overhead += GET_HEADER_SIZE(&call_down);
    }

    call_t call_down = CALL_DOWN(call);
    if(GET_HEADER_REAL_SIZE(&call_down))
    {
	destination_t *new = NEW(destination_t);
	new->id = call->node;
	new->position = *get_node_position(call->node);
	das_insert(entity_data->source, (void*)new);
	scheduler_add_callback(DEFAULT_START_TIME, call, call_back, NULL);
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// TX Functions

int call_back(call_t *call, void *args)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    call_t call_down = CALL_DOWN(call);

    packet_t *packet = packet_create(call, node_data->packet_size +
	node_data->overhead, -1);

    if(SET_HEADER(&call_down, packet, NULL) == ERROR)
    {
	fprintf(stderr, "[ERR] Can't Set Header\n");
	packet_dealloc(packet);
	return ERROR;
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
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);

    if(!node_data->received_packet)
    {
	entity_data->latency = get_time() - TX_START_TIME;
	node_data->received_packet = true;
	entity_data->num_rx++;
    }
    packet_dealloc(packet);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Helper Functions

bool compare_destinations(destination_t *l, destination_t *r)
{
    if(l == NULL || r == NULL)
	return false;
    if(l->id != r->id ||
	l->position.x != r->position.x ||
	l->position.y != r->position.y ||
	l->position.z != r->position.z)
	return false;
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Application Methods

application_methods_t methods =
{
	rx
};
