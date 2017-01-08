//James Robinson
//11/14/2016
//module for Dijkstra's algorithm (Shortest Path, geocasting)

#include <unistd.h>
#include <stdbool.h>

#include "include/modelutils.h"

////////////////////////////////////////////////////////////////////////////////
// Model Info

model_t model =
{
    "Geocasting Dijkstra's Algorithm",
    "James Robinson",
    "0.1", 
    MODELTYPE_ROUTING,
    {NULL, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Defines

#define LOG_TOPO_G
#define LOG_TOPO

#define ERROR -1
#define NONE -2

#define DEFAULT_START_TIME 0
#define PERIOD 1000000000

#define CALL_DOWN(call) {get_entity_bindings_down(call)->elts[0], call->node,\
	call->entity}
#define NODE_DATA(call) get_node_private_data(call)
#define ENTITY_DATA(call) get_entity_private_data(call)
#define PACKET_HEADER(packet, node_data) (header_t*)(packet->data +\
	node_data->overhead)
#define NEW(type) malloc(sizeof(type))

#define EMPTY_POSITION {-1, -1, -1}
#define DEFAULT_MAC_ADDR {BROADCAST_ADDR, EMPTY_POSITION}
#define THIS_DESTINATION(call) {call->node, *get_node_position(call->node)}

////////////////////////////////////////////////////////////////////////////////
// Typedefs and Structs

typedef enum {HELLO_PACKET, D_PACKET} packet_e;

//visited node data
typedef struct
{
    nodeid_t this;
    nodeid_t prev;
    double distance;
}visited_node_t;

//Routing Header
typedef struct
{
    destination_t sender;
    destination_t dest;
    packet_e type;
    void *path;
}header_t;

//individual node data
typedef struct
{
    void *nbrs;
    int overhead;
}node_data_t;

//global entity data
typedef struct
{
    float scale_postscript;
}entity_data_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes

int init(call_t *call, void *params);
int destroy(call_t *call);
int setnode(call_t *call, void *params);
int unsetnode(call_t *call);
int bootstrap(call_t *call);
int set_header(call_t *call, packet_t *packet, destination_t *dest);
void* get_shortest_path(call_t *call, destination_t *dest);
bool check_in_visited(void *visited, destination_t *to_check);
visited_node_t* get_node(void *visited, nodeid_t to_get);
int get_header_size(call_t *call);
int get_header_real_size(call_t *call);
int hello_callback(call_t *call, void* args);
void add_to_nbr(call_t *call, packet_t *packet);
void tx(call_t *call, packet_t *packet);
void rx(call_t *call, packet_t *packet);
bool compare_positions(position_t *l, position_t *r);
bool compare_destinations(destination_t *l, destination_t *r);
void topo_post_axes(call_t *call);
void topo_post_nodes(call_t *call);
void topo_post_data(call_t *call);


////////////////////////////////////////////////////////////////////////////////
// Initialization

//inits application entry and global entity parameters from config file
int init(call_t *call, void *params)
{
    entity_data_t *entity_data = NULL;
    if((entity_data = NEW(entity_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate entity data\nError in routing"
	    " module\n");
	return ERROR;
    }

#if defined LOG_TOPO_G
    position_t *topo_pos = get_topology_area();
    entity_data->scale_postscript = 1700.0 / (topo_pos->x + 20);
#endif

    set_entity_private_data(call, entity_data);

#ifdef LOG_TOPO_G
    if(access("topo_graph.ps", F_OK) != ERROR)
	remove("topo_graph.ps");
#endif
#ifdef LOG_TOPO
    if(access("topo.data", F_OK) != ERROR)
	remove("topo.data");
#endif
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

//free entity data at end of simulation
int destroy(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
#ifdef LOG_TOPO_G
    topo_post_axes(call);
#endif

    free(entity_data);
    entity_data = NULL;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

//bind applciation entity to a node and read node params from config file
//called during node creation when binding this particular module
int setnode(call_t *call, void* params)
{
    node_data_t *node_data = NULL;
    if((node_data = NEW(node_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate node data\nError in routing "
	    "module\n");
	return ERROR;
    }
    //uncomment later if adding parameters to config file
    /*param_t *params;*/

    if((node_data->nbrs = das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate neighbor list\nError in "
	    "rouing module\n");
	free(node_data);
	node_data = NULL;
	return ERROR;
    }
    node_data->overhead = NONE;

    /* uncomment if adding parameters later
    //get parameters
    das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
    }
    */

    set_node_private_data(call, node_data);

#ifdef LOG_TOPO
    topo_post_data(call);
#endif
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Unset Node

//called at node death to unbind application from nod
int unsetnode(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *dest;

    //add nodes to postscript files
#ifdef LOG_TOPO_G
    topo_post_nodes(call);
#endif

    //delete neighbor list
    das_init_traverse(node_data->nbrs);
    while((dest = (destination_t*)das_pop(node_data->nbrs)) != NULL)
	free(dest);
    das_destroy(node_data->nbrs);

    free(node_data);
    node_data = NULL;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap

//function called at node birth
int bootstrap(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    call_t call_down = CALL_DOWN(call);

    node_data->overhead = GET_HEADER_SIZE(&call_down);

    scheduler_add_callback(DEFAULT_START_TIME, call, hello_callback, NULL);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Header Functions

//exported and used by application layer to initialize routing header for
//packet
int set_header(call_t *call, packet_t *packet, destination_t *dest)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);
    if((header->path = get_shortest_path(call, dest)) == NULL)
    {
	fprintf(stderr, "[ERR] No shortest path found\n");
	return ERROR;
    }
    header->sender = my_pos;
    header->dest = *dest;
    header->type = D_PACKET;
    return 0;
}

void* get_shortest_path(call_t *call, destination_t *dest)
{
    node_data_t *node_data = NODE_DATA(call);
    call_t call_next = *call;
    destination_t *tmp = NULL;
    visited_node_t *tmp1 = NEW(visited_node_t), *tmp2 = NULL,
	*new = NEW(visited_node_t), *last_found = NEW(visited_node_t);
    void *visited = das_create(), *to_check = das_create();
    int num_targets = 0, num_found = 0, i;

    for(i = 0; i < get_node_count(); ++i)
    {
	if(i == call->node)
	    continue;
	destination_t tmp_dest = {i, *get_node_position(i)};

	if(tmp_dest.position.x >= dest->position.x - 1 &&
	   tmp_dest.position.x <= dest->position.x + 1 &&
	   tmp_dest.position.y >= dest->position.y - 1 &&
	   tmp_dest.position.y <= dest->position.y + 1)
	    ++num_targets;
    }

    //add source node to lists as top of tree
    tmp1->this = call->node;
    tmp1->prev = NONE;
    tmp1->distance = 0;
    *new = *tmp1;
    das_insert(to_check, (void*)tmp1);
    tmp1 = NEW(visited_node_t);
    *tmp1 = *new;
    das_insert(visited, (void*)tmp1);
    new->this = NONE;
    *last_found = *new;

    //while there are nodes to expand
    while(das_getsize(to_check) != 0)
    {
	bool nodes_visited = true;

	//while there are nodes in to_check
	das_init_traverse(to_check);
	while((tmp1 = (visited_node_t*)das_traverse(to_check)) != NULL)
	{
	    //remove previous node from to_check if no neigbors were checked
	    if(!nodes_visited)
	    {
		das_delete(to_check, (void*)tmp2);
		tmp2 = NULL;
	    }
	    nodes_visited = false;

	    call_next.node = tmp1->this;
	    node_data = NODE_DATA(&call_next);

	    //check each nodes neighbor list
	    das_init_traverse(node_data->nbrs);
	    while((tmp = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	    {
		//if neighbor isn't already visited
		if(!check_in_visited(visited, tmp))
		{
		    double dist = distance(get_node_position(tmp1->this),
			&tmp->position) + tmp1->distance;
		    if(new->this == NONE || dist < new->distance)
		    {
			new->this = tmp->id;
			new->prev = tmp1->this;
			new->distance = dist;
		    }
		    nodes_visited = true;
		}
	    }
	    if(nodes_visited == false)
		tmp2 = tmp1;
	}

	//duplicate check for visited neighbors for last node in list since loop
	//will exit before check is performed
	if(!nodes_visited)
	{
	    das_delete(to_check, (void*)tmp2);
	    tmp2 = NULL;
	}
	//add next hop to lists
	if(new->this != NONE)
	{
	    position_t tmp = *get_node_position(new->this);

	    if(tmp.x >= dest->position.x - 1 &&
		tmp.x <= dest->position.x + 1 &&
		tmp.y >= dest->position.y - 1 &&
		tmp.y <= dest->position.y + 1)
	    {
		*last_found = *new;
		++num_found;
	    }

	    if(num_found == num_targets)
		break;
	    tmp2 = NEW(visited_node_t);
	    *tmp2 = *new;
	    das_insert(to_check, (void*)new);
	    new = NEW(visited_node_t);
	    *new = *tmp2;
	    das_insert(visited, (void*)new);
	    new = NEW(visited_node_t);
	    new->this = NONE;
	    new->prev = NONE;
	    new->distance = 0;
	    free(tmp2);
	    tmp2 = NULL;
	}
    }

    //create path structure
    if(last_found->this == NONE)
	return NULL;
    void* path = das_create();
    nodeid_t* next = NEW(int);
    *next = last_found->this;
    das_insert(path, (void*)next);
    while((last_found = get_node(visited, last_found->prev)) != NULL)
    {
	if(last_found->this != call->node)
	{
	    next = NEW(int);
	    *next = last_found->this;
	    das_insert(path, (void*)next);
	}
    }

    //deallocate used memory
    while((new = (visited_node_t*)das_pop(to_check)) != NULL)
	free(new);
    das_destroy(to_check);
    while((new = (visited_node_t*)das_pop(to_check)) != NULL)
	free(new);
    das_destroy(visited);
    to_check = NULL;
    visited = NULL;

fprintf(stderr, "[ERR] printing path: ");
das_init_traverse(path);
nodeid_t *id = NULL;
while((id = (nodeid_t*)das_traverse(path)) != NULL)
fprintf(stderr, "%d, ", *id);
fprintf(stderr, "\n");

    return path;
}

bool check_in_visited(void* visited, destination_t *to_check)
{
    visited_node_t *tmp = NULL;
    das_init_traverse(visited);
    while((tmp = (visited_node_t*)das_traverse(visited)) != NULL)
	if(to_check->id == tmp->this)
	    return true;
    return false;
}

visited_node_t* get_node(void* visited, nodeid_t to_get)
{
    visited_node_t *tmp = NULL;

    das_init_traverse(visited);
    while((tmp = (visited_node_t*)das_traverse(visited)) != NULL)
    {
	if(tmp->this == to_get)
	    return tmp;
    }
    return NULL;
}

//exported and used by application layer to gt necessary size for packet
//allocation
int get_header_size(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);

    if(node_data->overhead == NONE)
    {
	call_t call_down = CALL_DOWN(call);
	node_data->overhead = GET_HEADER_SIZE(&call_down);
    }
    return node_data->overhead + sizeof(header_t);
}

//same aas get header but for real size
int get_header_real_size(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);

    if(node_data->overhead == NONE)
    {
	call_t call_down = CALL_DOWN(call);
	node_data->overhead = GET_HEADER_SIZE(&call_down);
    }
    return node_data->overhead + sizeof(header_t);
}

int hello_callback(call_t *call, void *args)
{
    node_data_t *node_data = NODE_DATA(call);
    call_t call_down = CALL_DOWN(call);
    destination_t destination = DEFAULT_MAC_ADDR,
	my_pos = THIS_DESTINATION(call);
    packet_t *packet = NULL;
    if((packet = packet_alloc(call, node_data->overhead + sizeof(header_t)))
	== NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate hello packet\n");
	return ERROR;
    }
    header_t *header = PACKET_HEADER(packet, node_data);

    if(SET_HEADER(&call_down, packet, &destination) == ERROR)
    {
	fprintf(stderr, "[ERR] Error setting MAC header\n");
	packet_dealloc(packet);
	return ERROR;
    }

    header->dest = destination;
    header->sender = my_pos;
    header->type = HELLO_PACKET;

    TX(&call_down, packet);
    return 0;
}

void add_to_nbr(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t *tmp = NULL;

    das_init_traverse(node_data->nbrs);
    while((tmp = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	if(tmp->id == header->sender.id)
	    return;
#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] Nbrs: node %d adds %d to neighbor list\n", call->node,
	header->sender.id);
#endif
    tmp = NEW(destination_t);
    *tmp = header->sender;
    das_insert(node_data->nbrs, (void*)tmp);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// tx

void tx(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    call_t call_down = CALL_DOWN(call);
    destination_t next_hop, my_pos = THIS_DESTINATION(call);
    header->sender = my_pos;

    next_hop.id = *((nodeid_t*)das_pop(header->path));
    next_hop.position.x = get_node_position(next_hop.id)->x;
    next_hop.position.y = get_node_position(next_hop.id)->y;
    next_hop.position.z = get_node_position(next_hop.id)->z;

    if(SET_HEADER(&call_down, packet, &next_hop) == ERROR)
    {
	fprintf(stderr, "[ERR] Unable to set MAC header\n");
	packet_dealloc(packet);
	packet = NULL;
	return;
    }
    TX(&call_down, packet);

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] sending optimal path packet from %d to %d\n",
	header->sender.id, next_hop.id);
#endif
    return;
}

////////////////////////////////////////////////////////////////////////////////
// rx

void rx(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    array_t *up = get_entity_bindings_up(call);
    int i = up->size;

    switch(header->type)
    {
	case HELLO_PACKET:
	    add_to_nbr(call, packet);
	    break;
	case D_PACKET:
	    if(das_getsize(header->path) == 0)
	    {
		while(i--)
		{
		    call_t call_up = {up->elts[i], call->node, call->entity};
		    RX(&call_up, packet);
		}
		return;
	    }
	    tx(call, packet);
	    break;
	default:
	    break;
    }
    return;
}

////////////////////////////////////////////////////////////////////////////////
// General Helper Functions

bool compare_positions(position_t *l, position_t *r)
{
    if(l->x == r->x && l->y == r->y && l->z == r->z)
	return true;
    return false;
}

bool compare_destinations(destination_t *l, destination_t *r)
{
    if(l->id == r->id && compare_positions(&l->position, &r->position))
	return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Postscript Functions

void topo_post_axes(call_t *call)
{
    FILE *graph;
    position_t *topo_pos = get_topology_area();
    entity_data_t *entity_data = ENTITY_DATA(call);

    char *name = (char*)malloc(15 * sizeof(int));
    sprintf(name, "topo_graph.ps");

    if((graph = fopen(name, "a")) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't open file %s in topo_post_axes()\n[ERR]"
	    " Error opening postscript file\n", name);
	return;
    }

    fprintf(graph, "15 15 moveto %lf 15 lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript + 15);
    fprintf(graph, "15 15 moveto 15 %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript + 15);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript / 4.0 + 15, topo_pos->x *
	entity_data->scale_postscript / 4.0 + 15);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript / 4.0 + 15, topo_pos->y *
	entity_data->scale_postscript / 4.0 + 15);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript / 2.0 + 15, topo_pos->x *
	entity_data->scale_postscript / 2.0 + 15);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript / 2.0 + 15, topo_pos->y *
	entity_data->scale_postscript / 2.0 + 15);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript * 3.0 / 4.0 + 15, topo_pos->x *
	entity_data->scale_postscript * 3.0 / 4.0 + 15);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript * 3.0 / 4.0 + 15, topo_pos->y *
	entity_data->scale_postscript * 3.0 / 4.0 + 15);

    fprintf(graph, "0.2 setlinewidth\n");

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript / 4.0 + 15, topo_pos->x *
	entity_data->scale_postscript / 4.0 + 15, topo_pos->x *
	entity_data->scale_postscript + 15);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript / 4.0 + 15, topo_pos->y *
	entity_data->scale_postscript + 15, topo_pos->y *
	entity_data->scale_postscript / 4.0 + 15);

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript / 2.0 + 15, topo_pos->x *
	entity_data->scale_postscript / 2.0 + 15, topo_pos->x *
	entity_data->scale_postscript + 15);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript / 2.0 + 15, topo_pos->y *
	entity_data->scale_postscript + 15, topo_pos->y *
	entity_data->scale_postscript / 2.0 + 15);

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
	entity_data->scale_postscript * 3.0 / 4.0 + 15, topo_pos->x *
	entity_data->scale_postscript * 3.0 / 4.0 + 15, topo_pos->x *
	entity_data->scale_postscript + 15);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
	entity_data->scale_postscript * 3.0 / 4.0 + 15, topo_pos->y *
	entity_data->scale_postscript + 15, topo_pos->y *
	entity_data->scale_postscript * 3.0 / 4.0 + 15);

    fclose(graph);
    return;
}

void topo_post_nodes(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    entity_data_t *entity_data = ENTITY_DATA(call);
    destination_t *nbr = NULL;
    position_t *pos = get_node_position(call->node);
    FILE *graph;

    char *name = (char*)malloc(15 * sizeof(int));
    sprintf(name, "topo_graph.ps");

    if((graph = fopen(name, "a")) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't open file %s in topo_post_nodes\n[ERR] "
	    "Error opening postscript\n", name);
	return;
    }

    if(call->node == 0)
	fprintf(graph, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0"
	    " 0 0 setrbgcolor\n", pos->x * entity_data->scale_postscript + 15,
	    pos->y * entity_data->scale_postscript + 15);
    else
	fprintf(graph, "%lf %lf 2.5 0 360 arc fill stroke\n", pos->x *
	    entity_data->scale_postscript + 15, pos->y *
	    entity_data->scale_postscript + 15);
    fprintf(graph, "/Times-Roman findfont 14 scalefont setfont newpath %lf "
	"%lf moveto (%d) show\n", pos->x * entity_data->scale_postscript + 18,
	pos->y * entity_data->scale_postscript + 18, call->node);

    das_init_traverse(node_data->nbrs);
    while((nbr = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	fprintf(graph, "%lf %lf moveto %lf %lf lineto stroke\n", pos->x *
	    entity_data->scale_postscript + 15, pos->y *
	    entity_data->scale_postscript + 15, nbr->position.x *
	    entity_data->scale_postscript + 15, nbr->position.y *
	    entity_data->scale_postscript + 15);

    fclose(graph);
    return;
}

void topo_post_data(call_t *call)
{
    destination_t my_pos = THIS_DESTINATION(call);
    FILE *data;

    char *name = (char*)malloc(9 * sizeof(int));
    sprintf(name, "topo.data");

    if((data = fopen(name, "a")) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't open file %s in topo_post_data\n[ERR] "
	    "Error opening data file\n", name);
	return;
    }

    fprintf(data, "%d %f %f %f\n", my_pos.id, my_pos.position.x,
	my_pos.position.y, my_pos.position.z);
    fclose(data);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Routing Methods

routing_methods_t methods =
{
rx,
tx,
set_header,
get_header_size,
get_header_real_size,
};
