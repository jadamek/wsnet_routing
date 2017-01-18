//James Robinson
//1/17/2017
//Module for testing Connected Dominating Set

#include <unistd.h>
#include <stdbool.h>

#include "include/modelutils.h"

////////////////////////////////////////////////////////////////////////////////
// Model Info

model_t model=
{
	"Connected Dominating Set",
	"James Robinson",
	"0.1",
	MODELTYPE_ROUTING,
	{NULL, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Defines

#define LOG_GG
#define LOG_TOPO_G
#define LOG_TOPO

#define ERROR -1
#define NONE -2
#define PRECISION 100000

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
#define NO_DESTINATION {NONE, EMPTY_POSITION}
#define THIS_DESTINATION(call) {call->node, *get_node_position(call->node)}

////////////////////////////////////////////////////////////////////////////////
// Typedefs and Structs

typedef struct
{
    destination_t sender;
    destination_t dest;
}header_t;

typedef struct
{
    void *nbrs;
    void *gg_list;
    int overhead;
}node_data_t;

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
int get_header_size(call_t *call);
int get_header_real_size(call_t *call);
int do_cds(call_t *call, destination_t *args);
void remove_from(void *list, destination_t *to_remove);
destination_t* has_common_nbr(void *l, void *r);
void add_first_link(call_t *call, void *l, void *r);
void planarize_graph(call_t *call, packet_t *packet);
bool node_present_in(void *das, destination_t *to_find);
int hello_callback(call_t *call, void *args);
bool check_in_geocast(destination_t *region, destination_t *to_check);
void tx(call_t *call, packet_t *packet);
void rx(call_t *call, packet_t *packet);
bool compare_positions(position_t *l, position_t *r);
bool compare_destinations(destination_t *l, destination_t *r);
void gg_post_axes(call_t *call);
void topo_post_axes(call_t *call);
void gg_post_nodes(call_t *call);
void topo_post_nodes(call_t *call);
void topo_post_data(call_t *call);



////////////////////////////////////////////////////////////////////////////////
// Initialization

int init(call_t *call, void *params)
{
    entity_data_t *entity_data = NULL;
    if((entity_data = NEW(entity_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate entity data\n[ERR] Error in "
	    "routing module\n");
	return ERROR;
    }

#if defined LOG_TOPO_G || defined LOG_GG
    position_t *topo_pos = get_topology_area();
    entity_data->scale_postscript = 1700.0 / (topo_pos->x + 20);
#endif
#ifdef LOG_TOPO_G
    if(access("topo_graph.ps", F_OK) != ERROR)
	remove("topo_graph.ps");
#endif
#ifdef LOG_GG
    if(access("gabriel_graph.ps", F_OK) != ERROR)
	remove("gabriel_graph.ps");
#endif
#ifdef LOG_TOPO
    if(access("topo.data", F_OK) != ERROR)
	remove("topo.data");
#endif

    set_entity_private_data(call, entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);

#ifdef LOG_GG
    gg_post_axes(call);
#endif
#ifdef LOG_TOPO_G
    topo_post_axes(call);
#endif

    free(entity_data);
    entity_data = NULL;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

int setnode(call_t *call, void *params)
{
    /*param_t *param;*/
    node_data_t *node_data = NULL;
    if((node_data = NEW(node_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate node data\n[ERR] Error in "
	    "routing module\n");
	return ERROR;
    }

    if((node_data->nbrs = das_create()) == NULL || (node_data->gg_list =
	das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate neighbor lists\n[ERR] Error in "
	    "routing module\n");
	free(node_data);
	node_data = NULL;
	return ERROR;
    }
    node_data->overhead = NONE;

    /*
    das_init_traverse(params)
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
// Un-Set Node

int unsetnode(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *dest;

#ifdef LOG_GG
    gg_post_nodes(call);
#endif
#ifdef LOG_TOPO_G
    topo_post_nodes(call);
#endif

    das_init_traverse(node_data->nbrs);
    while((dest = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	free(dest);
    das_destroy(node_data->nbrs);

    das_init_traverse(node_data->gg_list);
    while((dest = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	free(dest);
    das_destroy(node_data->gg_list);

    free(node_data);
    node_data = NULL;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap

int bootstrap(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    call_t call_down = CALL_DOWN(call);

    node_data->overhead = GET_HEADER_SIZE(&call_down);

    scheduler_add_callback(DEFAULT_START_TIME, call, hello_callback, NULL);
    scheduler_add_callback(DEFAULT_START_TIME + PERIOD, call, hello_callback,
	NULL);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Header Functions

int set_header(call_t *call, packet_t *packet, destination_t *dest)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    call_t call_down = CALL_DOWN(call);
    destination_t my_pos = THIS_DESTINATION(call), no_dest = NO_DESTINATION;

    header->sender = my_pos;
    header->dest = no_dest;

    do_cds(call, dest);

    return SET_HEADER(&call_down, packet, dest);
}

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

int get_header_real_size(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);

    if(node_data->overhead == NONE)
    {
	call_t call_down = CALL_DOWN(call);
	node_data->overhead = GET_HEADER_REAL_SIZE(&call_down);
    }

    return node_data->overhead + sizeof(header_t);
}

////////////////////////////////////////////////////////////////////////////////
// Graph Functions

int do_cds(call_t *call, destination_t *dest)
{
    void *possible = NULL, *dominators = NULL;
    if((possible = das_create()) == NULL || (dominators = das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate lists\n[ERR] Error in cds\n");
	return ERROR;
    }

    //get list of nodes in geocast region
    destination_t *tmp = NEW(destination_t), *tmp2 = NULL;
    int i;
    for(i = 0; i < get_node_count(); ++i)
    {
	if(tmp == NULL)
	    tmp = NEW(destination_t);
	tmp->id = i;
	tmp->position = *get_node_position(i);
	if(check_in_geocast(dest, tmp))
	{
	    destination_t *to_delete = NULL;
	    das_insert(possible, (void*)tmp);

	    call_t call_tmp = *call;
	    call_tmp.node = tmp->id;
	    node_data_t *tmp_node_data = NODE_DATA(&call_tmp);
	    while((to_delete = (destination_t*)das_pop(tmp_node_data->gg_list))
		!= NULL)
		free(to_delete);
	    to_delete = NULL;

	    das_init_traverse(tmp_node_data->nbrs);
	    while((to_delete =
		(destination_t*)das_traverse(tmp_node_data->nbrs)) != NULL)
	    {
		call_t call_delete = *call;
		call_delete.node = to_delete->id;
		node_data_t *delete_node_data = NODE_DATA(&call_delete);
		remove_from(delete_node_data->gg_list, tmp);
	    }
	    tmp = NULL;
	}
    }

    //generate list of dominators
    while((tmp = (destination_t*)das_pop(possible)) != NULL)
    {
	das_insert(dominators, (void*)tmp);

	call_t call_tmp = *call;
	call_tmp.node = tmp->id;
	node_data_t *node_data = NODE_DATA(&call_tmp);

	das_init_traverse(node_data->nbrs);
	while((tmp2 = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	{
	    destination_t *to_add = NEW(destination_t);

	    *to_add = *tmp2;
	    remove_from(possible, tmp2);
	    das_insert(node_data->gg_list, (void*)to_add);

	    call_t call_neighbor = *call;
	    call_neighbor.node = to_add->id;
	    node_data_t *neighbor_node_data = NODE_DATA(&call_neighbor);
	    to_add = NEW(destination_t);
	    *to_add = *tmp;
	    das_insert(neighbor_node_data->gg_list, (void*)to_add);
	}
    }
    das_destroy(possible);

    while((tmp = (destination_t*)das_pop(dominators)) != NULL)
    {
	call_t call_tmp = *call;
	call_tmp.node = tmp->id;
	node_data_t *tmp_node_data = NODE_DATA(&call_tmp);
	das_init_traverse(dominators);
	while((tmp2 = (destination_t*)das_traverse(dominators)) != NULL)
	{
	    call_t call_tmp2 = *call;
	    call_tmp2.node = tmp2->id;
	    node_data_t *tmp2_node_data = NODE_DATA(&call_tmp2);
	    if(has_common_nbr(tmp_node_data->gg_list,
		tmp2_node_data->gg_list) == NULL)
	    {
		add_first_link(call, tmp_node_data->gg_list,
		    tmp2_node_data->gg_list);
	    }
	}
	free(tmp);
    }
    das_destroy(dominators);

    return 0;
}

void remove_from(void* list, destination_t* to_remove)
{
    destination_t *tmp = NULL;

    das_init_traverse(list);
    while((tmp = (destination_t*)das_traverse(list)) != NULL)
    {
	if(compare_destinations(to_remove, tmp))
	{
	    das_delete(list, tmp);
	    return;
	}
    }
    return;
}

destination_t* has_common_nbr(void *l, void *r)
{
    destination_t *l_tmp = NULL, *r_tmp = NULL;
    das_init_traverse(l);
    while((l_tmp = (destination_t*)das_traverse(l)) != NULL)
    {
	das_init_traverse(r);
	while((r_tmp = (destination_t*)das_traverse(r)) != NULL)
	{
	    if(compare_destinations(l_tmp, r_tmp))
		return l_tmp;
	}
    }
    return NULL;
}

void add_first_link(call_t *call, void *l, void *r)
{
    destination_t *l_tmp = NULL, *result = NULL;

    das_init_traverse(l);
    while((l_tmp = (destination_t*)das_traverse(l)) != NULL)
    {
	call_t call_l = *call;
	call_l.node = l_tmp->id;
	node_data_t *l_node_data = NODE_DATA(&call_l);
	if((result = has_common_nbr(l_node_data->nbrs, r)) != NULL)
	{
	    call_t call_r = *call;
	    call_r.node = result->id;
	    node_data_t *r_node_data = NODE_DATA(&call_r);

	    destination_t *to_add = NEW(destination_t);
	    *to_add = *result;
	    das_insert(l_node_data->gg_list, (void*)to_add);
	    to_add = NEW(destination_t);
	    *to_add = *l_tmp;
	    das_insert(r_node_data->gg_list, (void*)to_add);
	}
    }
    return;
}

void planarize_graph(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t *tmp = NULL, my_pos = THIS_DESTINATION(call);
    if(!node_present_in(node_data->nbrs, &header->sender))
    {
#ifdef LOG_ROUTING
	PRINT_ROUTING("[RTG] GG: node %d adds %d to neighbor list\n",
	    call->node, header->sender.id);
#endif
	if((tmp = NEW(destination_t)) == NULL)
	{
	    fprintf(stderr, "[ERR] Can't allocate destination\n[ERR] Error in "
		"planarization\n");
	    return;
	}
	*tmp = header->sender;
	das_insert(node_data->nbrs, (void*)tmp);
	return;
    }

    position_t center;
    center.x = (header->sender.position.x + my_pos.position.x) / 2;
    center.y = (header->sender.position.y + my_pos.position.y) / 2;
    center.z = (header->sender.position.z + my_pos.position.z) / 2;
    double radius = distance(&center, &my_pos.position);

    das_init_traverse(node_data->nbrs);
    while((tmp = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	if(tmp->id != header->sender.id &&
	    distance(&tmp->position, &center) <= radius)
	    return;

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] GG: node %d adds %d to gg neighbor list\n",
	call->node, header->sender.id);
#endif
    if((tmp = NEW(destination_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate destination\n[ERR] Error in "
	    "planarization\n");
	return;
    }
    *tmp = header->sender;
    das_insert(node_data->gg_list, (void*)tmp);
    return;
}

bool node_present_in(void *das, destination_t *to_find)
{
    destination_t *tmp = NULL;

    das_init_traverse(das);
    while((tmp = (destination_t*)das_traverse(das)) != NULL)
	if(compare_destinations(tmp, to_find))
	    return true;
    return false;
}

int hello_callback(call_t *call, void* args)
{
    node_data_t *node_data = NODE_DATA(call);
    call_t call_down = CALL_DOWN(call);
    destination_t destination = DEFAULT_MAC_ADDR,
	my_pos = THIS_DESTINATION(call), no_dest = NO_DESTINATION;
    packet_t *packet = NULL;
    if((packet = packet_alloc(call, node_data->overhead + sizeof(header_t)))
	== NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate hello packet\n[ERR] Error in "
	    "hello_callback\n");
	return ERROR;
    }
    header_t *header = PACKET_HEADER(packet, node_data);

    if(SET_HEADER(&call_down, packet, &destination) == ERROR)
    {
	fprintf(stderr, "[ERR] Error setting MAC header\n");
	packet_dealloc(packet);
	return ERROR;
    }

    header->sender = my_pos;
    header->dest = no_dest;

    TX(&call_down, packet);
    return 0;
}

bool check_in_geocast(destination_t *region, destination_t *to_check)
{
    if((int)(to_check->position.x * PRECISION) >=
	(int)((region->position.x - 1) * PRECISION) &&
	(int)(to_check->position.x * PRECISION) <=
	(int)((region->position.x + 1) * PRECISION) &&
	(int)(to_check->position.y * PRECISION) >=
	(int)((region->position.y - 1) * PRECISION) &&
	(int)(to_check->position.y * PRECISION) <=
	(int)((region->position.y + 1) * PRECISION))
	return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// TX

void tx(call_t *call, packet_t *packet)
{
    packet_dealloc(packet);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// RX

void rx(call_t *call, packet_t *packet)
{
    planarize_graph(call, packet);
    packet_dealloc(packet);
    packet = NULL;
    return;
}

////////////////////////////////////////////////////////////////////////////////
// General Functions

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

void gg_post_axes(call_t *call)
{
    FILE *graph_gg;
    position_t *topo_pos = get_topology_area();
    entity_data_t *entity_data = ENTITY_DATA(call);

    char* name = (char*)malloc(25 * sizeof (int));
    sprintf(name, "gabriel_graph.ps");

    if((graph_gg = fopen(name, "a")) == NULL)
    {
        fprintf(stderr, "[ERR] Couldn't open file %s in destroy()\n", name);
        fprintf(stderr, "[ERR] Error opening postscript file\n");
        return;
    }

    fprintf(graph_gg, "15 15 moveto %lf 15 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript + 15);
    fprintf(graph_gg, "15 15 moveto 15 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript + 15);

    fprintf(graph_gg, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0);
    fprintf(graph_gg, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(graph_gg, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0);
    fprintf(graph_gg, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(graph_gg, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);
    fprintf(graph_gg, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);


    fprintf(graph_gg, "0.2 setlinewidth\n");

    fprintf(graph_gg, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(graph_gg, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(graph_gg, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(graph_gg, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(graph_gg, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(graph_gg, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);

    fclose(graph_gg);
    return;
}

void topo_post_axes(call_t *call)
{
    FILE *graph;
    position_t *topo_pos = get_topology_area();
    entity_data_t *entity_data = ENTITY_DATA(call);

    char *name = (char*)malloc(15 * sizeof(int));
    sprintf(name, "topo_graph.ps");

    if((graph = fopen(name, "a")) == NULL)
    {
        fprintf(stderr, "[ERR] Couldn't open file %s in destroy()\n", name);
        fprintf(stderr, "[ERR] Error opening postscript file\n");
        return;
    }

    fprintf(graph, "15 15 moveto %lf 15 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript + 15);
    fprintf(graph, "15 15 moveto 15 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript + 15);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);


    fprintf(graph, "0.2 setlinewidth\n");

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);

    fclose(graph);
    return;
}

void gg_post_nodes(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    entity_data_t *entity_data = ENTITY_DATA(call);
    destination_t *nbr = NULL;
    position_t *pos = get_node_position(call->node);
    FILE *graph_gg;

    char *name = (char*)malloc(25 * sizeof(int));
    sprintf(name, "gabriel_graph.ps");

    if((graph_gg = fopen(name, "a")) == NULL)
    {
        fprintf(stderr, "[ERR] Couldn't open file %s\nError opening postscript "
            "file\n", name);
        return;
    }

    if(call->node == 0)
        fprintf(graph_gg, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0"            " 0 0 setrgbcolor\n", pos->x * entity_data->scale_postscript + 15.0,
            pos->y * entity_data->scale_postscript + 15.0);
    else
        fprintf(graph_gg, "%lf %lf 2.5 0 360 arc fill stroke\n", pos->x *
            entity_data->scale_postscript + 15.0, pos->y *
            entity_data->scale_postscript + 15.0);
//original size 6
    fprintf(graph_gg, "/Times-Roman findfont 14 scalefont setfont newpath %lf "
        "%lf moveto (%d) show\n", pos->x * entity_data->scale_postscript + 18.0,
        pos->y * entity_data->scale_postscript + 18.0, call->node);

    das_init_traverse(node_data->gg_list);
    while((nbr = (destination_t*)das_traverse(node_data->gg_list)) != NULL)
    {
	fprintf(graph_gg, "%lf %lf moveto %lf %lf lineto stroke\n", pos->x *
	    entity_data->scale_postscript + 15.0, pos->y *
	    entity_data->scale_postscript + 15.0, nbr->position.x *
	    entity_data->scale_postscript + 15.0, nbr->position.y *
	    entity_data->scale_postscript + 15.0);
    }

    fclose(graph_gg);
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
        fprintf(stderr, "[ERR] Couldn't open file %s\nError opening postscript "
            "file\n", name);
        return;
    }

    if(call->node == 0)
        fprintf(graph, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0 0 "
            "0 setrgbcolor\n", pos->x * entity_data->scale_postscript + 15.0,
            pos->y * entity_data->scale_postscript + 15.0);
    else
        fprintf(graph, "%lf %lf 2.5 0 360 arc fill stroke\n", pos->x *
            entity_data->scale_postscript + 15.0, pos->y *
            entity_data->scale_postscript + 15.0);

    das_init_traverse(node_data->nbrs);
    while((nbr = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
        fprintf(graph, "%lf %lf moveto %lf %lf lineto stroke\n", pos->x *
            entity_data->scale_postscript + 15.0, pos->y *
            entity_data->scale_postscript + 15.0, nbr->position.x *
            entity_data->scale_postscript + 15.0, nbr->position.y *
            entity_data->scale_postscript + 15.0);

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
        fprintf(stderr, "[ERR] Couldn't open file %s\nError opening postscript "
            "file\n", name);
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
