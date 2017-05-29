// flooding+offgg.c
// Stateful Flooding Module with Geocasting Dijkstra and Connected
// Dominating Set, plus error rate
// James Robinson
// 12/11/2016

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

#include <include/modelutils.h>
//#define LOG_ROUTING
////////////////////////////////////////////////////////////////////////////////
// Model Info

model_t model = 
{
    "Stateful Flooding with BFS, CDS, and error rate",
    "Jordan Adamek",
    "0.1",
    MODELTYPE_ROUTING,
    {NULL, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Defines

//comment or uncomment depending on what output files desired
#define LOG_TOPO_G
#define LOG_GG
#define LOG_TOPO

//getting rid of constants in code
#define ERROR -1
//-1 used for broadcast address (defined in modelutils.h)
//so -2 used for no address in destination
#define NONE -2

#define DEFAULT_PACKET_SIZE 10
//time that first run of planarization algorithm starts in nanoseconds
#define DEFAULT_START_TIME 0
//period between executions of planarization algorithm in nanoseconds
#define PERIOD 1000000000
#define DEFAULT_TTL 55

//how many decimal places are used in calculations
#define PRECISION 100000

//communication radius or radio, in meters
#define RADIO_RANGE 100

//length (or diameter) of geocast region, in meters
#define REGION_SIZE 200

//default addresses
#define EMPTY_POSITION {-1, -1 -1}
#define DEFAULT_MAC_ADDR {BROADCAST_ADDR, EMPTY_POSITION}
#define NO_DESTINATION {NONE, EMPTY_POSITION}
#define THIS_DESTINATION(call) {call->node, *get_node_position(call->node)}

//shit I got tired of writing over and over
#define CALL_DOWN(call) {get_entity_bindings_down(call)->elts[0], call->node,\
	call->entity}
#define NODE_DATA(call) get_node_private_data(call)
#define ENTITY_DATA(call) get_entity_private_data(call)
#define PACKET_HEADER(packet, node_data) (header_t*)(packet->data + \
	node_data->overhead)
#define NEW(type) malloc(sizeof(type))

////////////////////////////////////////////////////////////////////////////////
// Structures and Typedefs

typedef enum {HELLO_PACKET, DATA_PACKET, LM_PACKET, D_PACKET} packet_e;
typedef enum {NO_DIR, TRAVERSE_R, TRAVERSE_L, BOTH} direction_e;
typedef enum {NO_INT, INTERSECTION, COLLINEAR} intersection_e;
typedef enum {NOT_FOUND, FOUND_THIS, FOUND_OTHER_1, FOUND_OTHER_2} mate_e;
typedef enum {DONT_INCREMENT, INCREMENT} operation_e;

/*destination type defined in types.h, included here for reference
typedef struct _destination {
    nodeid_t id;
    position_t position;
} destination_t;
*/

typedef struct
{
    nodeid_t this;
    nodeid_t prev;
}visited_node_t;

//packet information needed for routing
typedef struct
{
    destination_t dest;
    destination_t src;
    destination_t sender;
    destination_t next_node;
    packet_e type;
    direction_e direction;
    int ttl;
    void *path;
}header_t;

//Global node data, used for tracking information known to node
typedef struct
{
    void *nbrs;
    void *gg_list;
    int overhead;
    bool received_packet;
    bool broadcast_received;
}node_data_t;

//Global entity data, used for tracking statistics
typedef struct
{
    double loss_rate;
    int num_reachable;
    int total_num_hops;
    int deliver_num_hops;
    int num_packets;
    float scale_postscript;
    uint64_t dijk_latency;
}entity_data_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes

//init/destroy node stat data
int init(call_t *call, void *params);
int destroy(call_t *call);

//bind routing module to node
int setnode(call_t *call, void *params);
int unsetnode(call_t *call);

//startup instructions
int bootstrap(call_t *call);

//header functions
int set_header(call_t *call, packet_t *packet, destination_t *dest);
int get_header_size(call_t *call);
int get_header_real_size(call_t *call);

//graph functions
int do_cds(call_t *call, destination_t *dest);
void remove_from(void *list, destination_t *to_remove);
destination_t* has_common_nbr(void *l, void *r);
void add_first_link(call_t *call, void *l, void *r);
bool no_outside_nbr(void *nbr_list, destination_t *dest);
void add_outside_nbr(call_t *call, void *list, destination_t *dest);
void planarize_graph(call_t *call);
int hello_callback(call_t *call, void *args);
int start_dijk(call_t *call, destination_t *dest);
void* get_shortest_path(call_t *call, destination_t *dest);
bool check_in_visited(void *visited, destination_t *to_check);
visited_node_t* get_node(void *visited, nodeid_t to_get);


//tx
void tx(call_t *call, packet_t *packet);

//routing functions
void forward(call_t *call, packet_t *packet);
destination_t next_on_face(call_t *call, destination_t *start,
    destination_t *nodeRefs, direction_e direction);
bool check_in_geocast(destination_t *region, destination_t *to_check);
void traverse_first_face(call_t *call, packet_t *packet);
void flood(call_t *call, packet_t *packet);
void spray_packets(call_t *call, packet_t *packet);
void forward_packet(call_t *call, packet_t *packet, direction_e direction);
void deliver_packet(call_t *call, packet_t *packet);
bool combine_packets(call_t *call, destination_t *sender,
    destination_t *next_node, direction_e direction);

//receiving functions
void rx(call_t *call, packet_t *packet);
bool packet_lost(call_t *call);
mate_e find_mate(call_t *call, packet_t *packet);

//general helper functions
bool compare_positions(position_t *l, position_t *r);
bool compare_destinations(destination_t *l, destination_t *r);
intersection_e check_intersect(call_t *call, destination_t *source,
    destination_t *dest, destination_t *next);
bool node_present_in(void *das, destination_t *id);
bool on_gg(call_t *call);
packet_t* copy_packet(call_t *call, packet_t *packet, header_t* header,
    operation_e operation);
int set_mac_header_tx(call_t *call, packet_t *packet);

//debug file functions
void gg_post_axes(call_t *call);
void topo_post_axes(call_t *call);
void gg_post_nodes(call_t *call);
void topo_post_nodes(call_t *call);
void topo_post_data(call_t *call);

////////////////////////////////////////////////////////////////////////////////
// initialization
// initializes application entry and global entity parameters from config file

int init(call_t *call, void* params)
{
    param_t *param = NULL;
    entity_data_t *entity_data = NULL;
    if((entity_data = NEW(entity_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate entity data\nError in routing"
	    " module\n");
	return ERROR;
    }
    entity_data->deliver_num_hops = 0;
    entity_data->total_num_hops = 0;
    entity_data->num_packets = 1;
    entity_data->loss_rate = 0;
    entity_data->num_reachable = 0;

    das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
	if(!strcmp(param->key, "loss_rate"))
	    get_param_double_range(param->value, &(entity_data->loss_rate),
		0, 100);
    }

#if defined LOG_TOPO_G || defined LOG_GG
    position_t *topo_pos = get_topology_area();

    //initialize entity_data values
    entity_data->scale_postscript = 595.0 / (topo_pos->x + 20);
    //entity_data->scale_postscript = 1700.0 / (topo_pos->x + 20);
#endif
    //save entity private data
    set_entity_private_data(call, entity_data);

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
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy
// destroy entity at simulation end

//free entity_data and adds last info to postscript
int destroy(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    FILE *results;
    char *name = (char*)malloc(11 * sizeof(int));
    sprintf(name, "results.txt");

    fprintf(stderr, "Routing Statistics:\n");
    fprintf(stderr, "  number of packets: %d\n", entity_data->num_packets);
    fprintf(stderr, "  number of hops to deliver: %d\n",
	entity_data->deliver_num_hops);
    fprintf(stderr, "  total number of hops: %d\n",
	entity_data->total_num_hops);
    fprintf(stderr, "  BFS Latency: %lld nanoseconds\n",
	entity_data->dijk_latency);
    fprintf(stderr, "  Num reachable targets: %d %s\n",
	entity_data->num_reachable,
	(entity_data->num_reachable == 0 ? "<failed>" : ""));    

#ifdef LOG_GG
    gg_post_axes(call);
#endif
#ifdef LOG_TOPO_G
    topo_post_axes(call);
#endif
    if((results = fopen(name, "a")) == NULL)
	fprintf(stderr, "[ERR] Couldn't open file: results.txt\n");
    else
    {
	fprintf(results, "%d\t", entity_data->total_num_hops);
        fprintf(results, "%d\t", entity_data->num_packets);
        fprintf(results, "%lld\n", entity_data->dijk_latency);
    }

    free(entity_data);
    entity_data = NULL;
    if(results != NULL)
	fclose(results);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node
// bind application entity to a node and read node params from config file
// called during node creation when binding this particular module

int setnode(call_t *call, void* params)
{
    node_data_t *node_data = NULL;
    if((node_data = NEW(node_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate node data\nError in routing "
	    "module\n");
	return ERROR;
    }
    node_data->received_packet = false;
    node_data->broadcast_received = false;
    //uncomment later if adding parameters to config file
    /*param_t *param;*/

    if((node_data->nbrs = das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate neighbor list\nError in "
	    "routing module\n");
	free(node_data);
	node_data = NULL;
	return ERROR;
    }
    if((node_data->gg_list = das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate gg neighbor list\nError in "
	    "routing module\n");
	das_destroy(node_data->nbrs);
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
// Called at node death to unbind application from node

//deletes node info and adds nodes to postscript graphs
int unsetnode(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *dest;

    //add nodes to postscript files
#ifdef LOG_GG
    gg_post_nodes(call);
#endif
#ifdef LOG_TOPO_G
    topo_post_nodes(call);
#endif

    //delete neighbor list
    das_init_traverse(node_data->nbrs);
    while((dest = (destination_t*)das_pop(node_data->nbrs)) != NULL)
	free(dest);
    das_destroy(node_data->nbrs);

    //delete Gabriel Graph neighbor list
    das_init_traverse(node_data->gg_list);
    while((dest = (destination_t*)das_pop(node_data->gg_list)) != NULL)
	free(dest);
    das_destroy(node_data->gg_list);

    free(node_data);
    node_data = NULL;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap
// function called at node birth

//gets initial node overhead and schedules hello callback
int bootstrap(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    call_t call_down = CALL_DOWN(call);

    //get mac header overhead to send to application layer for packet allocation
    node_data->overhead = GET_HEADER_SIZE(&call_down);

    planarize_graph(call);

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Header Functions

//exported and used by application layer to initialize routing header for packet
//in that module
int set_header(call_t *call, packet_t *packet, destination_t *dest)
{
    do_cds(call, dest);
    if(start_dijk(call, dest) == ERROR)
	fprintf(stderr, "[ERR] unable to start shortest path\n");

    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    call_t call_down = CALL_DOWN(call);
    destination_t my_pos = THIS_DESTINATION(call), no_dest = NO_DESTINATION;

    header->src = my_pos;
    header->dest = *dest;
    header->sender = no_dest;
    header->next_node = no_dest;
    header->type = DATA_PACKET;
    header->direction = NO_DIR;
    header->ttl = DEFAULT_TTL;
    header->path = NULL;

    if(compare_destinations(&header->dest, &no_dest))
    {
	fprintf(stderr, "[ERR] Invalid destination\n");
	packet_dealloc(packet);
	packet = NULL;
	return ERROR;
    }
    else if(compare_destinations(&header->src, &no_dest))
    {
	fprintf(stderr, "[ERR] Invalid source\n");
	packet_dealloc(packet);
	packet = NULL;
	return ERROR;
    }

    //also call set_header in mac layer to initialize mac header for the
    //packet as well
    return SET_HEADER(&call_down, packet, dest);
}

//exported and used by application layer to get necessary size for packet
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

//same as get_header_size but for real size
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

//get connected dominating set for geocast region
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
	if(no_outside_nbr(tmp_node_data->gg_list, dest))
	    add_outside_nbr(call, tmp_node_data->gg_list, dest);
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

bool no_outside_nbr(void *nbr_list, destination_t *dest)
{
    destination_t *tmp = NULL;

    das_init_traverse(nbr_list);
    while((tmp = (destination_t*)das_traverse(nbr_list)) != NULL)
	if(!check_in_geocast(dest, tmp))
	    return false;
    return true;
}

void add_outside_nbr(call_t *call, void *list, destination_t *dest)
{
    destination_t *tmp = NULL, *nbr = NULL;

    das_init_traverse(list);
    while((tmp = (destination_t*)das_traverse(list)) != NULL)
    {
	call_t call_tmp = *call;
	call_tmp.node = tmp->id;
	node_data_t *tmp_node_data = NODE_DATA(&call_tmp);

	das_init_traverse(tmp_node_data->nbrs);
	while((nbr = (destination_t*)das_traverse(tmp_node_data->nbrs)) != NULL)
	{
	    if(!check_in_geocast(dest, nbr))
	    {
		destination_t *to_add = NEW(destination_t);
		*to_add = *nbr;
		das_insert(tmp_node_data->gg_list, (void*)to_add);
		to_add = NEW(destination_t);
		*to_add = *tmp;

		call_t call_nbr = *call;
		call_nbr.node = nbr->id;
		node_data_t *nbr_node_data = NODE_DATA(&call_nbr);
		das_insert(nbr_node_data->gg_list, (void*)to_add);
		return;
	    }
	}
    }
    return;
}

//planarization algorithm
void planarize_graph(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *tmp = NULL, my_pos = THIS_DESTINATION(call);

    int i;
    for(i = 0; i < get_node_count(); ++i)
    {
	if(i != call->node && distance(get_node_position(call->node),
	    get_node_position(i)) <= RADIO_RANGE)
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] GG: node %d adds %d to neighbor list\n",
		call->node, i);
#endif
	    tmp = NEW(destination_t);
	    tmp->id = i;
	    tmp->position = *get_node_position(i);
	    das_insert(node_data->nbrs, (void*)tmp);
	}
    }

    for(i = 0; i < das_getsize(node_data->nbrs); ++i)
    {
	int j = 0;
	destination_t *tmp2 = NULL;
	das_init_traverse(node_data->nbrs);
	while(j <= i)
	{
	    tmp = (destination_t*)das_traverse(node_data->nbrs);
	    ++j;
	}

	position_t center;
	center.x = (tmp->position.x + my_pos.position.x) / 2;
	center.y = (tmp->position.y + my_pos.position.y) / 2;
	center.z = (tmp->position.z + my_pos.position.z) / 2;
	double radius = distance(&center, &my_pos.position);

	bool witness = false;
	das_init_traverse(node_data->nbrs);
	while((tmp2 = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	{
	    if(tmp2->id != tmp->id && distance(&tmp2->position, &center)
		<= radius)
	    {
		witness = true;
		break;
	    }
	}

	if(!witness)
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] GG: node %d adds %d to gg neighbor list\n",
		call->node, tmp->id);
#endif
	    tmp2 = NEW(destination_t);
	    *tmp2 = *tmp;
	    das_insert(node_data->gg_list, (void*)tmp2);
	}
    }
    return;
}

//checks whether or not a node is present in a given data structure
bool node_present_in(void *das, destination_t *identify)
{
    das_init_traverse(das);
    destination_t *tmp = NULL;

    while((tmp = (destination_t*)das_traverse(das)) != NULL)
    {
	if(compare_destinations(tmp, identify))
	    return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Dijkstra Functions

int start_dijk(call_t *call, destination_t *dest)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    entity_data->dijk_latency = get_time();
    node_data_t *node_data = NODE_DATA(call);
    destination_t my_pos = THIS_DESTINATION(call), no_dest = NO_DESTINATION;
    packet_t *packet = NULL;
    if((packet = packet_alloc(call, node_data->overhead + sizeof(header_t) +
	DEFAULT_PACKET_SIZE))
	== NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate Dijkstra packet\nError setting"
	    " routing header\n");
	return ERROR;
    }
    header_t *header = PACKET_HEADER(packet, node_data);

    header->dest = *dest;
    header->src = my_pos;
    header->sender = my_pos;
    header->next_node = no_dest;
    header->type = D_PACKET;
    header->direction = NO_DIR;
    header->ttl = 0;

    if((header->path = get_shortest_path(call, dest)) == NULL)
    {
	fprintf(stderr, "[ERR] No shortest path found\n");
	return ERROR;
    }
    tx(call, packet);
    return 0;
}

void* get_shortest_path(call_t *call, destination_t *dest)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    call_t call_next = *call;
    destination_t *tmp = NULL;
    visited_node_t *tmp1 = NEW(visited_node_t), *tmp2 = NULL,
	*new = NEW(visited_node_t), *last_found = NEW(visited_node_t);
    void *visited = das_create(), *to_check = das_create();
    int num_targets = 0, i;

    //get total num of possible targets
    for(i = 0; i < get_node_count(); ++i)
    {
	if(i == call->node)
	    continue;
	destination_t tmp_dest = {i, *get_node_position(i)};

	if(check_in_geocast(dest, &tmp_dest))
	    ++num_targets;
    }

    //add source to list as top of tree
    tmp1->this = call->node;
    tmp1->prev = NONE;
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
	//while there nodes in to_check
	das_init_traverse(to_check);
	while((tmp1 = (visited_node_t*)das_traverse(to_check)) != NULL)
	{
	    if(tmp2 != NULL)
	    {
		das_delete(to_check, (void*)tmp2);
		tmp2 = NULL;
	    }
	    if(das_getsize(to_check) == 0)
		break;
	    call_next.node = tmp1->this;
	    node_data = NODE_DATA(&call_next);
	    //check nodes neighbor list
	    das_init_traverse(node_data->gg_list);
	    while((tmp = (destination_t*)das_traverse(node_data->gg_list))
		!= NULL)
	    {
		//if neighbor isn't already visited
		if(!check_in_visited(visited, tmp))
		{
		    new->this = tmp->id;
		    new->prev = tmp1->this;
		    if(check_in_geocast(dest, tmp))
		    {
			*last_found = *new;
			entity_data->num_reachable++;
		    }
		    if(entity_data->num_reachable == num_targets)
			break;
		    tmp2 = new;
		    das_insert(visited, (void*)new);
		    new = NEW(visited_node_t);
		    *new = *tmp2;
		    das_insert(to_check, (void*)new);
		    new = NEW(visited_node_t);
		    tmp2 = NULL;
		}
	    }
	    if(entity_data->num_reachable == num_targets)
		break;
	    tmp2 = tmp1;
	}
	if(entity_data->num_reachable == num_targets)
	    break;
    }

    //create path structure
    if(last_found->this == NONE)
	return NULL;
    void* path = das_create();
    nodeid_t* next = NEW(nodeid_t);
    *next = last_found->this;
    das_insert(path, (void*)next);
    while((last_found = get_node(visited, last_found->prev)) != NULL)
    {
	if(last_found->this != call->node)
	{
	    next = NEW(nodeid_t);
	    *next = last_found->this;
	    das_insert(path, (void*)next);
	}
    }

    //deallocate used memory
    while((new = (visited_node_t*)das_pop(to_check)) != NULL)
	free(new);
    das_destroy(to_check);
    while((new = (visited_node_t*)das_pop(visited)) != NULL)
	free(new);
    das_destroy(visited);
    to_check = NULL;
    visited = NULL;

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

visited_node_t* get_node(void *visited, nodeid_t to_get)
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

////////////////////////////////////////////////////////////////////////////////
// tx
// code executed when message is sent, mostly useful for debugging

void tx(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

    //if dijkstra packet
    if(header->type == D_PACKET)
    {
	if(das_getsize(header->path) == 0)
	{
	    entity_data_t *entity_data = ENTITY_DATA(call);

	    entity_data->dijk_latency = get_time() - entity_data->dijk_latency;
	    packet_dealloc(packet);
	    return;
	}

	header->next_node.id = *((nodeid_t*)das_pop(header->path)),
	header->next_node.position = *get_node_position(header->next_node.id);
	header->sender = my_pos;
	if(set_mac_header_tx(call, packet) == ERROR)
	    fprintf(stderr, "[ERR] Can't route packet at %d\n", call->node);
	return;
    }
#ifdef LOG_ROUTING
    
    fprintf(stderr, "[RTG] Flooding at %d.\n", my_pos.id);
#endif
    flood(call, packet);

    return;
}


////////////////////////////////////////////////////////////////////////////////
// Routing Functions

//basic packet forwarding
void forward(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

    intersection_e result;
    destination_t tmp1 = next_on_face(call, &my_pos, &header->sender,
	TRAVERSE_R), tmp2 = next_on_face(call, &my_pos, &header->sender,
	TRAVERSE_L);
    //if next edge segment has first endpoint colliner with source destination
    //line
    if((result = check_intersect(call, &header->src, &header->dest, &tmp1))
	== COLLINEAR || check_in_geocast(&header->dest, &my_pos))
    {
	spray_packets(call, packet);
	return;
    }
    //else, if only one edge to continue along and a both packet needs to just
    //continue routing without splitting
    else if(header->direction == BOTH && compare_destinations(&tmp1, &tmp2))
    {
	header->next_node = tmp1;
	header->sender =  my_pos;
	if(set_mac_header_tx(call, packet) == ERROR)
	    fprintf(stderr, "[ERR] Can't route packet at %d\n", call->node);
	return;
    }
    //else just split packet and traverse in appropriate directions
    else if(header->direction == BOTH)
    {
	packet_t *packet_2 = copy_packet(call, packet, header, INCREMENT);
	forward_packet(call, packet, TRAVERSE_L);
	forward_packet(call, packet_2, TRAVERSE_R);
	return;
    }
    //else standard packet forwarding
    forward_packet(call, packet, header->direction);
    return;
}

// returns next node for face routing
destination_t next_on_face(call_t *call, destination_t *start,
    destination_t *nodeRef, direction_e direction)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *gg_nbr = NULL;
    double angle, angle_min = 180.0, angle_max = 0.0, vect, d1, d2;
    destination_t winner = *start, max = *start, no_dest = NO_DESTINATION;

    if(das_getsize(node_data->gg_list) == 0)
	return no_dest;

    das_init_traverse(node_data->gg_list);
    while((gg_nbr = (destination_t*)das_traverse(node_data->gg_list)) != NULL)
    {
	bool result = false;
	if(gg_nbr->id != nodeRef->id)
	{
	    //calculate angle between start-ref vector and start-next vector
	    //using dot product
	    position_t *temp_pos = &gg_nbr->position;
	    d1 = distance(&start->position, &nodeRef->position);
	    d2 = distance(&start->position, temp_pos);
	    angle = acos((double)(((start->position.x - nodeRef->position.x) *
		(start->position.x - temp_pos->x) + (start->position.y -
		nodeRef->position.y) * (start->position.y - temp_pos->y))
		/ (d1 * d2)));

	    //calculate 3-dimensional cross product of start-ref and
	    //start-next, then use the property of right-hand orthogonality
	    //to determine if angle is calculated clockwise or
	    //counter-clockwise
	    vect = (double)((temp_pos->x - start->position.x) *
		(nodeRef->position.y - start->position.y)) -
		(double)((temp_pos->y - start->position.y) *
		(nodeRef->position.x - start->position.x));
	    if((direction == TRAVERSE_R && vect >= 0) || (direction ==
		TRAVERSE_L && vect <= 0))
		result = true;

	    //find the smallest angle going counter-clockwise
	    if(result && angle <= angle_min)
	    {
		angle_min = angle;
		winner = *gg_nbr;
	    }
	    //find the largest angle going clockwise
	    else if(!result && angle >= angle_max)
	    {
		angle_max = angle;
		max = *gg_nbr;
	    }
	}
    }
    //extreme cases
    if(compare_destinations(&winner, start) == true)
	winner = max;
    if(compare_destinations(&winner, start) == true)
	winner = *nodeRef;

    return winner;
}

//check to see if edge traversed is juncture and to start routing next face
intersection_e check_intersect(call_t *call, destination_t *source,
    destination_t *dest, destination_t *next)
{
    if(check_in_geocast(dest, next))
	return INTERSECTION;
    if(!check_in_geocast(dest, source))
    {
	destination_t corner = *dest;
	corner.position.x -= 1;
	corner.position.y -= 1;
	if(check_intersect(call, &corner, dest, next) != NO_INT)
	    return INTERSECTION;
	corner = *dest;
	corner.position.x += 1;
	corner.position.y -= 1;
	if(check_intersect(call, &corner, dest, next) != NO_INT)
	    return INTERSECTION;
	corner = *dest;
	corner.position.x -= 1;
	corner.position.y += 1;
	if(check_intersect(call, &corner, dest, next) != NO_INT)
	    return INTERSECTION;
	corner = *dest;
	corner.position.x += 1;
	corner.position.y += 1;
	if(check_intersect(call, &corner, dest, next) != NO_INT)
	    return INTERSECTION;
    }

    //ignore juncture if either endpoint of previous-current line segment
    //is the source or destination node
    if(next->id == source->id || call->node == source->id)
	return NO_INT;
    position_t *current_pos = get_node_position(call->node);
    //calculate 3-dimensional cross products of source-current vector and
    //source-destination vector and the source-prev vector and
    //source-destination vector
    double vect1 = ((current_pos->x - source->position.x) * (dest->position.y
	- source->position.y)) - ((current_pos->y - source->position.y) *
	(dest->position.x - source->position.x));

    double vect2 = ((next->position.x - source->position.x) * (dest->position.y
	- source->position.y)) - ((next->position.y - source->position.y) *
	(dest->position.x - source->position.x));

    //if BOTH vectors have the same sign, they are BOTH on the same side of the
    //source-destination line, don't change face
    if((vect1 < 0 && vect2 < 0) || (vect1 > 0 && vect2 >0))
	return NO_INT;

    //check to see if current-next and source-destination intersect
    double a1, a2, b1, b2, c1, c2, det, x, y, dist;

    //get formulas for the two lines the line segments are part of in
    //Ax + By = C form
    a1 = dest->position.y - source->position.y;
    b1 = source->position.x - dest->position.x;
    c1 = a1 * source->position.x + b1 * source->position.y;
    a2 = next->position.y - current_pos->y;
    b2 = current_pos->x - next->position.x;
    c2 = a2 * current_pos->x + b2 * current_pos->y;

    //if determinant of the two formulas is 0, lines are parallel, check for
    //overlap of the x coordinates unless source-destination is a vertical
    //line, then check y coordinates
    if((det = a1 * b2 - a2 * b1) == 0)
    {
	//if(source-destination isn't vertical && min(x)(source-destination) <
	//max(x)(prev-current) && max(x)(source-destination) >
	//min(x)(prev-current) || source-destination is vertical &&
	//min(y)(source-destination) < max(y)(prev-current) &&
	//max(y)(source-destination) > min(y)(prev-current)
	if((source->position.x - dest->position.x != 0 &&
	    fmin(source->position.x, dest->position.x) <
	    fmax(current_pos->x, next->position.x) &&
	    fmax(source->position.x, dest->position.x) >
	    fmin(current_pos->x, next->position.x)) ||
	    (source->position.x - dest->position.x == 0 &&
	    fmin(source->position.y, dest->position.y) <
	    fmax(current_pos->y, next->position.y) &&
	    fmax(source->position.y, dest->position.y) >
	    fmin(current_pos->y, next->position.y)))
	{
	    return COLLINEAR;
	}
    }
    //if lines are not parallel
    else
    {
	//apply Cramer's rule to solve for point of intersection
	x = (b2 * c1 - b1 * c2)/det;
	y = (a1 * c2 - a2 * c1)/det;

	//if the length of the vectors formed by the difference of the
	//destination and point of intersection and the source and point of
	//intersection equal the length of the source-destination line segment,
	//then the lines intersect 
	dist = hypot((dest->position.x - x), (dest->position.y - y)) +
	    hypot((source->position.x - x), (source->position.y - y));
	if(((int)(distance(&source->position, &dest->position) * PRECISION)) ==
	    (int)(dist * PRECISION))
	{
	    if(current_pos->x == x && current_pos->y == y)
		return COLLINEAR;
	    if(next->position.x == x && next->position.y == y)
		return NO_INT;
	    return INTERSECTION;
	}
    }
    return NO_INT;
}

bool check_in_geocast(destination_t *region, destination_t *to_check)
{
    //TODO:HARD CODED CONSTANTS
    if((int)(to_check->position.x * PRECISION) >=
	(int)((region->position.x - REGION_SIZE / 2) * PRECISION) &&
	(int)(to_check->position.x * PRECISION) <=
	(int)((region->position.x + REGION_SIZE / 2) * PRECISION) &&
	(int)(to_check->position.y * PRECISION) >=
	(int)((region->position.y - REGION_SIZE / 2) * PRECISION) &&
	(int)(to_check->position.y * PRECISION) <=
	(int)((region->position.y + REGION_SIZE / 2) * PRECISION))
	return true;
    return false;
}

//start routing on the first face
void traverse_first_face(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call),
	no_dest = NO_DESTINATION;

    //set up first packet
    header->direction = TRAVERSE_R;
    header->sender = my_pos;
    //if already in geocats region, spray packets
    if(check_in_geocast(&header->dest, &my_pos))
    {
#ifdef LOG_ROUTING
	PRINT_ROUTING("[RTG] Start routing, first node in geocast region\n");
#endif
	spray_packets(call, packet);
    	return;
    }

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] start routing from %d to %f, %f, %f\n", header->src.id,
	header->dest.position.x, header->dest.position.y,
	header->dest.position.z);
#endif

    header->next_node = next_on_face(call, &my_pos, &header->dest,
	header->direction);
    //get next hop using source-destination line for reference
    if(compare_destinations(&header->next_node, &no_dest))
    {
#ifdef LOG_ROUTING
	fprintf(stderr, "[RTG] Can't route, no neighbors\n");
#endif
	packet_dealloc(packet);
	packet = NULL;
	return;
    }

    //if neighbor are in geocast region, send correct starting packets
    if(check_intersect(call, &my_pos, &header->dest, &header->next_node)
	== INTERSECTION)
    {
	header->sender = header->dest;
	forward_packet(call, packet, header->direction);
	
    }
    else
    {
	if(set_mac_header_tx(call, packet) == ERROR)
	{
	    fprintf(stderr, "[ERR] Can't route packet\n");
	    return;
	}
    }
	
    destination_t tmp1 = next_on_face(call, &my_pos, &header->dest, TRAVERSE_L);
    if(combine_packets(call, &my_pos, &tmp1, TRAVERSE_L))
    {
	destination_t tmp2 = next_on_face(call, &my_pos, &tmp1, TRAVERSE_R);
	if(!compare_destinations(&tmp2, &tmp1))
	{
	    packet = copy_packet(call, packet, header, INCREMENT);
	    header = PACKET_HEADER(packet, node_data);
	    header->direction = TRAVERSE_L;
	    header->next_node = tmp2;
	    if(!combine_packets(call, &my_pos, &tmp1, TRAVERSE_L))
	    {
		if(set_mac_header_tx(call, packet) == ERROR)
		    fprintf(stderr, "[ERR] Can't route packet\n");
	    }

	    tmp2 = next_on_face(call, &my_pos, &tmp1, TRAVERSE_L);
	    if(combine_packets(call, &my_pos, &tmp2, TRAVERSE_R))
		return;
	    packet = copy_packet(call, packet, header, INCREMENT);
	    header = PACKET_HEADER(packet, node_data);
	    header->direction = TRAVERSE_R;
	    header->next_node = tmp2;
	    if(set_mac_header_tx(call, packet) == ERROR)
		fprintf(stderr, "[ERR] Can;t route packet\n");
	}
	return;
    }    
    //set up second packet
    packet = copy_packet(call, packet, header, INCREMENT);
    header = PACKET_HEADER(packet, node_data);
    header->direction = TRAVERSE_L;

    //get next hop using source destination line for reference
    header->next_node = tmp1;
    if(check_intersect(call, &my_pos, &header->dest, &header->next_node)
	== INTERSECTION)
    {
	header->sender = header->dest;
	forward_packet(call, packet, header->direction);
    }
    else
    {
	if(set_mac_header_tx(call, packet) == ERROR)
	    fprintf(stderr, "[ERR] Can't route packet\n");
    }
    return;
}

//basic stateful flooding (broadcast)
void flood(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call), previous = header->sender;
    bool sent = false;

    //check if a flooding message has already been received at this node; if so, break
    if(node_data->broadcast_received){
#ifdef LOG_ROUTING
	fprintf(stderr, "[RTG] %d has already participated in the broadcast. Ignoring.\n", my_pos.id);
#endif
	return;
    }

#ifdef LOG_ROUTING
    fprintf(stderr, "[RTG] Splitting packets at <%d> with previous <%d>\n", my_pos.id, previous.id);
#endif
    //send a packet to every other neighbor
    destination_t *gg_nbr = NULL;
    int i, size = das_getsize(node_data->gg_list);                    
    das_init_traverse(node_data->gg_list);                
    for(i = 0; i < size; ++i)
    {             
        gg_nbr = (destination_t*)das_traverse(node_data->gg_list);
#ifdef LOG_ROUTING
        fprintf(stderr, "[RTG]   Considering destination node <%d>\n", gg_nbr->id);
        fprintf(stderr, "[RTG]     Sending!!!\n");              
#endif
        if(sent){             
            packet = copy_packet(call, packet, header, INCREMENT);
            header = PACKET_HEADER(packet, node_data);
        }
        header->sender = my_pos;
        header->next_node = *gg_nbr;
        header->direction = NONE;
        if(set_mac_header_tx(call, packet) == ERROR)
            fprintf(stderr, "[ERR] Can't route packet\n");

        sent = true;        
    }

    if(!sent){
        packet_dealloc(packet);
        packet = NULL;
        header = NULL;
    }

    node_data->broadcast_received = true;

    return;
}


//spray packets when call node is COLLINEAR with source-destination line
void spray_packets(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call), reference = header->sender;
    bool sent = false;

    //if packet to call->node was single directional, traverse previous edge
    //backwards
    if(compare_destinations(&reference, &my_pos))
    {
	reference = my_pos;
    }
    else if(header->direction != BOTH && !combine_packets(call,
	&header->next_node, &header->sender, header->direction))
    {
	destination_t tmp = header->sender;
	header->sender = header->next_node;
	header->next_node = tmp;
	if(set_mac_header_tx(call, packet) == ERROR)
	    fprintf(stderr, "[ERR] Can't route packet\n");
	sent = true;
    }

    //send a BOTH direction packet to every other neighbor
    destination_t *gg_nbr = NULL;
    int i, size = das_getsize(node_data->gg_list);
    das_init_traverse(node_data->gg_list);
    for(i = 0; i < size; ++i)
    {
	gg_nbr = (destination_t*)das_traverse(node_data->gg_list);
	if(!compare_destinations(gg_nbr, &reference))
	{
	    if(combine_packets(call, &my_pos, gg_nbr, BOTH))
		continue;
	    if(sent)
	    {
		packet = copy_packet(call, packet, header, INCREMENT);
		header = PACKET_HEADER(packet, node_data);
	    }
	    header->sender = my_pos;
	    header->next_node = *gg_nbr;
	    header->direction = BOTH;
	    if(set_mac_header_tx(call, packet) == ERROR)
		fprintf(stderr, "[ERR] Can't route packet\n");
	    sent = true;
	}
    }
    if(!sent)
    {
	packet_dealloc(packet);
	packet = NULL;
	header = NULL;
    }
    return;
}

//basic packet forwarding
void forward_packet(call_t *call, packet_t *packet, direction_e direction)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call), o_sender = header->sender,
	tmp, reference;
    direction_e o_direction = header->direction;
    intersection_e result;
    bool sent = false;

    header->direction = direction;
    reference = header->next_node = next_on_face(call, &my_pos, &header->sender,
	header->direction);
    header->sender = my_pos;
    result = check_intersect(call, &header->src, &header->dest,
	&header->next_node);
    if(!compare_destinations(&tmp, &o_sender) || (compare_destinations(&tmp,
	&o_sender) && o_direction != BOTH && direction == o_direction))
    {
	if(!combine_packets(call, &header->sender, &header->next_node,
	    header->direction))
	{
	    if(result == INTERSECTION)
		header->direction = BOTH;
	    if(set_mac_header_tx(call, packet) == ERROR)
		fprintf(stderr, "[ERR] Can't route packet\n");
	    sent = true;
	}
    }
    if(result == NO_INT)
    {
	if(!sent)
	{
	    packet_dealloc(packet);
	    packet = NULL;
	    header = NULL;
	}
	return;
    }
    tmp = next_on_face(call, &my_pos, &header->next_node, direction);

    //if edge is a juncture
    while(check_intersect(call, &header->src, &header->dest, &tmp) ==
	INTERSECTION && !compare_destinations(&tmp, &o_sender)
	&& !compare_destinations(&tmp, &reference))
    {
	if(!combine_packets(call, &header->sender, &tmp, header->direction))
	{
	    if(sent)
	    {
		packet = copy_packet(call, packet, header, INCREMENT);
		header = PACKET_HEADER(packet, node_data);
	    }
	    header->next_node = tmp;
	    header->direction = BOTH;
	    if(set_mac_header_tx(call, packet) == ERROR)
		fprintf(stderr, "[ERR] Can't route packet\n");
	    sent = true;
	}
	header->next_node = tmp;
	tmp = next_on_face(call, &my_pos, &header->next_node, direction);
    }
    if(!compare_destinations(&tmp, &o_sender) || (compare_destinations(&tmp,
	&o_sender) && o_direction != BOTH && direction == o_direction))
    {
	if(!combine_packets(call, &header->sender, &tmp, direction))
	{
	    if(sent)
	    {
		packet = copy_packet(call, packet, header, INCREMENT);
		header = PACKET_HEADER(packet, node_data);
		header->direction = direction;
	    }
	    header->next_node = tmp;
	    if(set_mac_header_tx(call, packet) == ERROR)
		fprintf(stderr, "[ERR] Can't route packet\n");
	    sent = true;
        }
    }
    if(!sent)
    {
	packet_dealloc(packet);
	packet = NULL;
	header = NULL;
    }
    return;
}

//last mile packet delivery
void deliver_packet(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

    header->next_node = header->dest;
    header->sender = my_pos;
    header->type = LM_PACKET;
    header->direction = NONE;

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] Last Mile routing to destination %d from %d\n",
	header->dest.id, call->node);
#endif
    if(set_mac_header_tx(call, packet) == ERROR)
	fprintf(stderr, "[ERR] Can't route packet\n");
    return;
}

//attempts to set mac header, if successful, sends packet to mac module for
//transmission
//returns true (1) on success
//returns ERR0R (-1) on failure
int set_mac_header_tx(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    call_t call_down = CALL_DOWN(call);
    destination_t no_dest = NO_DESTINATION;

    if(compare_destinations(&header->sender, &no_dest))
    {
	fprintf(stderr, "[ERR] Invalid sender\n");
	packet_dealloc(packet);
	packet = NULL;
	return ERROR;
    }
    if(compare_destinations(&header->next_node, &no_dest))
    {
	fprintf(stderr, "[ERR] Invalid next hop\n");
	packet_dealloc(packet);
	packet = NULL;
	return ERROR;
    }

    if(SET_HEADER(&call_down, packet, &header->next_node) == ERROR)
    {
	fprintf(stderr, "[ERR] Unable to set MAC header\n");
	packet_dealloc(packet);
	packet = NULL;
	return ERROR;
    }
    TX(&call_down, packet);
#ifdef LOG_ROUTING
    if(header->type == D_PACKET)
	PRINT_ROUTING("[RTG] optimal path packet at %d sent to %d\n",
	    call->node, header->next_node.id);
    else if(header->direction == TRAVERSE_R)
	PRINT_ROUTING("[RTG] packet at %d, sweep right to %d\n", call->node,
	    header->next_node.id);
    else if(header->direction == TRAVERSE_L)
	PRINT_ROUTING("[RTG] packet at %d, sweep left to %d\n", call->node,
	    header->next_node.id);
    else if(header->direction == BOTH)
	PRINT_ROUTING("[RTG] packet at %d, sweep both directions to %d\n",
	    call->node, header->next_node.id);
#endif
    return true;
}

//checks to see if packets can be combined to reduce number of packets
//returns true if packets are combined, false otherwise
bool combine_packets(call_t *call, destination_t *sender,
    destination_t *next_node, direction_e direction)
{
    node_data_t *node_data = NODE_DATA(call);

    bool combined = false;
    call_t call_down = CALL_DOWN(call);
    void *buffer = GET_BUFFER(&call_down);
    void* new_buffer = das_create();
    buffer_entry_t *entry = NULL;

    while((entry = (buffer_entry_t*)das_pop_FIFO(buffer)) != NULL)
    {
	header_t *header = PACKET_HEADER(entry->packet, node_data);
	if(header->type == DATA_PACKET &&
	    compare_destinations(sender, &header->sender) &&
	    compare_destinations(next_node, &header->next_node))
	{
	    if(header->direction == TRAVERSE_R || header->direction ==
		TRAVERSE_L)
		header->direction = BOTH;
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] packets combined, packet at %d sweeping both "
		"directions to %d\n", call->node, header->next_node.id);
#endif
	    combined = true;
	}
	das_insert(new_buffer, (void*)entry);
    }
    SET_BUFFER(&call_down, new_buffer);
    return combined;
}

//clones packet, sets all information, and tracks num of packets created
packet_t* copy_packet(call_t *call, packet_t *packet, header_t* header,
    operation_e operation)
{
    node_data_t *node_data = NODE_DATA(call);
    entity_data_t *entity_data = ENTITY_DATA(call);
    packet_t *copy_packet = packet_clone(packet);
    header_t *copy_header = PACKET_HEADER(copy_packet, node_data);

    copy_header->dest = header->dest;
    copy_header->src = header->src;
    copy_header->sender = header->sender;
    copy_header->next_node = header->next_node;
    copy_header->type = header->type;
    copy_header->direction = header->direction;
    copy_header->ttl = header->ttl;
    copy_header->path = NULL;

    if(operation != DONT_INCREMENT)
	entity_data->num_packets++;
    return copy_packet;
}

////////////////////////////////////////////////////////////////////////////////
// RX
//function called when the application receives a packet

void rx(call_t *call, packet_t *packet)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    array_t *up = get_entity_bindings_up(call);
    destination_t me = THIS_DESTINATION(call);
    mate_e result;
    int i = up->size;

    switch(header->type)
    {
	case D_PACKET:
	    tx(call, packet);
	    break;
	case HELLO_PACKET:
	    //planarize_graph(call, packet);
	    packet_dealloc(packet);
	    packet = NULL;
	    header = NULL;
	    break;
	case DATA_PACKET:
	    if(packet_lost(call))
	    {
#ifdef LOG_ROUTING
		fprintf(stderr, "[RTG] packet from %d to %d lost\n",
		    header->sender.id, call->node);
#endif
		packet_dealloc(packet);
		packet = NULL;
		break;
	    }
	    entity_data->total_num_hops++;
	    header->ttl--;
	    if(header->ttl != 0)
	    {
	        tx(call, packet);

		//if calling node is destination we need to keep routing packet
		//but also deliver packet, so fall through to deliver case
		if(!check_in_geocast(&header->dest, &me))
		    break;
	    }
	    else
	    {
#ifdef LOG_ROUTING
		fprintf(stderr, "[RTG] packet from %d to %d hit ttl\n",
		    header->sender.id, call->node);
#endif
		if(header->dest.id != call->node)
		{
		    packet_dealloc(packet);
		    packet = NULL;
		    break;
		}
	    }
	    if(call->node == header->src.id)
		break;
	case LM_PACKET:
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] delivering packet at %d\n", call->node);
#endif
	    if(header->type == LM_PACKET)
		entity_data->total_num_hops++;
	    if(!node_data->received_packet)
	    {
		entity_data->deliver_num_hops = entity_data->total_num_hops;
		node_data->received_packet = true;
	    }
	    while(i--)
	    {
		call_t call_up = {up->elts[i], call->node, call->entity};
		packet_t *packet_up;
		if(i > 0)
		    packet_up = copy_packet(call, packet, header,
			DONT_INCREMENT);
		else if(header->type == LM_PACKET)
		    packet_up = packet;
		else
		    packet_up = copy_packet(call, packet, header,
			DONT_INCREMENT);
		RX(&call_up, packet_up);
	    }
	    break;
	default:
	    break;
    }
    return;
}

bool packet_lost(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);

    if(entity_data->loss_rate == 0)
	return false;
    if((get_random_double() * 100) < entity_data->loss_rate)
	return true;
    return false;
}

mate_e find_mate(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] packet at %d from %d looking for mate\n", call->node,
	header->sender.id);
#endif

    //check to see if a both direction packet is its own mate, can occur on
    //nodes with only one neighbor on GG graph
    if(header->direction == BOTH)
    {
	destination_t tmp1 = next_on_face(call, &my_pos, &header->sender,
	    TRAVERSE_R), tmp2 = next_on_face(call, &my_pos, &header->sender,
	    TRAVERSE_L);
	if(compare_destinations(&header->sender, &tmp1) &&
	    compare_destinations(&header->sender, &tmp2))
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] found mates at %d, deallocating packets\n",
		call->node);
#endif
	    packet_dealloc(packet);
	    packet = NULL;
	    header = NULL;
	    return FOUND_THIS;
	}
    }

    buffer_entry_t* entry = NULL;
    call_t call_down = CALL_DOWN(call);
    void *buffer = GET_BUFFER(&call_down), *tmp = das_create();
    mate_e found = NOT_FOUND;

    das_init_traverse(buffer);
    while((entry = (buffer_entry_t*)das_pop_FIFO(buffer)) != NULL)
    {
	header_t *comp_header = PACKET_HEADER(entry->packet, node_data);
	if(comp_header->type == DATA_PACKET &&
	    compare_destinations(&comp_header->next_node,  &header->sender) &&
	    compare_destinations(&comp_header->sender, &header->next_node) &&
	    (comp_header->direction != header->direction ||
	    (comp_header->direction == BOTH && header->direction == BOTH)))
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] found mates at %d, deallocating packets\n",
		call->node);
#endif
	    //if mate in buffer found is both packet
	    if(comp_header->direction == BOTH && header->direction != BOTH)
	    {
		comp_header->direction = header->direction;
		found = FOUND_OTHER_1;
	    }
	    //if mate found in buffer isn't both packet, but received packet is
	    //both packet
	    else if(header->direction == BOTH && comp_header->direction != BOTH)
	    {
		header->direction = comp_header->direction;
		scheduler_delete_callback(call, entry->event);
		packet_dealloc(entry->packet);
		entry->packet = NULL;
		comp_header = NULL;
		entry = NULL;
		found = FOUND_OTHER_2;
		continue;
	    }
	    //else if both mates are single direction packets
	    else
	    {
		scheduler_delete_callback(call, entry->event);
		packet_dealloc(entry->packet);
		entry->packet = NULL;
		comp_header = NULL;
		entry = NULL;
		found = FOUND_OTHER_1;
		continue;
	    }
	}
	das_insert(tmp, (void*)entry);
    }
    //if received packet is no longer needed
    if(found == FOUND_OTHER_1)
    {
	packet_dealloc(packet);
	packet = NULL;
	header = NULL;
    }
    SET_BUFFER(&call_down, tmp);
    return found;
}

////////////////////////////////////////////////////////////////////////////////
// General Helper Functions

//cpmpares two positions
bool compare_positions(position_t *l, position_t *r)
{
    if(l->x == r->x && l->y == r->y && l->z == r->z)
	return true;
    return false;
}

//compares two destinations
bool compare_destinations(destination_t *l, destination_t *r)
{
    if(l->id == r->id && compare_positions(&l->position, &r->position))
	return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Postscript functions

void gg_post_axes(call_t *call)
{
    FILE *graph_gg;
    position_t *topo_pos = get_topology_area();
    entity_data_t *entity_data = ENTITY_DATA(call);

    char *name = (char*)malloc(25*sizeof(int));
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
	fprintf(graph_gg, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0"	    " 0 0 setrgbcolor\n", pos->x * entity_data->scale_postscript + 15.0,
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
// data structure describing exported functions

routing_methods_t methods = 
{
rx,
tx,
set_header,
get_header_size,
get_header_real_size,
};
