// mflood.c
// Stateful Flooding for Multicasting Protocol Module with Dijkstra
// plus error rate and offline planarization
// Jordan Adamek
// 12/11/2016

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "../linked_list/linked_list.c"

#include <include/modelutils.h>

//#define LOG_ROUTING
//#define DBG_STEINER
////////////////////////////////////////////////////////////////////////////////
// Model Info

model_t model = 
{
    "M-Flood with BFS, CDS, and error rate",
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
#define DEFAULT_TTL 500

//how many decimal places are used in calculations
#define PRECISION 100000

//radio range in meters
#define RADIO_RANGE 100

//region length (or diameter) in meteres redefined here
#define REGION_SIZE 200

//value of Pi for steiner geometry
#define PI 3.14159265

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

typedef enum {DATA_PACKET, DIJK_PACKET} packet_e;
typedef enum {NO_DIR, TRAVERSE_R, TRAVERSE_L, BOTH} direction_e;
typedef enum {NO_INT, INTERSECTION, COLLINEAR} intersection_e;
typedef enum {NOT_FOUND, FOUND_THIS, FOUND_OTHER_1, FOUND_OTHER_2} mate_e;
typedef enum {DONT_INCREMENT, INCREMENT} operation_e;
typedef enum {GREEDY, FACE} mode_e;

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

typedef struct     
{
    nodeid_t target;
    uint64_t start;
    uint64_t latency;
    void *path;    
}path_t;

//Routing tree of multicast targets
typedef struct tnode
{
    destination_t location;
    struct tnode** children;
    int child_count;
    bool isVirtual;
    bool isActive;
} tree_node;

//Tuple (root, steiner, left, right) for tree construction
typedef struct
{
    tree_node* s;
    tree_node* t;
    tree_node* u;
    tree_node* v;
    double reduction_ratio;
    bool active;
} steiner_tuple;

typedef struct
{
    tree_node* pivot;
    destination_t* target;
} spawn_request;

//packet information needed for routing
typedef struct
{
    destination_t dest;
    destination_t src;
    destination_t sender;
    destination_t next_node;
    packet_e type;
    mode_e mode;
    direction_e direction;
    int ttl;
    void *path;
    int target_count;
    destination_t** target_list;
    int destination_count;
    destination_t** destination_list;
    tree_node* root;
    bool face_mode;
    destination_t gmp_check_point;
    destination_t gfg_check_point;
}header_t;

//Global node data, used for tracking information known to node
typedef struct
{
    void *nbrs;
    void *gg_list;
    int overhead;
    bool received_packet;
}node_data_t;

//Global entity data, used for tracking statistics
typedef struct
{
    int num_reachable;
    int total_num_hops;
    int deliver_num_hops;
    int num_packets;
    float scale_postscript;
    uint64_t dijk_latency;
    nodeid_t last_reached;
    void* paths;
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
int start_dijk(call_t *call, destination_t *dest, destination_t** target_list, int target_count, tree_node* root);
int dijk_start(call_t* call, void* dests);
void* get_shortest_path(call_t *call, void *dests);
void* make_path(call_t *call, void* dests, visited_node_t*);
bool check_in_visited(void *visited, destination_t *to_check);
visited_node_t* get_node(void *visited, nodeid_t to_get);
bool check_node_in(void*, nodeid_t);

//routing functions - gmp
void forward(call_t *call, packet_t *packet);
bool check_in_target_list(destination_t** target_list, int target_count, destination_t* to_check);

//tx
void tx(call_t *call, packet_t *packet);

//receiving functions
void rx(call_t *call, packet_t *packet);

//general helper functions
double compute_distance(position_t a, position_t b);
double absolute_value(double n);
void* das_select(void *das, int index);
bool compare_positions(position_t *l, position_t *r);
bool compare_destinations(destination_t *l, destination_t *r);
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
    entity_data->num_reachable = 0;
    entity_data->dijk_latency = 0;
    entity_data->last_reached = NONE;

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

    if(entity_data->last_reached != NONE)
    {
        path_t *to_check = NULL;
        das_init_traverse(entity_data->paths);
        while((to_check = (path_t*)das_traverse(entity_data->paths)) != NULL)
        {
            if(to_check->target == entity_data->last_reached)
            {
                entity_data->dijk_latency = to_check->latency;
                break;
            }
        }
    }


    fprintf(stderr, "Routing Statistics:\n");
    fprintf(stderr, "  number of packets: %d\n", entity_data->num_packets);
    fprintf(stderr, "  number of hops to deliver: %d\n",
	entity_data->deliver_num_hops);
    fprintf(stderr, "  total number of hops: %d\n",
	entity_data->total_num_hops);
    fprintf(stderr, "  BFS Latency: %lld nanoseconds\n",
	entity_data->dijk_latency);
    fprintf(stderr, "  Num reachable targets: %d",
	entity_data->num_reachable);
    if(entity_data->num_reachable < 1)
        fprintf(stderr, " <failed>");
    fprintf(stderr, "\n");

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
        fprintf(results, "%lld\t", entity_data->dijk_latency);
        fprintf(results, "%d\n", entity_data->num_reachable);
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
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    call_t call_down = CALL_DOWN(call);
    destination_t my_pos = THIS_DESTINATION(call), no_dest = NO_DESTINATION;

    header->src = my_pos;
    header->dest = *dest;
    header->sender = no_dest;
    header->next_node = no_dest;
    header->type = DATA_PACKET;
    header->mode = GREEDY;
    header->direction = NO_DIR;
    header->ttl = DEFAULT_TTL;
    header->path = NULL;
    header->target_count = dest->id;
    header->destination_count = dest->id;
    header->target_list = malloc(sizeof(destination_t*) * header->target_count);
    header->destination_list = malloc(sizeof(destination_t*) * header->destination_count);
    header->face_mode = false;
    header->gmp_check_point = my_pos;
    header->gfg_check_point = my_pos;

    // Generate target list
    int i = 0, di = 0;
    destination_t* tmp = 0;

    for(i = 0; i < header->target_count; i++){
        if(i == my_pos.id){
	    // Skip one ID if Source would be included as a target
	    di++;
	}

	tmp = NEW(destination_t);
	tmp->id = i + di;
	tmp->position = *get_node_position(i + di);
	header->target_list[i] = tmp;
	header->destination_list[i] = tmp;
    }

    header->root = build_steiner_tree(&header->src, header->target_list, header->target_count);
	
    //do_cds(call, dest);
    if(start_dijk(call, dest, header->target_list, header->target_count, header->root) == ERROR)
	fprintf(stderr, "[ERR] unable to start shortest path\n");

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
	if(i != call->node && compute_distance(*get_node_position(call->node),
	    *get_node_position(i)) <= RADIO_RANGE)
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
	double radius = compute_distance(center, my_pos.position);

	bool witness = false;
	das_init_traverse(node_data->nbrs);
	while((tmp2 = (destination_t*)das_traverse(node_data->nbrs)) != NULL)
	{
	    if(tmp2->id != tmp->id && compute_distance(tmp2->position, center)
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

int start_dijk(call_t *call, destination_t* dest, destination_t** target_list, int target_count, tree_node* root)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    entity_data->dijk_latency = get_time();
    node_data_t *node_data = NODE_DATA(call);
    destination_t my_pos = THIS_DESTINATION(call), no_dest = NO_DESTINATION;
    packet_t *packet = NULL;
    int i;
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
    header->type = DIJK_PACKET;
    header->direction = NO_DIR;
    header->ttl = 0;
    header->target_list = target_list;
    header->target_count = target_count;
    header->root = root;

    void* dests = das_create();
    for(i = 0; i < target_count; i++){
        das_insert(dests, (void*)header->target_list[i]);
    }

    if((entity_data->paths = get_shortest_path(call, dests)) == NULL)
    {
        fprintf(stderr, "[ERR] No shortest path found\n");
        return ERROR; 
    }

    path_t *to_start = NULL;
    das_init_traverse(entity_data->paths);
    for(i = 0; i < das_getsize(entity_data->paths); i++)
    {
        packet_t *packet_2 = NULL;
        header_t *header_2 = NULL;

        to_start = (path_t*)das_traverse(entity_data->paths);
        if(i < das_getsize(entity_data->paths) - 1)
        {
            packet_2 = copy_packet(call, packet, header, DONT_INCREMENT);
            header_2 = PACKET_HEADER(packet_2, node_data);
            header_2->path = to_start->path;
            to_start->start = get_time() + i * PERIOD;
            scheduler_add_callback(get_time() + i * PERIOD, call, dijk_start,
                (void*)packet_2);
        }
        else
        {
            header->path = to_start->path;
            to_start->start = get_time() + i * PERIOD;
            scheduler_add_callback(get_time() + i * PERIOD, call, dijk_start,
                (void*)packet);
        }
    }
    return 0;
}

int dijk_start(call_t *call, void *args)
{
    tx(call, (packet_t*)args);
    return 0;
}

void* get_shortest_path(call_t *call, void* dests)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    call_t call_next = *call;
    destination_t *tmp = NULL;
    visited_node_t *tmp1 = NEW(visited_node_t), *tmp2 = NULL,
        *new = NEW(visited_node_t), *last_found = NEW(visited_node_t);
    path_t *new_path = NULL;
    void *visited = das_create(), *to_check = das_create(),
        *paths = das_create();
    int num_targets = 0;

    num_targets = das_getsize(dests);

    tmp1->this = call->node;
    tmp1->prev = NONE;
    *new = *tmp1;
    das_insert(to_check, (void*)tmp1);
    tmp1 = NEW(visited_node_t);
    *tmp1 = *new;
    das_insert(visited, (void*)tmp1);
    new->this = NONE;
    *last_found = *new;

    while(das_getsize(to_check) != 0)
    {
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
            das_init_traverse(node_data->gg_list);
            while((tmp = (destination_t*)das_traverse(node_data->gg_list))
                != NULL)
            {
                if(!check_in_visited(visited, tmp))
                {
                    new->this = tmp->id;
                    new->prev = tmp1->this;
                    if(check_node_in(dests, tmp->id))
                    {
                        *last_found = *new;
                        new_path = NEW(path_t);
                        new_path->target = last_found->this;
                        new_path->latency = 0;
			new_path->path = make_path(call, visited, last_found);
                        das_insert(paths, (void*)new_path);
                        new_path = NULL;
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

    while((new = (visited_node_t*)das_pop(to_check)) != NULL)
        free(new);
    das_destroy(to_check);           
    while((new = (visited_node_t*)das_pop(visited)) != NULL)
        free(new);                  
    das_destroy(visited);
    to_check = NULL;
    visited = NULL;         

    return paths;
}

void* make_path(call_t *call, void *visited, visited_node_t *end)
{
    void *path = NULL;                  
    if((path = das_create()) == NULL)         
    {
        fprintf(stderr, "[ERR] Can't create path to %d\n", end->this);    
        return NULL;    
    }        
    nodeid_t *next = NEW(nodeid_t);                
    *next = end->this;
    das_insert(path, (void*)next);      
    while((end = get_node(visited, end->prev)) != NULL)
    {
        if(end->this != call->node)
        {
            next = NEW(nodeid_t);              
            *next = end->this;
            das_insert(path, (void*)next);
            next = NULL;
        }
    }
das_init_traverse(path);
while((next = (nodeid_t*)das_traverse(path)) != NULL)
fprintf(stderr, "%d, ", *next);
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

bool check_node_in(void* das, nodeid_t to_check)
{
    destination_t *entry = NULL;    
    das_init_traverse(das);           
    while((entry = (destination_t*)das_traverse(das)) != NULL)
    {
        if(entry->id == to_check)                
            return true;
    }
    return false;             
}


////////////////////////////////////////////////////////////////////////////////
// tx
// code executed when message is sent, mostly useful for debugging

void tx(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);

    if(header->type == DIJK_PACKET)
    {
        if(das_getsize(header->path) == 0)   
        {
            entity_data_t *entity_data = ENTITY_DATA(call);
            path_t *tmp = NULL;

            das_init_traverse(entity_data->paths);
            while((tmp = (path_t*)das_traverse(entity_data->paths)) != NULL)
            {                 
                if(tmp->target == call->node)
                {
                    tmp->latency = get_time() - tmp->start;
                    break;
                }           
            }         
            packet_dealloc(packet);
            return;
        }

        destination_t my_pos = THIS_DESTINATION(call);
        header->next_node.id = *((nodeid_t*)das_pop(header->path));
        header->next_node.position = *get_node_position(header->next_node.id);
        header->sender = my_pos;
        if(set_mac_header_tx(call, packet) == ERROR)
            fprintf(stderr, "[ERR] Can't route dijkstra packet at %d\n",
                call->node);
        return;
    }


    if(header->target_count == 1){
	// If there is only a single target left, will be GFG routed directly to that target
	if(!compare_destinations(&header->dest, header->target_list[0])){
	    header->dest = *header->target_list[0];
	}

#ifdef LOG_ROUTING
        fprintf(stderr, "[RTG] GFG routing single target %d at %d, headed for %d\n", header->target_list[0]->id, call->node, header->dest.id);
#endif

	gfg_forward(call, packet);
    }
    else{
#ifdef LOG_ROUTING
        fprintf(stderr, "[RTG] GMP routing packet at %d\n", call->node);
#endif
        forward(call, packet);
    }

//getchar();
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Routing Functions - GMP

//basic packet forwarding
void forward(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

    return;
}

//
// return whether a given destination is an intended multicast target
bool check_in_target_list(destination_t** target_list, int target_count, destination_t* to_check)
{
    int i;
    for(i = 0; i < target_count; i++){
	if(compare_destinations(target_list[i], to_check)) return true;
    }
    return false;
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
    if(header->type == DIJK_PACKET)
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

//Checks if this packet qualifies as a flooding packet. For SF, this is clearly just always true.
//The real use of this method is to allow easy integration with other delivery algorithms.
bool flooding_packet(call_t *call, packet_t *packet){         
    return true;
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
    int i = up->size;

    if(header->type == DIJK_PACKET)
    {
        tx(call, packet);
        return;              
    }

    entity_data->total_num_hops++;
    header->ttl--;
    if(header->ttl != 0)
    {
	// forward
	tx(call, packet);

	if(!check_in_target_list(header->destination_list, header->destination_count, &me))
	    return;
    }
    else
    {
#ifdef LOG_ROUTING
	fprintf(stderr, "[RTG] packet from %d to %d hit ttl\n",
	    header->sender.id, call->node);
#endif
	if(!check_in_target_list(header->destination_list, header->destination_count, &me))
	{
	    packet_dealloc(packet);
	    packet = NULL;
	    return;
	}
    }

#ifdef LOG_ROUTING
    fprintf(stderr, "[RTG] delivering packet at %d\n", call->node);
#endif

    if(!node_data->received_packet)
    {
        entity_data->last_reached = call->node;
    	entity_data->deliver_num_hops = entity_data->total_num_hops;
	node_data->received_packet = true;	
    }
    while(i--)
    {
	call_t call_up = {up->elts[i], call->node, call->entity};
	packet_t *packet_up = copy_packet(call, packet, header, DONT_INCREMENT);
	RX(&call_up, packet_up);
    }
    return;
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

mate_e sf_find_mate(call_t *call, packet_t *packet)
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
	    compare_destinations(&comp_header->sender, &header->next_node))
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] found mates at %d, deallocating packets\n",
		call->node);
#endif
	    scheduler_delete_callback(call, entry->event);
	    packet_dealloc(entry->packet);
	    entry->packet = NULL;
	    comp_header = NULL;
	    entry = NULL;
	    found = FOUND_OTHER_1;
	    continue;
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

//get the absolute value of a double-type number
double absolute_value(double n){
    if(n < 0) n *= -1;
    return n;
}

//select an item in a das 'index' number of items from the initial position
void* das_select(void* das, int index){
    void* selection = NULL;
    int i = 0;

    das_init_traverse(das);
    while((selection = das_traverse(das)) != NULL){
        if(i++ == index) break;
    }

    return selection;
}

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
	fprintf(stderr, "[ERR] Couldn't open file %s\nError opening postscript file\n", name);
	return;
    }

    if(call->node == 0)
	fprintf(graph_gg, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0 0 0 setrgbcolor\n", pos->x * entity_data->scale_postscript + 15.0,
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
