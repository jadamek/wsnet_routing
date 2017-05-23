//James Robinson
//5/7/2017
//Module for minimum spanning tree

#include <unistd.h>
#include <stdbool.h>

#include "include/modelutils.h"

////////////////////////////////////////////////////////////////////////////////
// Model Info

model_t model =
{
	"Minimum Spanning Tree",
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
#define LOG_MST

#define ERROR -1
#define NONE -2

#define GG_RANGE 100
#define DEFAULT_PACKET_SIZE 10
#define DEFAULT_TTL 55
#define PRECISION 1000000

#define CALL_DOWN(call) {get_entity_bindings_down(call)->elts[0], call->node,\
	call->entity}
#define NODE_DATA(call) get_node_private_data(call)
#define ENTITY_DATA(call) get_entity_private_data(call)
#define PACKET_HEADER(packet, node_data) (header_t*)(packet->data +\
	node_data->overhead)
#define NEW(type) malloc(sizeof(type))

#define EMPTY_POSITION {-1, -1, -1}
#define NO_DESTINATION {NONE, EMPTY_POSITION}
#define THIS_DESTINATION(call) {call->node, *get_node_position(call->node)}
#define DEFAULT_MAC_ADDR {BROADCAST_ADDR, EMPTY_POSITION}

////////////////////////////////////////////////////////////////////////////////
// Typedefs and Structs

typedef enum {DATA_PACKET, DIJK_PACKET} packet_e;
typedef enum {NO_DIR, TRAVERSE_R, TRAVERSE_L, BOTH} direction_e;
typedef enum {NO_INT, INTERSECTION, COLLINEAR} intersection_e;
typedef enum {NOT_FOUND, FOUND_THIS, FOUND_OTHER_1, FOUND_OTHER_2} mate_e;
typedef enum {DONT_INCREMENT, INCREMENT} operation_e;

typedef struct
{
    destination_t this;
    destination_t parent;
    void *children;
}tree_node_t;

typedef struct
{
    destination_t *parent;
    destination_t *to_add;
    double distance;
}winner_t;

typedef struct
{
    nodeid_t this;
    nodeid_t prev;
}visited_node_t;

typedef struct
{
    destination_t source;
    destination_t sender;
    destination_t next_node;
    void *dests;
    tree_node_t *mst;
    direction_e direction;
    int ttl;
    packet_e type;
    void *path;
}header_t;

typedef struct
{
    void *nbrs;
    void *gg_list;
    int overhead;
    bool received_packet;
}node_data_t;

typedef struct
{
    int num_reachable;
    int total_num_hops;
    int deliver_num_hops;
    int num_packets;
    uint64_t dijk_latency;

    float scale_postscript;
    tree_node_t *mst;
}entity_data_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes

int init(call_t*, void*);
int destroy(call_t*);
int setnode(call_t*, void*);
int unsetnode(call_t*);
int bootstrap(call_t*);
int set_header(call_t*, packet_t*, destination_t*);
void get_dests(call_t*, void*, int);
bool check_node_in(void*, nodeid_t);
tree_node_t* get_mst(call_t*, void*);
winner_t get_shortest_dist(tree_node_t*, destination_t*);
tree_node_t* get_branch(tree_node_t*, destination_t*);
int get_header_size(call_t*);
int get_header_real_size(call_t*);
int start_dijk(call_t*, void*);
void* get_shortest_path(call_t*, void*);
bool check_in_visited(void*, destination_t*);
visited_node_t* get_node(void*, nodeid_t);
void planarize_graph(call_t*);
void tx(call_t*, packet_t*);
void forward(call_t*, packet_t*);
destination_t next_on_face(call_t*, destination_t*, destination_t*,
    direction_e);
intersection_e check_intersect(call_t*, destination_t*, header_t*,
    tree_node_t*);
void traverse_first_face(call_t*, packet_t*);
void spray_packets(call_t*, packet_t*);
void follow_tree(call_t*, packet_t*);
bool check_duplicates(call_t*, destination_t*, destination_t*, direction_e);
void forward_packet(call_t*, packet_t*, direction_e);
int set_mac_header_tx(call_t*, packet_t*);
bool combine_packets(call_t*, destination_t*, destination_t*,
    direction_e);
packet_t* copy_packet(call_t*, packet_t*, header_t*, operation_e);
void rx(call_t*, packet_t*);
mate_e find_mate(call_t*, packet_t*);
void delete_tree(tree_node_t*);
bool compare_position(position_t*, position_t*);
bool compare_destinations(destination_t*, destination_t*);
void gg_post_axes(call_t*);
void topo_post_axes(call_t*);
void mst_post_axes(call_t*);
void gg_post_nodes(call_t*);
void topo_post_nodes(call_t*);
void mst_post_nodes(call_t*, tree_node_t*);
void topo_post_data(call_t*);

////////////////////////////////////////////////////////////////////////////////
// Initialization

int init(call_t *call, void *params)
{
    entity_data_t *entity_data = NULL;
    if((entity_data = NEW(entity_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate entity data in routing module\n");
	return ERROR;
    }

    entity_data->total_num_hops = 0;
    entity_data->deliver_num_hops = 0;
    entity_data->num_packets = 0;
    entity_data->num_reachable = 0;
    entity_data->dijk_latency = 0;

#if defined LOG_TOPO_G || defined LOG_GG
    position_t *topo_pos = get_topology_area();
    entity_data->scale_postscript = 595.0 / (topo_pos->x + 20);
    //entity_data->scale_postscript = 1700.0 / (topo_pos->x + 20);
#endif
#ifdef LOG_TOPO_G
    if(access("topo_graph.ps", F_OK) != ERROR)
	remove("topo_graph.ps");
#endif
#ifdef LOG_GG
    if((access("gabriel_graph.ps", F_OK)) != ERROR)
	remove("gabriel_graph.ps");
#endif
#ifdef LOG_TOPO
    if(access("topo.data", F_OK) != ERROR)
	remove("topo.data");
#endif
#ifdef LOG_MST
    if(access("mst.ps", F_OK) != ERROR)
	remove("mst.ps");
#endif

    set_entity_private_data(call, entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);

    delete_tree(entity_data->mst);

    FILE *results;
    char* name= (char*)malloc(11 * sizeof(int));
    sprintf(name, "results.txt");

    fprintf(stderr, "Routing Statistics:\n");
    fprintf(stderr, "  number of packets: %d\n", entity_data->num_packets);
    fprintf(stderr, "  number of hops to deliver: %d\n",
	entity_data->deliver_num_hops);
    fprintf(stderr, "  total number of hops: %d\n",
	entity_data->total_num_hops);
    fprintf(stderr, "  BFS latency: %lld nanoseconds\n",
	entity_data->dijk_latency);
    fprintf(stderr, "  Num reachable targets: %d\n",
	entity_data->num_reachable);

    if((results = fopen(name, "a")) == NULL)
	fprintf(stderr, "[ERR] Couldn't open file %s\n", name);
    else
	fprintf(results, "%lld\t%d\t%d\n", entity_data->dijk_latency,
	    entity_data->num_reachable, entity_data->num_packets);

    free(entity_data);
    entity_data = NULL;
    if(results != NULL)
	fclose(results);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

int setnode(call_t *call, void *params)
{
    /*param_t *param*/
    node_data_t *node_data = NULL;
    if((node_data = NEW(node_data_t)) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate node data in routing module\n");
	return ERROR;
    }

    if((node_data->nbrs = das_create()) == NULL || (node_data->gg_list =
	das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate neighbor lists in routing"
	    " module\n");
	free(node_data);
	node_data = NULL;
	return ERROR;
    }

    node_data->received_packet = false;
    node_data->overhead = NONE;

    /*das_init_traverse(params);
    while((param = (param_t*)das_traverse(params)) != NULL)
    {
    }*/

    set_node_private_data(call, node_data);

#ifdef LOG_TOPO
    topo_post_data(call);
#endif

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Unset Node

int unsetnode(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *dest;

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
    planarize_graph(call);
#ifdef LOG_GG
    gg_post_axes(call);
    gg_post_nodes(call);
#endif
#ifdef LOG_TOPO_G
    topo_post_axes(call);
    topo_post_nodes(call);
#endif
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Header Functions

int set_header(call_t *call, packet_t *packet, destination_t *dest)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    call_t call_down = CALL_DOWN(call);
    destination_t none = NO_DESTINATION, my_pos = THIS_DESTINATION(call);

    header->source = my_pos;
    header->sender = none;
    header->next_node = none;
    header->direction = NO_DIR;
    header->ttl = DEFAULT_TTL;
    header->type = DATA_PACKET;
    header->path = NULL;
    header->dests = das_create();
    get_dests(call, header->dests, dest->id);
    if(das_getsize(header->dests) == 0)
    {
	fprintf(stderr, "[ERR] Error getting targets\n");
	return ERROR;
    }
    if((entity_data->mst = get_mst(call, header->dests)) == NULL)
    {
	fprintf(stderr, "[ERR] Error calculating Minimum Spanning Tree\n");
	return ERROR;
    }
    header->mst = entity_data->mst;
    if(start_dijk(call, header->dests) == ERROR)
	fprintf(stderr, "[ERR] unable to start shortest path\n");

#ifdef LOG_MST
    mst_post_nodes(call, entity_data->mst);
    mst_post_axes(call);
#endif
    entity_data->num_packets++;
    return SET_HEADER(&call_down, packet, dest);
}

void get_dests(call_t *call, void *dests, int num_targets)
{
    int i;
    for(i = 0; i < num_targets; ++i)
    {
	bool found = false;
	do
	{
	    nodeid_t target = get_random_node(NONE);
	    if(target != call->node && !check_node_in(dests, target))
	    {
		destination_t *new = NEW(destination_t);
		new->id = target;
		new->position = *get_node_position(target);
		das_insert(dests, (void*)new);
		new = NULL;
		found = true;
	    }
	}while(!found);
    }
   return;
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

tree_node_t* get_mst(call_t *call, void* dests)
{
    destination_t my_pos = THIS_DESTINATION(call), none = NO_DESTINATION; 
    void *added = das_create();

    //init tree root to source node
    tree_node_t *root = NEW(tree_node_t);
    root->this = my_pos;
    root->parent = none;
    root->children = das_create();

    //find next closest not already added target and add to tree
    while(das_getsize(added) < das_getsize(dests))
    {
	winner_t tmp, shortest;
	shortest.distance = 0;
	destination_t *to_add = NULL;

	das_init_traverse(dests);
	while((to_add = (destination_t*)das_traverse(dests)) != NULL)
	{
	    if(!check_node_in(added, to_add->id))
	    {
		tmp = get_shortest_dist(root, to_add);
		if(shortest.distance == 0 || tmp.distance < shortest.distance)
		    shortest = tmp;
	    }
	}
	tree_node_t *branch;
	if((branch = get_branch(root, shortest.parent)) == NULL)
	{
	    fprintf(stderr, "[ERR] Parent not found in tree\n");
	    delete_tree(root);
	    return NULL;
	}
	tree_node_t *new_child = NEW(tree_node_t);
	new_child->this = *shortest.to_add;
	new_child->parent = branch->this;
	new_child->children = das_create();
	das_insert(branch->children, (void*)new_child);
	new_child = NULL;
	das_insert(added, (void*)shortest.to_add);
    }
    return root;
}

winner_t get_shortest_dist(tree_node_t *root, destination_t *to_add)
{
    winner_t to_return, tmp;
    to_return.parent = &root->this;
    to_return.to_add = to_add;
    to_return.distance = distance(&root->this.position, &to_add->position);

    if(das_getsize(root->children) != 0)
    {
	tree_node_t *child = NULL;
	das_init_traverse(root->children);
	while((child = (tree_node_t*)das_traverse(root->children)) != NULL)
	{
	    tmp = get_shortest_dist(child, to_add);
	    if(tmp.distance < to_return.distance)
		to_return = tmp;
	}
    }
    return to_return;
}

tree_node_t* get_branch(tree_node_t *root, destination_t *to_get)
{
    if(compare_destinations(&root->this, to_get))
	return root;
    tree_node_t *child = NULL, *tmp = NULL;
    das_init_traverse(root->children);
    while((child = (tree_node_t*)das_traverse(root->children)) != NULL)
    {
	tmp = get_branch(child, to_get);
	if(tmp != NULL && compare_destinations(&tmp->this, to_get))
	    return tmp;
    }
    return NULL;
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
// Dijkstra Functions

int start_dijk(call_t *call, void *dests)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    entity_data->dijk_latency = get_time();
    node_data_t *node_data = NODE_DATA(call);
    destination_t my_pos = THIS_DESTINATION(call), none = NO_DESTINATION;
    packet_t *packet = NULL;

    if((packet = packet_alloc(call, node_data->overhead + sizeof(header_t) +
	DEFAULT_PACKET_SIZE)) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't allocate Dijkstra packet\nError setting"
	    " routing header\n");
	return ERROR;
    }
    header_t *header = PACKET_HEADER(packet, node_data);

    header->source = my_pos;
    header->next_node = none;
    header->type = DIJK_PACKET;
    header->direction = NO_DIR;
    header->mst = NULL;
    header->dests = NULL;
    header->ttl = 0;

    if((header->path = get_shortest_path(call, dests)) == NULL)
    {
	fprintf(stderr, "[ERR] No shortest path found\n");
	return ERROR;
    }
    tx(call, packet);
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
    void *visited = das_create(), *to_check = das_create();
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

    if(last_found->this == NONE)
	return NULL;
    void *path = das_create();
    nodeid_t *next = NEW(nodeid_t);
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

bool check_in_visited(void *visited, destination_t *to_check)
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
// Graph Functions

void planarize_graph(call_t *call)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *tmp = NULL, my_pos = THIS_DESTINATION(call);

    int i;
    for(i = 0; i < get_node_count(); ++i)
    {
	if(i != call->node && distance(&my_pos.position, get_node_position(i))
	    <= GG_RANGE)
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] GG: node %d adds %d to neighbor list\n",
		call->node, i);
#endif
	    tmp = NEW(destination_t);
	    tmp->id = i;
	    tmp->position = *get_node_position(i);
	    das_insert(node_data->nbrs, (void*)tmp);
	    tmp = NULL;
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
	    PRINT_ROUTING("[RTG] GG: node %d adds node %d to gg neighbor list\n"
		, call->node, tmp->id);
#endif
	    tmp2 = NEW(destination_t);
	    *tmp2 = *tmp;
	    das_insert(node_data->gg_list, (void*)tmp2);
	    tmp2 = NULL;
	}
    }
    return;
}

////////////////////////////////////////////////////////////////////////////////
// TX

void tx(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);

    if(header->type == DIJK_PACKET)
    {
	if(das_getsize(header->path) == 0)
	{
	    entity_data_t *entity_data = ENTITY_DATA(call);

	    entity_data->dijk_latency = get_time() - entity_data->dijk_latency;
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

    if(header->sender.id == NONE)
	traverse_first_face(call, packet);
    else
	forward(call, packet);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Routing Functions

void forward(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

    intersection_e result;
    destination_t tmp1 = next_on_face(call, &my_pos, &header->sender,
        TRAVERSE_R), tmp2 = next_on_face(call, &my_pos, &header->sender,
        TRAVERSE_L);

    if((result = check_intersect(call, &tmp1, header, header->mst))
	== COLLINEAR || check_node_in(header->dests, call->node))
    {
	spray_packets(call, packet);
	return;
    }
    if(header->direction == BOTH && compare_destinations(&tmp1, &tmp2))
    {
	header->next_node = tmp1;
	header->sender = my_pos;
	if(set_mac_header_tx(call, packet) == ERROR)
	    fprintf(stderr, "[ERR] Can't route packet at %d\n", call->node);
	return;
    }
    if(header->direction == BOTH)
    {
	packet_t *packet_2 = copy_packet(call, packet, header, INCREMENT);
	forward_packet(call, packet, TRAVERSE_L);
	forward_packet(call, packet_2, TRAVERSE_R);
	return;
    }
    forward_packet(call, packet, header->direction);
    return;
}

destination_t next_on_face(call_t *call, destination_t *start, destination_t
    *nodeRef, direction_e direction)
{
    node_data_t *node_data = NODE_DATA(call);
    destination_t *gg_nbr = NULL, winner = *start, max = *start,
	none = NO_DESTINATION;
    double angle, angle_min = 180.0, angle_max = 0.0, vect, d1, d2;

    if(das_getsize(node_data->gg_list) == 0)
	return none;

    das_init_traverse(node_data->gg_list);
    while((gg_nbr = (destination_t*)das_traverse(node_data->gg_list)) != NULL)
    {
	bool result = false;
	if(gg_nbr->id != nodeRef->id)
	{
	    position_t *temp_pos = &gg_nbr->position;
	    d1 = distance(&start->position, &nodeRef->position);
	    d2 = distance(&start->position, temp_pos);
	    angle = acos((double)(((start->position.x - nodeRef->position.x) *
		(start->position.x - temp_pos->x) + (start->position.y -
		nodeRef->position.y) * (start->position.y - temp_pos->y))
		/ (d1 * d2)));

	    vect = (double)((temp_pos->x - start->position.x) *
		(nodeRef->position.y - start->position.y)) -
		(double)((temp_pos->y - start->position.y) *
		(nodeRef->position.x - start->position.x));
	    if((direction == TRAVERSE_R && vect >= 0) || (direction ==
		TRAVERSE_L && vect <= 0))
		result = true;

	    if(result && angle <= angle_min)
	    {
		angle_min = angle;
		winner = *gg_nbr;
	    }
	    else if(!result && angle >= angle_max)
	    {
		angle_max = angle;
		max = *gg_nbr;
	    }
	}
    }
    if(compare_destinations(&winner, start))
	winner = max;
    if(compare_destinations(&winner, start))
	winner = *nodeRef;

    return winner;
}

intersection_e check_intersect(call_t *call, destination_t *next,
    header_t *header, tree_node_t *root)
{
    destination_t none = NO_DESTINATION, my_pos = THIS_DESTINATION(call);
    intersection_e result = NO_INT;

    if(das_getsize(root->children) != 0)
    {
	tree_node_t *child = NULL;
	das_init_traverse(root->children);
	while((child = (tree_node_t*)das_traverse(root->children)) != NULL)
	{
	    result = check_intersect(call, next, header, child);
	    if(result != NO_INT)
		return result;
	}
    }
    if(compare_destinations(&root->parent, &none) ||
	((compare_destinations(&header->sender, &my_pos) ||
	compare_destinations(&header->sender, next)) &&
	compare_destinations(&header->sender, &root->parent)))
	return result;

    position_t *current_pos = get_node_position(call->node);

    double vect1 = ((current_pos->x - root->this.position.x) *
	(root->parent.position.y - root->this.position.y)) - ((current_pos->y -
	root->this.position.y) * (root->parent.position.x -
	root->this.position.x));

    double vect2 = ((next->position.x - root->this.position.x) *
	(root->parent.position.y - root->this.position.y)) - ((next->position.y
	- root->this.position.y) * (root->parent.position.x -
	root->this.position.x));

    if((vect1 < 0 && vect2 < 0) || (vect1 > 0 && vect2 > 0))
	return NO_INT;

    double a1, a2, b1, b2, c1, c2, det, x, y, dist;

    a1 = root->parent.position.y - root->this.position.y;
    b1 = root->this.position.x - root->parent.position.x;
    c1 = a1 * root->this.position.x + b1 * root->this.position.y;
    a2 = next->position.y - current_pos->y;
    b2 = current_pos->x - next->position.x;
    c2 = a2 * current_pos->x + b2 * current_pos->y;

    if((det = a1 * b2 - a2 * b1) == 0)
    {
	if((root->this.position.x - root->parent.position.x != 0 &&
	    fmin(root->this.position.x, root->parent.position.x) <
	    fmax(current_pos->x, next->position.x) &&
	    fmax(root->this.position.x, root->parent.position.x) >
	    fmin(current_pos->x, next->position.x)) ||
	    (root->this.position.x - root->parent.position.x == 0 &&
	    fmin(root->this.position.y, root->parent.position.y) <
	    fmax(current_pos->x, next->position.x) &&
	    fmax(root->this.position.y, root->parent.position.y) >
	    fmin(current_pos->y, next->position.y)))
		return COLLINEAR;
    }
    else
    {
	x = (b2 * c1 - b1 * c2) / det;
	y = (a1 * c2 - a2 * c1) / det;

	dist = hypot((root->parent.position.x - x),
	    (root->parent.position.y - y)) +
	    hypot((root->this.position.x - x),
	    (root->this.position.y - y));
	if(((int)(distance(&root->this.position, &root->parent.position) *
	    PRECISION)) == (int)(dist * PRECISION))
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

void traverse_first_face(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call);

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] start routing from %d\n", call->node);
#endif

    header->sender = my_pos;
    header->direction = BOTH;
    spray_packets(call, packet);
    return;
}

void spray_packets(call_t *call, packet_t *packet)
{
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    destination_t my_pos = THIS_DESTINATION(call), reference = header->sender;
    bool sent = false;

    if(compare_destinations(&reference, &my_pos))
	reference = my_pos;
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
    result = check_intersect(call, &header->next_node, header, header->mst);
    if(!compare_destinations(&reference, &o_sender) ||
	(compare_destinations(&reference, &o_sender) && o_direction != BOTH
	&& direction == o_direction))
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
    if(result == NO_INT || result == COLLINEAR)
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

    while(check_intersect(call, &tmp, header, header->mst) ==
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
    if(header->direction == TRAVERSE_R)
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

bool combine_packets(call_t *call, destination_t *sender,
    destination_t *next_node, direction_e direction)
{
    node_data_t *node_data = NODE_DATA(call);

    bool combined = false;
    call_t call_down = CALL_DOWN(call);
    void *buffer = GET_BUFFER(&call_down);
    void *new_buffer = das_create();
    buffer_entry_t *entry = NULL;

    while((entry = (buffer_entry_t*)das_pop_FIFO(buffer)) != NULL)
    {
	header_t *header = PACKET_HEADER(entry->packet, node_data);
	if(compare_destinations(sender, &header->sender) &&
	    compare_destinations(next_node, &header->next_node))
	{
	    if((header->direction == TRAVERSE_R  && direction == TRAVERSE_L)
		|| (header->direction == TRAVERSE_L && direction == TRAVERSE_R))
		header->direction = BOTH;
#ifdef LOG_ROUTING
	    if(header->direction == TRAVERSE_L)
		PRINT_ROUTING("[RTG] packets combined, packet at %d sweep "
		    "left to %d\n", call->node, header->next_node.id);
	    else if(header->direction == TRAVERSE_R)
		PRINT_ROUTING("[RTG] packets combined, packet at %d sweep "
		    "right to %d\n", call->node, header->next_node.id);
	    else
		PRINT_ROUTING("[RTG] packets combined, packet at %d sweep "
		    "both directions to %d\n", call->node,
		    header->next_node.id);
#endif
		combined = true;
	}
	das_insert(new_buffer, (void*)entry);
    }
    SET_BUFFER(&call_down, new_buffer);
    return combined;
}

packet_t* copy_packet(call_t *call, packet_t *packet, header_t *header,
    operation_e operation)
{
    node_data_t *node_data = NODE_DATA(call);
    entity_data_t *entity_data = ENTITY_DATA(call);
    packet_t *copy_packet = packet_clone(packet);
    header_t *copy_header = PACKET_HEADER(copy_packet, node_data);

    copy_header->sender = header->sender;
    copy_header->next_node = header->next_node;
    copy_header->dests = header->dests;
    copy_header->mst = entity_data->mst;
    copy_header->direction = header->direction;
    copy_header->ttl = header->ttl;

    if(operation != DONT_INCREMENT)
	entity_data->num_packets++;
    return copy_packet;
}

////////////////////////////////////////////////////////////////////////////////
// RX

void rx(call_t *call, packet_t *packet)
{
    entity_data_t *entity_data = ENTITY_DATA(call);
    node_data_t *node_data = NODE_DATA(call);
    header_t *header = PACKET_HEADER(packet, node_data);
    array_t *up = get_entity_bindings_up(call);
    mate_e result;
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
	if((result = find_mate(call, packet)) == FOUND_OTHER_1)
	    return;
	if(result != FOUND_THIS)
	    tx(call, packet);
	if(!check_node_in(header->dests, call->node))
	    return;
    }
    else
    {
#ifdef LOG_ROUTING
	PRINT_ROUTING("[RTG] packet from %d to %d hit ttl\n",
	    header->sender.id, call->node);
#endif
	if(!check_node_in(header->dests, call->node))
	{
	    packet_dealloc(packet);
	    packet = NULL;
	    return;
	}
    }

#ifdef LOG_ROUTING
    PRINT_ROUTING("[RTG] delivering packet at %d\n", call->node);
#endif
    if(!node_data->received_packet)
    {
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

    if(header->direction == BOTH)
    {
	destination_t tmp1 = next_on_face(call, &my_pos, &header->sender,
	    TRAVERSE_R), tmp2 = next_on_face(call, &my_pos, &header->sender,
	    TRAVERSE_L);
	if(compare_destinations(&header->sender, &tmp1) &&
	    compare_destinations(&header->sender, &tmp2))
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] found self at %d, deallocating packets\n",
		call->node);
#endif
	    packet_dealloc(packet);
	    packet = NULL;
	    header = NULL;
	    return FOUND_THIS;
	}
    }

    buffer_entry_t *entry = NULL;
    call_t call_down = CALL_DOWN(call);
    void *buffer = GET_BUFFER(&call_down), *tmp = das_create();
    mate_e found = NOT_FOUND;

    while((entry = (buffer_entry_t*)das_pop_FIFO(buffer)) != NULL)
    {
	header_t *comp_header = PACKET_HEADER(entry->packet, node_data);
	if(compare_destinations(&comp_header->next_node, &header->sender) &&
	    compare_destinations(&comp_header->sender, &header->next_node) &&
	    (comp_header->direction != header->direction ||
	    (comp_header->direction == BOTH && header->direction == BOTH)))
	{
#ifdef LOG_ROUTING
	    PRINT_ROUTING("[RTG] found mates at %d, deallocating packets\n",
		call->node);
#endif
	    if(comp_header->direction == BOTH && header->direction != BOTH)
	    {
		comp_header->direction = header->direction;
		found = FOUND_OTHER_1;
	    }
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
// Helper Functions

void delete_tree(tree_node_t *root)
{
    if(das_getsize(root->children) != 0)
    {
	tree_node_t *child = NULL;

	das_init_traverse(root->children);
	while((child = (tree_node_t*)das_traverse(root->children)) != NULL)
	    delete_tree(child);
    }
    free(root);
    return;
}

bool compare_position(position_t *l, position_t *r)
{
    if(l == NULL || r == NULL)
	return false;
    if(l->x == r->x && l->y == r->y && l->z == r->z)
	return true;
    return false;
}

bool compare_destinations(destination_t *l, destination_t *r)
{
    if(l == NULL || r == NULL)
	return false;
    if(l->id == r->id && compare_position(&l->position, &r->position))
	return true;
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Postscript Functiond

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

void mst_post_axes(call_t *call)
{
    FILE *mst;
    position_t *topo_pos = get_topology_area();
    entity_data_t *entity_data = ENTITY_DATA(call);

    char *name = (char*)malloc(6 * sizeof(int));
    sprintf(name, "mst.ps");

    if((mst = fopen(name, "a")) == NULL)
    {
        fprintf(stderr, "[ERR] Couldn't open file %s in destroy()\n", name);
        fprintf(stderr, "[ERR] Error opening postscript file\n");
        return;
    }

    fprintf(mst, "15 15 moveto %lf 15 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript + 15);
    fprintf(mst, "15 15 moveto 15 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript + 15);

    fprintf(mst, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0);
    fprintf(mst, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
	    entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(mst, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0);
    fprintf(mst, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(mst, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);
    fprintf(mst, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);


    fprintf(mst, "0.2 setlinewidth\n");

    fprintf(mst, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(mst, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(mst, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(mst, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(mst, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(mst, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);

    fclose(mst);
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
	fprintf(stderr, "[ERR] Couldn't open file %s\n[ERR] Error opening "
	    "postscript file\n", name);
	return;
    }

    if(call->node == 0)
	fprintf(graph_gg, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0"
	    " 0 0 setrgbcolor\n", pos->x * entity_data->scale_postscript + 15.0,
	    pos->y * entity_data->scale_postscript + 15.0);
    else
	fprintf(graph_gg, "%lf %lf 2.5 0 360 arc fill stroke\n", pos->x *
	    entity_data->scale_postscript + 15.0, pos->y *
	    entity_data->scale_postscript + 15.0);

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
	fprintf(stderr, "[ERR] Couldn't open file %s\n[ERR] Error opening "
	    "postscript file\n", name);
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
    {
	fprintf(graph, "%lf %lf moveto %lf %lf lineto stroke\n", pos->x *
	    entity_data->scale_postscript + 15.0, pos->y *
	    entity_data->scale_postscript + 15.0, nbr->position.x *
	    entity_data->scale_postscript + 15.0, nbr->position.y *
	    entity_data->scale_postscript + 15.0);
    }

    fclose(graph);
    return;
}

void mst_post_nodes(call_t *call, tree_node_t *root)
{
    destination_t none = NO_DESTINATION;

    if(das_getsize(root->children) != 0)
    {
	tree_node_t *child = NULL;

	das_init_traverse(root->children);
	while((child = (tree_node_t*)das_traverse(root->children)) != NULL)
	    mst_post_nodes(call, child);
    }

    entity_data_t *entity_data = ENTITY_DATA(call);
    FILE *mst = NULL;

    char *name = (char*)malloc(6 * sizeof(int));
    sprintf(name, "mst.ps");

    if((mst = fopen(name, "a")) == NULL)
    {
	fprintf(stderr, "[ERR] Couldn't open file %s\n[ERR] Error opening "
	    "postscript file\n", name);
	return;
    }

    if(compare_destinations(&root->parent, &none))
	fprintf(mst, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0"
	    " 0 0 setrgbcolor\n", root->this.position.x *
	    entity_data->scale_postscript + 15.0, root->this.position.y *
	    entity_data->scale_postscript + 15.0);
    else
	fprintf(mst, "%lf %lf 2.5 0 360 arc fill stroke\n",
	    root->this.position.x * entity_data->scale_postscript + 15.0,
	    root->this.position.y * entity_data->scale_postscript + 15.0);

    fprintf(mst, "/Times-Roman findfont 14 scalefont setfont newpath %lf "
	"%lf moveto (%d) show\n", root->this.position.x *
	entity_data->scale_postscript + 18.0, root->this.position.y *
	entity_data->scale_postscript + 18.0, root->this.id);

    if(!compare_destinations(&root->parent, &none))
	fprintf(mst, "%lf %lf moveto %lf %lf lineto stroke\n",
	    root->this.position.x * entity_data->scale_postscript + 15.0,
	    root->this.position.y * entity_data->scale_postscript + 15.0,
	    root->parent.position.x * entity_data->scale_postscript + 15.0,
	    root->parent.position.y * entity_data->scale_postscript + 15.0);

    fclose(mst);
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
	fprintf(stderr, "[ERR] Couldn't open file %s\n[ERR] Error opening "
	    "postscript file\n", name);
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
