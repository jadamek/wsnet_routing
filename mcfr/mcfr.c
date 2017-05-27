// James Robinson
// 5/25/2017
// Module for MCFR: CFR over a steiner tree

#include <unistd.h>
#include <stdbool.h>

#include "include/modelutils.h"

////////////////////////////////////////////////////////////////////////////////
// Model Info

model_t model =
{
        "MCFR",
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
#define LOG_STEINER

#define ERROR -1
#define NONE -2
#define PI 3.14159265

#define PERIOD 5000000000ull
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
// Typdefs and Structs

typedef enum {DATA_PACKET, DIJK_PACKET} packet_e;
typedef enum {NO_DIR, TRAVERSE_R, TRAVERSE_L, BOTH} direction_e;
typedef enum {NO_INT, INTERSECTION, COLLINEAR} intersection_e;
typedef enum {NOT_FOUND, FOUND_THIS, FOUND_OTHER_1, FOUND_OTHER_2} mate_e;
typedef enum {DONT_INCREMENT, INCREMENT} operation_e;

typedef struct tnode
{
    destination_t parent;
    destination_t location;
    struct tnode **children;
    int child_count;
    bool isVirtual;
    bool isActive;
}tree_node_t;

typedef struct
{
    tree_node_t *s;
    tree_node_t *t;
    tree_node_t *u;
    tree_node_t *v;
    double reduction_ratio;
    bool active;
}steiner_tuple;

typedef struct
{
    nodeid_t this;
    nodeid_t prev;
}visited_node_t;

typedef struct
{
    nodeid_t target;
    int num_hops;
    uint64_t start;
    uint64_t latency;
    void *path;
}path_t;

typedef struct
{
    destination_t source;
    destination_t sender;
    destination_t next_node;
    void *dests;
    tree_node_t *steiner;
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
    uint64_t punishment_latency;
    void *delivered_to;
    nodeid_t last_reached;
    void *paths;

    float scale_postscript;
    tree_node_t *steiner;
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
tree_node_t* build_steiner_tree(destination_t*, void*);
tree_node_t* generate_real_node(destination_t*);
tree_node_t* generate_virtual_node(position_t);
void add_child(tree_node_t*, tree_node_t*);
void remove_child(tree_node_t*, int);
double distance_sum(tree_node_t*, position_t);
int sub_target_count(tree_node_t*);
destination_t** sub_target_list(tree_node_t*);
void accumulate_target_list(tree_node_t*, destination_t**, int*);
steiner_tuple* generate_steiner_tuple(tree_node_t*, tree_node_t*, tree_node_t*);
int compare_steiner_tuples(const void*, const void*);
double get_angle_single(position_t);
double get_angle_double(position_t, position_t);
double get_angle_triple(position_t, position_t, position_t);
double get_acute_angle_triple(position_t, position_t, position_t);
position_t equillateral_third(position_t, position_t, position_t);
position_t get_intersection_point(position_t, position_t, position_t,
    position_t);
tree_node_t* steiner_point(tree_node_t*, tree_node_t*, tree_node_t*);
double compute_node_distance(tree_node_t*, tree_node_t*);
double compute_distance(position_t, position_t);
int get_header_size(call_t*);
int get_header_real_size(call_t*);
int start_dijk(call_t*, void*);
int dijk_start(call_t*, void*);
void* get_shortest_path(call_t*, void*);
void* make_path(call_t*, void*, visited_node_t*);
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
double absolute_value(double);
bool compare_position(position_t*, position_t*);
bool compare_destinations(destination_t*, destination_t*);
void gg_post_axes(call_t*);
void topo_post_axes(call_t*);
void steiner_post_axes(call_t*);
void gg_post_nodes(call_t*);
void topo_post_nodes(call_t*);
void steiner_post_nodes(call_t*, tree_node_t*);
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
    if((entity_data->delivered_to = das_create()) == NULL)
    {
	fprintf(stderr, "[ERR] Can't allocate entity_data delivered_to\n");
	return ERROR;
    }
    if((entity_data->paths = das_create()) == NULL)
    {
        fprintf(stderr, "[ERR] Can't alocate entity_data paths\n");
        return ERROR;
    }

    entity_data->total_num_hops = 0;
    entity_data->deliver_num_hops = 0;
    entity_data->num_packets = 0;
    entity_data->num_reachable = 0;
    entity_data->dijk_latency = 0;
    entity_data->punishment_latency = 0;
    entity_data->last_reached = NONE;
    entity_data->steiner = NULL;

#if defined LOG_TOPO_G || defined LOG_GG || defined LOG_STEINER
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
#ifdef LOG_STEINER
    if(access("steiner.ps", F_OK) != ERROR)
	remove("steiner.ps");
#endif

    set_entity_private_data(call, entity_data);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    entity_data_t *entity_data = ENTITY_DATA(call);

    delete_tree(entity_data->steiner);

    FILE *results;
    char* name= (char*)malloc(11 * sizeof(int));
    sprintf(name, "results.txt");


    if(das_getsize(entity_data->delivered_to) < entity_data->num_reachable)
    {
	bool punished_lat = false;
        path_t *to_check = NULL;
        das_init_traverse(entity_data->paths);
        while((to_check = (path_t*)das_traverse(entity_data->paths)) != NULL)
        {
	    if(!punished_lat && to_check->latency != 0)
	    {
		entity_data->punishment_latency = to_check->latency;
		punished_lat = true;
	    }
            if(to_check->target == entity_data->last_reached)
                entity_data->dijk_latency = to_check->latency;
	    if(!check_node_in(entity_data->delivered_to, to_check->target))
		entity_data->num_packets += to_check->num_hops;
        }
    }
    else if(entity_data->last_reached != NONE)
    {
	path_t *to_check = NULL;
	das_init_traverse(entity_data->paths);
	while((to_check = (path_t*)das_traverse(entity_data->paths)) != NULL)
	{
	    if(to_check->target == entity_data->last_reached)
	    {
		entity_data->dijk_latency = to_check->latency;
	    }
	}
    }

    fprintf(stderr, "Routing Statistics:\n");
    fprintf(stderr, "  number of packets: %d\n", entity_data->num_packets);
    fprintf(stderr, "  number of hops to deliver: %d\n",
        entity_data->deliver_num_hops);
    fprintf(stderr, "  total number of hops: %d\n",
        entity_data->total_num_hops);
    fprintf(stderr, "  BFS latency: %lld nanoseconds\n",
        entity_data->dijk_latency);
    fprintf(stderr, "  Punishment Latency: %lld nanoseconds\n",
	entity_data->punishment_latency);
    fprintf(stderr, "  Num reachable targets: %d\n",
        entity_data->num_reachable);

    if((results = fopen(name, "a")) == NULL)
        fprintf(stderr, "[ERR] Couldn't open file %s\n", name);
    else
        fprintf(results, "%lld\t%lld\t%d\t%d\n", entity_data->dijk_latency,
            entity_data->punishment_latency, entity_data->num_reachable,
	    entity_data->num_packets);


    destination_t *to_delete = NULL;
    das_init_traverse(entity_data->delivered_to);
    while((to_delete = (destination_t*)das_traverse(entity_data->delivered_to))
	!= NULL)
	free(to_delete);
    das_destroy(entity_data->delivered_to);
    to_delete = NULL;
    path_t *to_destroy = NULL;
    das_init_traverse(entity_data->paths);
    while((to_destroy = (path_t*)das_traverse(entity_data->paths)) != NULL)
    {
	das_destroy(to_destroy->path);
	free(to_destroy);
    }
    das_destroy(entity_data->paths);
    to_destroy = NULL;

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
    if((entity_data->steiner = build_steiner_tree(&header->source,
        header->dests)) == NULL)
    {
        fprintf(stderr, "[ERR] Error calculating steiner tree\n");
        return ERROR;
    }
    header->steiner = entity_data->steiner;
    if(start_dijk(call, header->dests) == ERROR)
        fprintf(stderr, "[ERR] unable to start shortest path\n");

#ifdef LOG_STEINER
    steiner_post_nodes(call, entity_data->steiner);
    steiner_post_axes(call);
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

tree_node_t* build_steiner_tree(destination_t *source, void* target_list)
{
    int i = 0, j, k, target_count = das_getsize(target_list),
        updated_target_count, updated_pair_count;
    destination_t *tmp = NULL;

    tree_node_t *root = generate_real_node(source);

    tree_node_t **destinations = malloc(sizeof(tree_node_t*)
        * target_count);

    das_init_traverse(target_list);
    while((tmp = (destination_t*)das_traverse(target_list)) != NULL)
    {
        destinations[i] = generate_real_node(tmp);
        i++;
    }

    int pair_count = target_count * (target_count + 1) / 2;
    steiner_tuple **pairs = malloc(sizeof(steiner_tuple*) * pair_count);

    for(i = 0, k = 0; i < target_count; i++)
    {
        for(j = 0; j < target_count; j++)
        {
            if(destinations[i]->location.id <= destinations[j]->location.id)
                pairs[k++] = generate_steiner_tuple(root, destinations[i],
                    destinations[j]);
        }
    }

    qsort(pairs, pair_count, sizeof(steiner_tuple*), compare_steiner_tuples);

    tree_node_t *s = root, *addition = NULL;
    bool clean = false, clean_pairs = false;

    while(pair_count > 0)
    {
        steiner_tuple *best = pairs[0];

        if(best->u == best->v)
        {
            add_child(s, best->u);
            clean = true;
        }
        else if(best->t == s)
        {
            add_child(s, best->u);
            add_child(s, best->v);
            clean = true;
        }
        else if(best->t == best->u)
        {
            add_child(best->u, best->v);
            clean = true;
        }
        else if(best->t == best->v)
        {
            add_child(best->v, best->u);
            clean = true;
        }
        else if(compute_node_distance(s, best->u) < GG_RANGE
            && compute_node_distance(s, best->v) <  GG_RANGE)
        {
            best->active = false;
            clean_pairs = true;
        }
        else if(compute_node_distance(s, best->u) < GG_RANGE)
        {
            if(GG_RANGE + compute_node_distance(best->t, best->u)
                + compute_node_distance(best->t, best->v)
                > compute_node_distance(s, best->u)
                + compute_node_distance(s, best->v))
            {
                best->active = false;
                clean_pairs = true;
            }
            else
            {
                add_child(best->u, best->v);
                clean = true;
            }
        }
        else if(compute_node_distance(s, best->v) < GG_RANGE)
        {
            if(GG_RANGE + compute_node_distance(best->t, best->u)
                + compute_node_distance(best->t, best->v)
                > compute_node_distance(s, best->u)
                + compute_node_distance(s, best->v))
            {
                best->active = false;
                clean_pairs = true;
            }
            else
            {
                add_child(best->v, best->u);
                clean = true;
            }
        }
        else if(compute_node_distance(s, best->t) < GG_RANGE
            && GG_RANGE + compute_node_distance(best->t, best->u)
            + compute_node_distance(best->t, best->v)
            > compute_node_distance(s, best->u)
            + compute_node_distance(s, best->v))
        {
            add_child(s, best->u);
            add_child(s, best->v);
            clean = true;
        }
        else
        {
            add_child(best->t, best->u);
            add_child(best->t, best->v);
            addition = best->t;
            clean = true;
        }

        if(clean)
        {
            for(i = 0, updated_target_count = 0; i < target_count; i++)
                if(destinations[i]->isActive)
                    updated_target_count++;

            if(addition != NULL)
                updated_target_count++;

            tree_node_t **updated_destinations = malloc(sizeof(tree_node_t*)
                * updated_target_count);
	    for(i = 0, j = 0; i < target_count; i++)
                if(destinations[i]->isActive)
                    updated_destinations[j++] = destinations[i];

            if(addition != NULL)
                updated_destinations[j] = addition;

            destinations = updated_destinations;
            target_count = updated_target_count;
        }

        if(clean || clean_pairs)
        {
            for(i = 0, updated_pair_count = 0; i < pair_count; i++)
                if(pairs[i]->active && (pairs[i]->u->isActive
                    && pairs[i]->v->isActive))
                    updated_pair_count++;

            if(addition != NULL)
                updated_pair_count += target_count;

            steiner_tuple **updated_pairs = malloc(sizeof(steiner_tuple*)
                * updated_pair_count);

            for(i = 0, j = 0; i < pair_count; i++)
                if(pairs[i]->active && (pairs[i]->u->isActive
                    && pairs[i]->v->isActive))
                    updated_pairs[j++] = pairs[i];

            if(addition != NULL)
                for(i = 0; i < target_count; i++)
                    updated_pairs[j++] = generate_steiner_tuple(root, addition,
                        destinations[i]);

            pairs = updated_pairs;
            pair_count = updated_pair_count;

            if(addition != NULL)
                qsort(pairs, pair_count, sizeof(steiner_tuple*),
                    compare_steiner_tuples);

            clean = false;
            clean_pairs = false;
            addition = NULL;
        }
    }
    return root;
}

tree_node_t* generate_real_node(destination_t *target)
{
    tree_node_t *node = NEW(tree_node_t);
    destination_t none = NO_DESTINATION;

    node->parent = none;
    node->location = *target;
    node->children = NULL;
    node->child_count = 0;
    node->isVirtual = false;
    node->isActive = true;

    return node;
}

tree_node_t* generate_virtual_node(position_t position)
{
    tree_node_t *vnode = NEW(tree_node_t);
    destination_t *vdest = NEW(destination_t), none = NO_DESTINATION;

    vdest->id = -3;
    vdest->position.x = position.x;
    vdest->position.y = position.y;
    vdest->position.z = position.z;

    vnode->parent = none;
    vnode->location = *vdest;
    vnode->children = NULL;
    vnode->child_count = 0;
    vnode->isVirtual = true;
    vnode->isActive = true;

    return vnode;
}

void add_child(tree_node_t *parent, tree_node_t *child)
{
    tree_node_t **children = parent->children;
    int i = 0;

    parent->child_count++;
    parent->children = malloc(sizeof(tree_node_t*) * parent->child_count);

    if(children != NULL)
	for(i = 0; i < parent->child_count - 1; i++)
	    parent->children[i] = children[i];

    parent->children[i] = child;
    child->parent = parent->location;
    child->isActive = false;
}

void remove_child(tree_node_t *parent, int child_index)
{
    tree_node_t **children = parent->children;
    int i, j;

    parent->child_count--;
    parent->children = malloc(sizeof(tree_node_t*) * parent->child_count);

    for(i = 0, j = 0; i < parent->child_count; j++)
    {
	parent->children[i] = children[j];
	if(i != child_index)
	    i++;
    }
    return;
}

double distance_sum(tree_node_t *node, position_t origin)
{
    double d = (node->isVirtual ? 0 : compute_distance(node->location.position
	, origin));

    int i;
    for(i = 0; i < node->child_count; i++)
	d += distance_sum(node->children[i], origin);

    return d;
}

int sub_target_count(tree_node_t *node)
{
    int total = 0, i;

    if(!node->isVirtual)
	total++;

    for(i = 0; i < node->child_count; i++)
	total += sub_target_count(node->children[i]);

    return total;
}

destination_t** sub_target_list(tree_node_t *node)
{
    destination_t **target_list = malloc(sizeof(destination_t*) *
	sub_target_count(node));
    int current = 0;

    accumulate_target_list(node, target_list, &current);

    return target_list;
}

void accumulate_target_list(tree_node_t *node, destination_t **target_list,
    int *current)
{
    if(!node->isVirtual)
	target_list[(*current)++] = &node->location;

    int i;
    for(i = 0; i < node->child_count; i++)
	accumulate_target_list(node->children[i], target_list, current);
}

steiner_tuple* generate_steiner_tuple(tree_node_t *root, tree_node_t *left,
    tree_node_t *right)
{
    steiner_tuple *tuple = NEW(steiner_tuple);
    tuple->s = root;
    tuple->t = steiner_point(root, left, right);
    tuple->u = left;
    tuple->v = right;
    tuple->active = true;

     double st = compute_node_distance(tuple->s, tuple->t),
	su = compute_node_distance(tuple->s, tuple->u),
	sv = compute_node_distance(tuple->s, tuple->v),
	tu = compute_node_distance(tuple->t, tuple->u),
	tv = compute_node_distance(tuple->t, tuple->v);

    tuple->reduction_ratio = 1.0 - (st + tu + tv) / (su + sv);

    return tuple;
}

int compare_steiner_tuples(const void *tuple1, const void *tuple2)
{
    double ratio1 = (*((steiner_tuple**)tuple1))->reduction_ratio,
	ratio2 = (*((steiner_tuple**)tuple2))->reduction_ratio;

    if(ratio1 > ratio2)
	return -1;
    if(ratio1 < ratio2)
	return 1;
    return 0;
}

double get_angle_single(position_t a)
{
    return atan2(a.y, a.x);
}

double get_angle_double(position_t a, position_t b)
{
    return atan2(a.y - b.y, a.x - b.x);
}

double get_angle_triple(position_t a, position_t b, position_t c)
{
    return atan2(c.y - b.y, c.x - b.x) - atan2(a.y - b.y, a.x - b.x);
}

double get_acute_angle_triple(position_t a, position_t b, position_t c)
{
    double angle = absolute_value(get_angle_triple(a, b, c));

    if(angle > PI)
	angle = 2 * PI - angle;

    return angle;
}

position_t get_equillateral_third(position_t a, position_t b, position_t c)
{
    double dAB = compute_distance(a, b),
        AB = PI - get_angle_double(a, b),
        BAC = get_angle_triple(b, a, c),
        shift = PI / 3;

    if(BAC < 0 || absolute_value(BAC) > PI)
        shift *= -1;

    AB += shift;

    position_t equillateral_third;
    equillateral_third.x = a.x + dAB * cos(AB);
    equillateral_third.y = a.y - dAB * sin(AB);
    equillateral_third.z = 0;

    return equillateral_third;
}

tree_node_t* steiner_point(tree_node_t *a, tree_node_t *b, tree_node_t *c)
{
    double bac = get_acute_angle_triple(b->location.position,
        a->location.position, c->location.position),
        abc = get_acute_angle_triple(a->location.position, b->location.position,
        c->location.position),
        acb = get_acute_angle_triple(a->location.position, c->location.position,
        b->location.position);

    if(compare_position(&a->location.position, &b->location.position))
        return c;
    else if(compare_position(&a->location.position, &c->location.position))
        return b;
    else if(compare_position(&b->location.position, &c->location.position))
        return a;
    else if(bac > 2 * PI / 3)
        return a;
    else if(abc > 2 * PI / 3)
        return b;
    else if(acb > 2 * PI / 3)
        return c;
    else
    {
        position_t ab = get_equillateral_third(a->location.position,
            b->location.position, c->location.position),
            ac = get_equillateral_third(a->location.position,
            c->location.position, b->location.position),
            steiner = get_intersection_point(ab, c->location.position,
            ac, b->location.position);
        return generate_virtual_node(steiner);
    }
}

double compute_node_distance(tree_node_t *a, tree_node_t *b)
{
    return compute_distance(a->location.position, b->location.position);
}

double compute_distance(position_t a, position_t b)
{
    double dx = a.x - b.x, dy = a.y - b.y;

    return sqrt(dx * dx + dy * dy);
}

position_t get_intersection_point(position_t a1, position_t a2, position_t b1,
    position_t b2)
{
    position_t intersection;
    intersection.z = 0;

    double m1 = 0, m2 = 0;

    if(a2.x != a1.x)
        m1 = (a2.y - a1.y) / (a2.x - a1.x);
    if(b2.x != b1.x)
        m2 = (b2.y - b1.y) / (b2.x - b1.x);

    if(a2.x == a1.x && b2.x == b1.x)
    {
        intersection.x = 0;
        intersection.y = 0;
    }
    else if(a2.x == a1.x)
    {
        intersection.x = a1.x;
        intersection.y = m2 * (a1.x - b1.x) + b1.y;
    }
    else if(b2.x == b1.x)
    {
        intersection.x = b1.x;
        intersection.y = m1 * (b1.x - a1.x) + a1.y;
    }
    else
    {
        intersection.x = (a1.y - m1 * a1.x - b1.y + m2 * b1.x) / (m2 - m1);
        intersection.y = (m1 * m2 * (b1.x - a1.x) - m1 * b1.y + m2 * a1.y)
            / (m2 - m1);
    }

    return intersection;
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
    header->steiner = NULL;
    header->dests = NULL;
    header->ttl = 0;

    if((entity_data->paths = get_shortest_path(call, dests)) == NULL)
    {
        fprintf(stderr, "[ERR] No shortest path found\n");
        return ERROR;
    }

    int i;
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
			new_path->num_hops = das_getsize(new_path->path);
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

    if((result = check_intersect(call, &tmp1, header, header->steiner))
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

    if(root->child_count != 0)
    {
	int i;
	for(i = 0; i < root->child_count; i++)
	{
	    result = check_intersect(call, next, header, root->children[i]);
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

    double vect1 = ((current_pos->x - root->location.position.x) *
        (root->parent.position.y - root->location.position.y)) -
	((current_pos->y - root->location.position.y) *
	(root->parent.position.x - root->location.position.x));

    double vect2 = ((next->position.x - root->location.position.x) *
        (root->parent.position.y - root->location.position.y)) -
	((next->position.y - root->location.position.y) *
	(root->parent.position.x - root->location.position.x));

    if((vect1 < 0 && vect2 < 0) || (vect1 > 0 && vect2 > 0))
        return NO_INT;

    double a1, a2, b1, b2, c1, c2, det, x, y, dist;

    a1 = root->parent.position.y - root->location.position.y;
    b1 = root->location.position.x - root->parent.position.x;
    c1 = a1 * root->location.position.x + b1 * root->location.position.y;
    a2 = next->position.y - current_pos->y;
    b2 = current_pos->x - next->position.x;
    c2 = a2 * current_pos->x + b2 * current_pos->y;

    if((det = a1 * b2 - a2 * b1) == 0)
    {
        if((root->location.position.x - root->parent.position.x != 0 &&
            fmin(root->location.position.x, root->parent.position.x) <
            fmax(current_pos->x, next->position.x) &&
            fmax(root->location.position.x, root->parent.position.x) >
            fmin(current_pos->x, next->position.x)) ||
            (root->location.position.x - root->parent.position.x == 0 &&
            fmin(root->location.position.y, root->parent.position.y) <
            fmax(current_pos->x, next->position.x) &&
            fmax(root->location.position.y, root->parent.position.y) >
            fmin(current_pos->y, next->position.y)))
                return COLLINEAR;
    }
    else
    {
        x = (b2 * c1 - b1 * c2) / det;
        y = (a1 * c2 - a2 * c1) / det;

        dist = hypot((root->parent.position.x - x),
            (root->parent.position.y - y)) +
            hypot((root->location.position.x - x),
            (root->location.position.y - y));
        if(((int)(distance(&root->location.position, &root->parent.position) *
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
    result = check_intersect(call, &header->next_node, header, header->steiner);
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

    while(check_intersect(call, &tmp, header, header->steiner) ==
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
    copy_header->steiner = entity_data->steiner;
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
	destination_t *new_reached = NEW(destination_t);
	new_reached->id = call->node;
	new_reached->position = *get_node_position(call->node);
	das_insert(entity_data->delivered_to, (void*)new_reached);
	new_reached = NULL;
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
    if(root->child_count != 0)
    {
	int i;
	for(i = 0; i < root->child_count; i++)
	    delete_tree(root->children[i]);
    }
    free(root);
    return;
}

double absolute_value(double n)
{
    if(n < 0)
	n *= -1;
    return n;
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
// Postscript Functions

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

void steiner_post_axes(call_t *call)
{
    FILE *steiner;
    position_t *topo_pos = get_topology_area();
    entity_data_t *entity_data = ENTITY_DATA(call);

    char *name = (char*)malloc(10 * sizeof(int));
    sprintf(name, "steiner.ps");

    if((steiner = fopen(name, "a")) == NULL)
    {
        fprintf(stderr, "[ERR] Couldn't open file %s in destroy()\n", name);
        fprintf(stderr, "[ERR] Error opening postscript file\n");
        return;
    }

    fprintf(steiner, "15 15 moveto %lf 15 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript + 15);
    fprintf(steiner, "15 15 moveto 15 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript + 15);

    fprintf(steiner, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0);
    fprintf(steiner, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(steiner, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0);
    fprintf(steiner, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(steiner, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);
    fprintf(steiner, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);


    fprintf(steiner, "0.2 setlinewidth\n");

    fprintf(steiner, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(steiner, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 4.0 + 15.0);

    fprintf(steiner, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(steiner, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript / 2.0 + 15.0);

    fprintf(steiner, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->x *
            entity_data->scale_postscript + 15.0);
    fprintf(steiner, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0, topo_pos->y *
            entity_data->scale_postscript + 15.0, topo_pos->y *
            entity_data->scale_postscript * 3.0 / 4.0 + 15.0);

    fclose(steiner);
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

void steiner_post_nodes(call_t *call, tree_node_t *root)
{
    destination_t none = NO_DESTINATION;

    if(root->child_count != 0)
    {
	int i;
	for(i = 0; i < root->child_count; i++)
	    steiner_post_nodes(call, root->children[i]);
    }

    entity_data_t *entity_data = ENTITY_DATA(call);
    FILE *steiner = NULL;

    char *name = (char*)malloc(6 * sizeof(int));
    sprintf(name, "steiner.ps");

    if((steiner = fopen(name, "a")) == NULL)
    {
        fprintf(stderr, "[ERR] Couldn't open file %s\n[ERR] Error opening "
            "postscript file\n", name);
        return;
    }

    if(compare_destinations(&root->parent, &none))
        fprintf(steiner, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0"
            " 0 0 setrgbcolor\n", root->location.position.x *
            entity_data->scale_postscript + 15.0, root->location.position.y *
            entity_data->scale_postscript + 15.0);
    else
        fprintf(steiner, "%lf %lf 2.5 0 360 arc fill stroke\n",
            root->location.position.x * entity_data->scale_postscript + 15.0,
            root->location.position.y * entity_data->scale_postscript + 15.0);

    fprintf(steiner, "/Times-Roman findfont 14 scalefont setfont newpath %lf "
        "%lf moveto (%d) show\n", root->location.position.x *
        entity_data->scale_postscript + 18.0, root->location.position.y *
        entity_data->scale_postscript + 18.0, root->location.id);

    if(!compare_destinations(&root->parent, &none))
        fprintf(steiner, "%lf %lf moveto %lf %lf lineto stroke\n",
            root->location.position.x * entity_data->scale_postscript + 15.0,
            root->location.position.y * entity_data->scale_postscript + 15.0,
            root->parent.position.x * entity_data->scale_postscript + 15.0,
            root->parent.position.y * entity_data->scale_postscript + 15.0);

    fclose(steiner);
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

