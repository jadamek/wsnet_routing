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

#define GG_RANGE 1

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
    destination_t *sender;
    void *dests;
    tree_node_t *mst;
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
void planarize_graph(call_t*);
void tx(call_t*, packet_t*);
void rx(call_t*, packet_t*);
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

#if defined LOG_TOPO_G || defined LOG_GG
    position_t *topo_pos = get_topology_area();
    entity_data->scale_postscript = 1700.0 / (topo_pos->x + 20);
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

#ifdef LOG_GG
    gg_post_axes(call);
#endif
#ifdef LOG_TOPO_G
    topo_post_axes(call);
#endif
#ifdef LOG_MST
    mst_post_nodes(call, entity_data->mst);
    mst_post_axes(call);
#endif
    delete_tree(entity_data->mst);

    free(entity_data);
    entity_data = NULL;
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
    planarize_graph(call);
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
    destination_t none = NO_DESTINATION;

    header->sender = &none;
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
    header->mst = header->mst;

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
    return;
}

////////////////////////////////////////////////////////////////////////////////
// RX

void rx(call_t *call, packet_t *packet)
{
    return;
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
