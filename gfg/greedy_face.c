/**
*  \file   greedyFaceGreedy.c
*  \brief  Greedy-FACE-Greedy geographic routing
*  \author Cesar Marchal
*  \date   2014
**/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#include <include/modelutils.h>


/* ************************************************** */
/* ************************************************** */
model_t model =  {
  "Greedy-Face-Greedy geographic routing",
  "Cesar Marchal",
  "0.1",
  MODELTYPE_ROUTING,
  {NULL, 0}
};


/* *********************************************** */
/* *********************************************** */

#define GG                  1
#define SENSOR              2           /*** Node default type ***/

#define HELLO_PACKET        1
#define DATA_PACKET         2

#define NEIGHBOR_BASED      1
#define GG_BASED            2
#define RANDOM_BASED        3

#define SYMETRIK            3
#define ASYMETRIK           4

#define DEG2RAD(angle)      (M_PI * angle / 180)

#define MAXDATA_SIZE        32



/* ************************************************** */
/* ************************************************** */
struct routing_header {
  nodeid_t dst;
  position_t dst_pos;
  nodeid_t src;
  position_t src_pos;
  nodeid_t sender;
  // Utiles car destination utilisée pour broadcaster (comme ça, le noeud précédent peut vérifier si le forwarding a bien été réalisé ou non)
  // Useful as destination used for broadcaster (like that, the previous node can check whether forwarding has been acheived or not)
  nodeid_t next_node;
  position_t next_node_pos;
  int32_t numseq;
  //
  int face_initiator;
  position_t face_initiator_pos;
  int face_activated; // 1 -> activated // 0 -> No
  int hop;
  int type;
};

struct hello {
  int nbr;
};


struct gg_neighbor {
  int         id;
  position_t  position;
  int         mark;
  int         link_type;
  uint64_t    time;
  position_t  middle_pos;
  double      distance;
  double      angle;
  void        *neighborlist;
  void        *witnessnode;
};


struct gg_neighbor_hello   {
  int         neighbors_id;
  position_t  neg_pos;
  int         neighbor_type;
};


struct gg_neighborhood {
  int         neighbor_idN;
  position_t  NGhood_pos;
  int         link_typeN;
  uint64_t    node_ttl;
  double      dist;
  double      dist_from_C;
};


struct GGnode {
  int         GG_id;
  position_t  GG_pos;
  double      angle;
  uint64_t    timeGG;
  double      dist_GG;
};


struct nodedata {
  void *neighbors;
  void *GGlist;
  int overhead;

  uint64_t start;
  uint64_t period;
  uint64_t timeout;
  int hop;

  /* stats */
  int hello_tx;
  int hello_rx;
  int data_tx;
  int data_rx;
  int data_noroute;
  int data_hop;
};

struct entitydata {
   int nb_hop;
   int nb_rx;
   int face_activation;
   int face_call;
   int nb_paquets_too_many_hops;

   /** USED TO PRINT POSTSCRIPT GRAPH IN ONE SHEET **/
   float scale_postscript;
};


/* ************************************************** */
/* ************************************************** */
int hello_callback(call_t *c, void *args);
void onehoplist_time(call_t *c , void *args);
void set_mark(call_t *c, int node);
void add_neighbor(call_t *c, packet_t *packet);
void GG_Neighbor(call_t *c, void *args);
void updateGG(call_t *c, void *args);
int check_in_neighbor(call_t *c, packet_t *packet);
int search_node(call_t *c, int identity);
struct gg_neighbor* getnode(call_t *c, int ggid);
void setwitness_node(call_t *c, void *args);
int neighbor_timeout(void *data, void *args);
int neighborhood_timeout(void *data, void *args);
int GG_timeout(void *data, void *args);
void make_witness(call_t *c, packet_t *packet);
void Witness_process(call_t *c);
struct GGnode *Get_GGNode(call_t *c, position_t *dst_pos);
double Angular(position_t *position, position_t *pf_pos, position_t *dst_pos);
int compare_headers(struct routing_header *first, struct routing_header *second);
struct gg_neighbor* getnode(call_t *c, int ggid);
struct gg_neighbor* get_nexthop_FACE(call_t *c, struct routing_header *header);
struct gg_neighbor* get_nexthop_greedy(call_t *c, struct routing_header *header);
struct gg_neighbor* nextOnFace(call_t *c, int depart, position_t* depart_pos, int noeudRef, position_t* noeudRef_pos, int destination, void* nodes_to_ignore);
int changeFace(call_t *c, int source, position_t* source_pos, int destination, position_t* dest_pos, int next, position_t* next_pos);
int isPresentIn(void* das, int id);


/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {

   struct entitydata *entitydata = malloc(sizeof(struct entitydata));

   /* set default entity values */
   entitydata->nb_hop = 0;
   entitydata->nb_rx = 0;
   entitydata->face_activation = 0;
   entitydata->face_call = 0;
   entitydata->nb_paquets_too_many_hops = 0;

   position_t* topo_pos = get_topology_area();
   entitydata->scale_postscript = 595.0 / (topo_pos->x + 20);

   /* save entity private data */
   set_entity_private_data(c, entitydata);

  if( access( "topo_graph.ps", F_OK ) != -1 ) {
    remove("topo_graph.ps");
    remove("gabriel_graph.ps");
  }
  return 0;
}

int destroy(call_t *c) {

  struct GGnode *gg = NULL;
  position_t *pos = get_node_position(c->node);
  FILE *graph_GG, *graph;
  position_t* topo_pos = get_topology_area();
  struct entitydata *entitydata = get_entity_private_data(c);


  // Postscript Graphe de Gabriel
  char *name = (char *) malloc(25*sizeof(int));
  sprintf(name, "gabriel_graph.ps");

  if((graph_GG = fopen(name, "a")) == NULL) {
    fprintf(stderr, "GG_sensor: can not open file %s in init()\n", name);
    printf("erreur dans l'ouverture du fichier postscript\n");
    return -1;
  }else {
    fprintf(graph_GG, "15 15 moveto %lf 15 lineto stroke\n", (topo_pos->x* entitydata->scale_postscript)+15);
    fprintf(graph_GG, "15 15 moveto 15 %lf lineto stroke\n", (topo_pos->y* entitydata->scale_postscript)+15);


    fprintf(graph_GG, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x * entitydata->scale_postscript/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript/4.0 + 15.0);
    fprintf(graph_GG, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/4.0 +15.0,topo_pos->y * entitydata->scale_postscript/4.0 +15.0);

    fprintf(graph_GG, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x * entitydata->scale_postscript/2.0 + 15.0, topo_pos->x * entitydata->scale_postscript/2.0 + 15.0);
    fprintf(graph_GG, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/2.0 + 15.0,topo_pos->y * entitydata->scale_postscript/2.0 + 15.0);

    fprintf(graph_GG, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->x * entitydata->scale_postscript * 3.0 /4.0 + 15.0);
    fprintf(graph_GG, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->y * entitydata->scale_postscript * 3.0 /4.0 + 15.0);

    //Sous-axes
	//Under Axis
    fprintf(graph_GG, "0.2 setlinewidth\n");

    fprintf(graph_GG, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x * entitydata->scale_postscript/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript +15);
    fprintf(graph_GG, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/4.0 +15.0, topo_pos->y * entitydata->scale_postscript + 15, topo_pos->y * entitydata->scale_postscript/4.0 +15.0);

    fprintf(graph_GG, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x * entitydata->scale_postscript/2.0 + 15.0, topo_pos->x * entitydata->scale_postscript/2.0 + 15.0, topo_pos->x * entitydata->scale_postscript + 15);
    fprintf(graph_GG, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/2.0 + 15.0, topo_pos->y * entitydata->scale_postscript + 15, topo_pos->y * entitydata->scale_postscript/2.0 +15.0);

    fprintf(graph_GG, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->x * entitydata->scale_postscript * 3.0/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript + 15);
    fprintf(graph_GG, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->y * entitydata->scale_postscript + 15, topo_pos->y * entitydata->scale_postscript * 3.0/4.0 +15.0);

  }
  fclose(graph_GG);

  // Postscript Topologie

  char *name2 = (char *) malloc(15*sizeof(int));
  sprintf(name2, "topo_graph.ps");

  if((graph = fopen(name2, "a")) == NULL) {
    fprintf(stderr, "GG_sensor: can not open file %s in init()\n", name2);
    printf("erreur dans l'ouverture du fichier postscript\n");
    return -1;
  }else {

    // Axes + graduations
    fprintf(graph, "15 15 moveto %lf 15 lineto stroke\n", (topo_pos->x* entitydata->scale_postscript)+15);
    fprintf(graph, "15 15 moveto 15 %lf lineto stroke\n", (topo_pos->y* entitydata->scale_postscript)+15);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x * entitydata->scale_postscript/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript/4.0 + 15.0);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/4.0 +15.0,topo_pos->y * entitydata->scale_postscript/4.0 +15.0);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x * entitydata->scale_postscript/2.0 + 15.0, topo_pos->x * entitydata->scale_postscript/2.0 + 15.0);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/2.0 + 15.0,topo_pos->y * entitydata->scale_postscript/2.0 + 15.0);

    fprintf(graph, "%lf 13 moveto %lf 17 lineto stroke\n", topo_pos->x * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->x * entitydata->scale_postscript * 3.0 /4.0 + 15.0);
    fprintf(graph, "13 %lf moveto 17 %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->y * entitydata->scale_postscript * 3.0 /4.0 + 15.0);

    //Sous-axes
    fprintf(graph, "0.2 setlinewidth\n");

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x * entitydata->scale_postscript/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript +15);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/4.0 +15.0, topo_pos->y * entitydata->scale_postscript + 15, topo_pos->y * entitydata->scale_postscript/4.0 +15.0);

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x * entitydata->scale_postscript/2.0 + 15.0, topo_pos->x * entitydata->scale_postscript/2.0 + 15.0, topo_pos->x * entitydata->scale_postscript + 15);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript/2.0 + 15.0, topo_pos->y * entitydata->scale_postscript + 15, topo_pos->y * entitydata->scale_postscript/2.0 +15.0);

    fprintf(graph, "%lf 15 moveto %lf %lf lineto stroke\n", topo_pos->x * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->x * entitydata->scale_postscript * 3.0/4.0 + 15.0, topo_pos->x * entitydata->scale_postscript + 15);
    fprintf(graph, "15 %lf moveto %lf %lf lineto stroke\n", topo_pos->y * entitydata->scale_postscript * 3.0 /4.0 + 15.0, topo_pos->y * entitydata->scale_postscript + 15, topo_pos->y * entitydata->scale_postscript * 3.0/4.0 +15.0);



  }
  fclose(graph);


   printf("entity rx (routing): %d\n", entitydata->nb_rx);
   printf("entity hops : %d\n", entitydata->nb_hop);
   printf("mean number of hops : %f\n", (double) entitydata->nb_hop / (double) entitydata->nb_rx);
   printf("face activation : %d\n", entitydata->face_activation );
   printf("face calls : %d\n", entitydata->face_call);
   printf("expired packets (too many hops) : %d\n", entitydata->nb_paquets_too_many_hops);
   free(entitydata);

  return 0;
}


/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
  struct nodedata *nodedata = malloc(sizeof(struct nodedata));
  param_t *param;

  /* default values */
  nodedata->neighbors = das_create();
  nodedata->GGlist = das_create();
  nodedata->overhead = -1;
  nodedata->hello_tx = 0;
  nodedata->hello_rx = 0;
  nodedata->data_tx = 0;
  nodedata->data_rx = 0;
  nodedata->data_noroute = 0;
  nodedata->data_hop = 0;
  nodedata->start = 0;
  nodedata->hop = 32;
  nodedata->period = 1000000000;
  nodedata->timeout = 2500000000ull;

  /* get params */
  das_init_traverse(params);
  while ((param = (param_t *) das_traverse(params)) != NULL) {
    if (!strcmp(param->key, "start")) {
      if (get_param_time(param->value, &(nodedata->start))) {
        goto error;
      }
    }
    if (!strcmp(param->key, "period")) {
      if (get_param_time(param->value, &(nodedata->period))) {
        goto error;
      }
    }
    if (!strcmp(param->key, "hop")) {
      if (get_param_integer(param->value, &(nodedata->hop))) {
        goto error;
      }
    }
    if (!strcmp(param->key, "timeout")) {
      if (get_param_time(param->value, &(nodedata->timeout))) {
        goto error;
      }
    }
  }

  set_node_private_data(c, nodedata);
  return 0;

  error:
  free(nodedata);
  return -1;
}


int unsetnode(call_t *c) {
  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL;
  struct gg_neighborhood *neighborhood = NULL, *neighborhood1 = NULL;
  struct entitydata *entitydata = get_entity_private_data(c);

  struct GGnode *gg = NULL;
  position_t *pos = get_node_position(c->node);
  FILE *graph_GG, *graph;

  // Postscript Graphe de Gabriel

  char *name = (char *) malloc(25*sizeof(int));
  sprintf(name, "gabriel_graph.ps");

  if((graph_GG = fopen(name, "a")) == NULL) {
    fprintf(stderr, "GG_sensor: can not open file %s in init()\n", name);
    printf("erreur dans l'ouverture du fichier postscript\n");
    return -1;
  }else {
    if(c->node == 0)
    fprintf(graph_GG, "1 0 0 setrgbcolor %lf %lf 4 0 360 arc fill stroke\n0 0 0 setrgbcolor\n", (pos->x* entitydata->scale_postscript)+15, (pos->y* entitydata->scale_postscript)+15);
    else
    fprintf(graph_GG, "%lf %lf 2.5 0 360 arc fill stroke\n", (pos->x* entitydata->scale_postscript)+15, (pos->y* entitydata->scale_postscript)+15);
    fprintf(graph_GG, "/Times-Roman findfont 6 scalefont setfont newpath %lf %lf moveto (%d) show\n", (pos->x* entitydata->scale_postscript)+18, (pos->y* entitydata->scale_postscript)+18, c->node );
    das_init_traverse(nodedata->GGlist);
    while ((gg = (struct GGnode *) das_traverse(nodedata->GGlist)) != NULL) {
      fprintf(graph_GG, "%lf %lf moveto %lf %lf lineto stroke\n", (pos->x* entitydata->scale_postscript)+15, (pos->y* entitydata->scale_postscript)+15, (gg->GG_pos.x)* entitydata->scale_postscript+15, (gg->GG_pos.y)* entitydata->scale_postscript+15);
    }
  }
  fclose(graph_GG);

  // Postscript Topologie

  char *name2 = (char *) malloc(15*sizeof(int));
  sprintf(name2, "topo_graph.ps");

  if((graph = fopen(name2, "a")) == NULL) {
    fprintf(stderr, "graph: can not open file %s in init()\n", name2);
    printf("erreur dans l'ouverture du fichier postscript\n");
    return -1;
  }else {
    if(c->node == 0)
    fprintf(graph, "1 0 0 setrgbcolor %lf %lf 4  0 360 arc fill stroke\n0 0 0 setrgbcolor\n", (pos->x* entitydata->scale_postscript)+15, (pos->y* entitydata->scale_postscript)+15);
    else
    fprintf(graph, "%lf %lf 2.5 0 360 arc fill stroke\n", (pos->x* entitydata->scale_postscript)+15, (pos->y* entitydata->scale_postscript)+15);
    das_init_traverse(nodedata->neighbors);
    while ((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
      fprintf(graph, "%lf %lf moveto %lf %lf lineto stroke\n", (pos->x* entitydata->scale_postscript)+15, (pos->y* entitydata->scale_postscript)+15, (neighbor->position.x)* entitydata->scale_postscript+15, (neighbor->position.y)* entitydata->scale_postscript+15);
    }
  }
  fclose(graph);


  while ((neighbor = (struct gg_neighbor *) das_pop(nodedata->neighbors)) != NULL) {
    while((neighborhood = (struct gg_neighborhood *) das_pop(neighbor->neighborlist)) != NULL) {
      free(neighborhood);
    }
    das_destroy(neighbor->neighborlist);
    while((neighborhood1 = (struct gg_neighborhood *) das_pop(neighbor->witnessnode)) != NULL) {
      free(neighborhood1);
    }
    das_destroy(neighbor->witnessnode);
    free(neighbor);
  }
  das_destroy(nodedata->neighbors);

  struct gg_neighbor *gg_neighbor;
  while ((gg_neighbor = (struct gg_neighbor*) das_pop(nodedata->GGlist)) != NULL) {
    free(gg_neighbor);
  }
  das_destroy(nodedata->GGlist);
  free(nodedata);
  return 0;
}


/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
  struct nodedata *nodedata = get_node_private_data(c);
  call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};

  /* get mac header overhead */
  nodedata->overhead = GET_HEADER_SIZE(&c0);

  /*  hello packet */
  if (nodedata->period > 0) {
    uint64_t start = get_time() + nodedata->start + get_random_double() * nodedata->period;
    get_random_double();
    scheduler_add_callback(start, c, hello_callback, NULL);
  }

  return 0;
}

int ioctl(call_t *c, int option, void *in, void **out) {
  return 0;
}


/* ************************************************** */
/* ************************************************** */
struct gg_neighbor* get_nexthop_greedy(call_t *c, struct routing_header *header) {

  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL, *n_hop = NULL;
  uint64_t clock = get_time();
  position_t *dst = &(header->dst_pos);
  double dist = distance(get_node_position(c->node), dst);
  double d = dist;


  /* parse neighbors */
  das_init_traverse(nodedata->neighbors);
  while ((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {

    if(neighbor->id == header->dst)
    return neighbor;

    if ((nodedata->timeout > 0) && (clock - neighbor->time) >= nodedata->timeout ) {
      continue;
    }

    /* choose next hop */
    if ((d = distance(&(neighbor->position), dst)) < dist) {
      dist = d;
      n_hop = neighbor;
    }
  }

  if(n_hop == NULL)
  {
    printf("%d - Greedy impossible\n", c->node);

  }

  return n_hop;
}

/* ************************************************** */
/* ************************************************** */

struct gg_neighbor* get_nexthop_FACE(call_t *c, struct routing_header *header) {

  //return NULL;
  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *n_hop = NULL;
  position_t *dst = &(header->dst_pos), *my_pos = get_node_position(c->node);

  if(header->face_activated == 1)
  {
    void* nodes_to_ignore = das_create();

    n_hop = nextOnFace(c, c->node, my_pos, header->sender, get_node_position(header->sender), header->dst, NULL);
    printf("%d - %d FACE activated source %d previous %d dest %d \n",c->node, n_hop->id, header->src, header->sender, header->dst);
    das_insert(nodes_to_ignore, &(n_hop->id));

    while (changeFace(c, header->face_initiator, &(header->face_initiator_pos), header->dst, dst, n_hop->id, get_node_position(n_hop->id)) == 1){
      n_hop = nextOnFace(c, c->node, my_pos, header->sender, get_node_position(header->sender), header->dst, nodes_to_ignore);
      das_insert(nodes_to_ignore, &(n_hop->id));
    }

  }else{
    n_hop = nextOnFace(c, c->node, my_pos, header->dst, dst, header->dst, NULL);
    printf("%d - %d premiere FACE to node %d localized %lf - %lf\n",c->node, n_hop->id, header->dst, dst->x, dst->y);
  }

  return n_hop;
}


/* ************************************************** */
/* ************************************************** */

struct gg_neighbor* nextOnFace(call_t *c, int depart, position_t* depart_pos, int noeudRef, position_t* noeudRef_pos, int destination, void* nodes_to_ignore)
{
  //Retourne le prochain noeud dans le Face Routing selon si on est une source (type = 0, noeudRef= destination)  ou un noeud intermédiaire sur la face (type = 1, noeudRef = noeud précédent)
  //On applique la règle de la main droite
  //Returns the next node in the face routing depending on whether one is a source (type = 0, nodeRef = destination) or an intermediate node on the side (type = 1, nodeRef = previous node)
  //We apply the right-hand rule
  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor * neighbor = NULL, *node_ref = malloc(sizeof(struct gg_neighbor)), *node_max = malloc(sizeof(struct gg_neighbor)), *node_depart = malloc(sizeof(struct gg_neighbor)), *node_winner = malloc(sizeof(struct gg_neighbor));
  double  angle, angle_min, angle_max, vect, d1, d2;
  int winner, OK, max;

  angle_min = 180.0;
  angle_max = 0.0;
  winner = depart;
  max = depart;

  //  printf("NextOnFace %d %d\n", depart, noeudRef);

  d1 = distance(depart_pos, noeudRef_pos);
  //int* tmp = malloc(sizeof(int));

  das_init_traverse(nodedata->GGlist);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->GGlist)) != NULL) {


    if(nodes_to_ignore == NULL || isPresentIn(nodes_to_ignore, neighbor->id) == 0){

      if(neighbor->id == destination)
      {
        free(node_ref);
        free(node_depart);
        free(node_max);
        free(node_winner);
        return neighbor;
      }else
      {
        if(neighbor->id != noeudRef){

          // printf("%d - voisin %d source %d \n", c->node, neighbor->id, depart);

          position_t* temp_pos = get_node_position(neighbor->id);
          d2 = distance(depart_pos, temp_pos);
          angle = acos((double)((depart_pos->x - noeudRef_pos->x)*(depart_pos->x - temp_pos->x) + (depart_pos->y - noeudRef_pos->y)*(depart_pos->y -temp_pos->y)) /((double)((double)d1*(double)d2)));

          vect = (double)((temp_pos->x - depart_pos->x)*(noeudRef_pos->y - depart_pos->y)) -  (double)((temp_pos->y - depart_pos->y)*(noeudRef_pos->x -depart_pos->x));


          //			  printf("\tNoeud ref : %lf %lf\n", noeudRef_pos->x, noeudRef_pos->y);
          //			  printf("\tDepart : %lf %lf\n", depart_pos->x, depart_pos->y);
          //			  printf("\tTemp : %lf %lf\n", temp_pos->x, temp_pos->y);
          //			  printf("\tAngle : %lf\n", angle);
          //			  printf("\tVect : %lf\n", vect);

          //vect = 3e composante du produit vectoriel  (DepartTemp)^(DepartRef)
		  //vect = third component of the vector product (startTemp)^(startRef)
          OK = 0;

          if (vect >=0) {
            // 	printf("\t vect >= 0 ou type == 1 ");
            OK = 1;
          }
          //Pour choisir de partir du bon côté (main droite), il faut que la 3e coordonnée du produit vectoriel (SourceNext)^(SourceDest) soit positive (donc vect >= 0)
		  //To choose from the right side (right hand), we need the third cooordinate of the vector product (SourceNext)^(SourceDest) or positive (thus vect >= 0)


          if ((OK==1) && (angle <= angle_min)){
            //    printf("\t angle < angle_min ");
            angle_min = angle;
            winner = neighbor->id;
            *node_winner = *neighbor;
          }
          if ((OK==0) && (angle >= angle_max))
          {
            //     printf("\t OK=0 & angle > angle_max ");
            angle_max = angle;
            max = neighbor->id;
            *node_max = *neighbor;
          }

          //printf("\n");
        }else{
          *node_ref = *neighbor;
        }

        if(neighbor->id == depart)
        *node_depart = *neighbor;
      }
    }
  }

  if (winner == depart) //on prend la face extérieure : we take the outside
  {
    winner = max;
    *node_winner = *node_max;
    //printf(" winner = depart1 %d\n", max);
  }

  if (winner == depart) //addr est une feuille dans le graphe planaire : addr is a sheet in the planar graph
  {
    winner = noeudRef; //on fait demi tour sur la face extérieure : we turned around on the outside
    *node_winner = *node_ref;
    //printf(" winner = depart2 %d\n", noeudRef);
  }


  //printf(" winner %d\n", winner);



  free(node_ref);
  free(node_depart);
  free(node_max);
  return(node_winner);

}//fin NextOnFace



/* ************************************************** */
/* ************************************************** */


int changeFace(call_t *c, int source, position_t* source_pos, int destination, position_t* dest_pos, int next, position_t* next_pos)
{

  printf("call changeFace -> Source %d - Dest %d - Courant %d - Next %d\n", source, destination, c->node, next);
  // Courant = c->node
  double vect1, vect2, b1, b2, x, y, d;
  int OK = 1;

  if ((next == destination) || (c->node == source) )  return 0;

  position_t* my_pos = get_node_position(c->node);

  //On change de Face si l'angle entre les vecteurs SourceCourant et SourceDestination est de signe différent de l'angle entre les vecteurs SourceNext et SourceDestination
  //We change the face if the angle between the vectors SourceCurrent and SourceDestination is a different sign of the angle between the vectors SourceNext and SourceDestination
  vect1 = ((my_pos->x - source_pos->x) * (dest_pos->y - source_pos->y)) -  ((my_pos->y - source_pos->y) * (dest_pos->x - source_pos->x));
  //vect1 = produit vectoriel SC^SD

  //  if (vect1 == 0.0) return 0; //source, current node and destination nodes are aligned.


  vect2 = ((next_pos->x - source_pos->x) * (dest_pos->y - source_pos->y)) -  ((next_pos->y - source_pos->y) * (dest_pos->x - source_pos->x));
  //vect2 = produit vectoriel SN^SD

  if ( ((vect1<=0) && (vect2<=0)) || ((vect1>=0) && (vect2>=0))) OK = 0;


  if (OK == 1)
  //les droites CourantNext et SourceDestination se coupent. Il reste à déterminer si l'intersection est sur le segment SourceDestination
  //The Line CurrentNext and SourceDestination intersect. It isn't determined whether the intersection is on the segment SourceDestination
  {
    //      printf("intersection entre %d(%f, %f) %d(%f, %f))\n", courant, my_pos->x, my_pos->y, next, next_pos->x, next_pos->y);
    OK = 0;

    vect1 = (dest_pos->y - source_pos->y) / (double)(dest_pos->x - source_pos->x);
    vect2 = (next_pos->y - my_pos->y) / (double)(next_pos->x - my_pos->x);
    b1 = source_pos->y - vect1 * source_pos->x;
    b2 = my_pos->y - vect2 * my_pos->x ;

    x = (b2-b1)/(double)(vect1-vect2);
    y = (double)(vect1*x)+b1;


    d = hypot((source_pos->x - x), (source_pos->y - y)) +  hypot((dest_pos->x - x), (dest_pos->y - y));

    //printf("vect1 %lf - vext2 %lf - b1 %lf - b2 %lf - d %lf\n",vect1, vect2, b1, b2, d);

    //  printf("I(%f, %f), d = %f, %f\n", x, y, d, topology_distance(source, destination));
    if (((int)(distance(source_pos, dest_pos) * pow(10,6))) == (int)(d * pow(10,6))) //le point d'intersection est sur le segment, on change de face
    OK = 1;


  }

  //  printf("OK = %d\n", OK);
  return(OK);


}//fin changeFace



/* ************************************************** */
/* ************************************************** */

void add_neighbor(call_t *c, packet_t *packet) {

  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL;
  struct gg_neighborhood *neighborhood = NULL;
  struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);
  struct hello *hello = (struct hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header));
  position_t *pos = get_node_position(c->node);
  int i = 0, size = 0, temoin = 0;

  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    if(neighbor->id == header->src) {
      neighbor->position.x = header->src_pos.x;
      neighbor->position.y = header->src_pos.y;
      neighbor->position.z = header->src_pos.z;
      if(check_in_neighbor(c, packet) == 1) {
        neighbor->mark = 0;
        neighbor->link_type = SYMETRIK;
      }else {
        neighbor->mark = 0;
        neighbor->link_type = ASYMETRIK;
      }
      neighbor->time = get_time();
      neighbor->middle_pos.x = (pos->x + header->src_pos.x)/2;
      neighbor->middle_pos.y = (pos->y + header->src_pos.y)/2;
      neighbor->middle_pos.z = (pos->z + header->src_pos.z)/2;
      neighbor->distance = distance(pos, &(header->src_pos));
      neighbor->angle = 0;
      size = hello->nbr;
      if(size > 0) {
        for(i = 0;  i < size; i++) {
          struct gg_neighbor_hello   *neighbors = (struct gg_neighbor_hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header) +
          sizeof(struct hello) + i*sizeof(struct gg_neighbor_hello));
          if(neighbors->neighbors_id != c->node) {
            temoin = 0;
            das_init_traverse(neighbor->neighborlist);
            while((neighborhood = (struct gg_neighborhood *) das_traverse(neighbor->neighborlist)) != NULL) {
              if(neighborhood->neighbor_idN ==  neighbors->neighbors_id) {
                neighborhood->NGhood_pos.x = neighbors->neg_pos.x;
                neighborhood->NGhood_pos.y = neighbors->neg_pos.y;
                neighborhood->NGhood_pos.z = neighbors->neg_pos.z;
                neighborhood->link_typeN = neighbors->neighbor_type;
                neighborhood->node_ttl = get_time();
                neighborhood->dist = distance(&(neighbor->position), &(neighbors->neg_pos));
                neighborhood->dist_from_C = distance(pos, &(neighbors->neg_pos));
                temoin = 1;
              }
            }
            if(temoin == 0) {
              neighborhood = (struct gg_neighborhood *) malloc(sizeof(struct gg_neighborhood));
              if(neighborhood == NULL) { return;}
              neighborhood->neighbor_idN = neighbors->neighbors_id;
              neighborhood->NGhood_pos.x = neighbors->neg_pos.x;
              neighborhood->NGhood_pos.y = neighbors->neg_pos.y;
              neighborhood->NGhood_pos.z = neighbors->neg_pos.z;
              neighborhood->link_typeN = neighbors->neighbor_type;
              neighborhood->node_ttl = get_time();
              neighborhood->dist = distance(&(neighbor->position), &(neighbors->neg_pos));
              neighborhood->dist_from_C = distance(pos, &(neighbors->neg_pos));
              das_insert(neighbor->neighborlist, (void *) neighborhood);
            }
          }
        }
      }
      return;
    }
  }

  neighbor = (struct gg_neighbor *) malloc(sizeof(struct gg_neighbor));
  if(neighbor == NULL) {
    return;
  }

  neighbor->id = header->src;
  neighbor->position.x = header->src_pos.x;
  neighbor->position.y = header->src_pos.y;
  neighbor->position.z = header->src_pos.z;

  if(check_in_neighbor(c, packet) == 1) {
    neighbor->mark = 0;
    neighbor->link_type = SYMETRIK;
  }
  else {
    neighbor->mark = 0;
    neighbor->link_type = ASYMETRIK;
  }

  neighbor->time = get_time();
  neighbor->middle_pos.x = (pos->x + header->src_pos.x)/2;
  neighbor->middle_pos.y = (pos->y + header->src_pos.y)/2;
  neighbor->middle_pos.z = (pos->z + header->src_pos.z)/2;
  neighbor->distance = distance(pos, &(header->src_pos));
  neighbor->angle = 0;
  neighbor->neighborlist = das_create();
  neighbor->witnessnode = das_create();
  size = hello->nbr;
  if(size > 0) {
    for(i = 0; i < size; i++) {
      struct gg_neighbor_hello *neighbors = (struct gg_neighbor_hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header) + sizeof(struct hello)
      + i*sizeof(struct gg_neighbor_hello));
      if(neighbors->neighbors_id != c->node) {
        neighborhood = (struct gg_neighborhood *) malloc(sizeof(struct gg_neighborhood));
        if(neighborhood == NULL) {
          return;
        }
        neighborhood->neighbor_idN = neighbors->neighbors_id;
        neighborhood->NGhood_pos.x = neighbors->neg_pos.x;
        neighborhood->NGhood_pos.y = neighbors->neg_pos.y;
        neighborhood->NGhood_pos.z = neighbors->neg_pos.z;
        neighborhood->link_typeN = neighbors->neighbor_type;
        neighborhood->node_ttl = get_time();
        neighborhood->dist = distance(&(neighbor->position), &(neighbors->neg_pos));
        neighborhood->dist_from_C = distance(pos, &(neighbors->neg_pos));
        das_insert(neighbor->neighborlist, (void *) neighborhood);
      }
    }
  }
  das_insert(nodedata->neighbors, (void *) neighbor);
  return;
}


/* *********************************************** */
/* *********************************************** */
int search_node(call_t *c, int identity) {

  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL, *witness = NULL;
  int found = 0;

  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    if(neighbor->id == identity) {
      witness = neighbor;
      found = 1;
      break;
    }
  }
  if((found == 1) && (witness->link_type == SYMETRIK)) {
    return 1;
  }

  return 0;
}





/* *********************************************** */
/* *********************************************** */
int check_in_neighbor(call_t *c, packet_t *packet) {

  struct nodedata  *nodedata = get_node_private_data(c);
  struct hello *hello = (struct hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header));
  int size = hello->nbr, i;

  if(size > 0) {
    for(i = 0; i < size; i++) {
      struct gg_neighbor_hello   *neighbors = (struct gg_neighbor_hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header) + sizeof(struct hello)
      + i*sizeof(struct gg_neighbor_hello));
      if(c->node == neighbors->neighbors_id) {
        return 1;
      }
    }
  }
  return 0;
}




/* ************************************************** */
/* ************************************************** */
int set_header(call_t *c, packet_t *packet, destination_t *dst) {

  struct nodedata *nodedata = get_node_private_data(c);
  destination_t destination;
  struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);

  header->dst = dst->id;
  header->dst_pos.x = dst->position.x;
  header->dst_pos.y = dst->position.y;
  header->dst_pos.z = dst->position.z;
  header->src = c->node;


  struct gg_neighbor *n_hop = get_nexthop_greedy(c, header);
  if(n_hop == NULL)
  {
    struct entitydata *entitydata = get_entity_private_data(c);;
    n_hop = get_nexthop_FACE(c, header);
    header->face_initiator = c->node;
    header->face_initiator_pos = *(get_node_position(c->node));
    header->face_activated = 1;
    entitydata->face_activation++;
    entitydata->face_call++;
    set_entity_private_data(c, entitydata);
  }else
  {
    header->face_initiator = -1;
    header->face_activated = 0;
  }

  call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};

  /* if no route, return -1 */
  if (dst->id != BROADCAST_ADDR && n_hop == NULL) {
    nodedata->data_noroute++;
    return -1;
  }
  else if (dst->id == BROADCAST_ADDR) {
    n_hop->id = BROADCAST_ADDR;
  }

  /* set routing header */
  header->src_pos.x = get_node_position(c->node)->x;
  header->src_pos.y = get_node_position(c->node)->y;
  header->src_pos.z = get_node_position(c->node)->z;
  header->sender = c->node;
  header->type = DATA_PACKET;
  header->hop = nodedata->hop;

  // Pour ne pas dépasser le max dans l'incrémentation des numseq
  // Order not to exceed the max in incrementing numseq
  header->numseq = rand()%(INT32_MAX - nodedata->hop - 1);
  header->next_node = n_hop->id;
  header->next_node_pos.x = get_node_position(n_hop->id)->x;
  header->next_node_pos.y = get_node_position(n_hop->id)->y;
  header->next_node_pos.z = get_node_position(n_hop->id)->z;

  /* Set mac header */
  destination.id = BROADCAST_ADDR;
  destination.position.x = -1;
  destination.position.y = -1;
  destination.position.z = -1;
  return SET_HEADER(&c0, packet, &destination);
}

int get_header_size(call_t *c) {
  struct nodedata *nodedata = get_node_private_data(c);

  if (nodedata->overhead == -1) {
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    nodedata->overhead = GET_HEADER_SIZE(&c0);
  }

  return nodedata->overhead + sizeof(struct routing_header);
}

int get_header_real_size(call_t *c) {
  struct nodedata *nodedata = get_node_private_data(c);

  if (nodedata->overhead == -1) {
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    nodedata->overhead = GET_HEADER_REAL_SIZE(&c0);
  }

  return nodedata->overhead + sizeof(struct routing_header);
}


/* ************************************************** */
/* ************************************************** */
int neighbor_timeout(void *data, void *arg) {

  struct gg_neighbor *neighbor = (struct gg_neighbor *) data;
  call_t *c = (call_t *) arg;
  struct nodedata *nodedata = get_node_private_data(c);
  if ((get_time() - neighbor->time) >= nodedata->timeout) {
    return 1;
  }
  return 0;
}
/* *********************************************** */
/* *********************************************** */

int GG_timeout(void *data, void *args) {

  struct GGnode *gg = (struct GGnode *) data;
  call_t *c = (call_t *) args;
  struct nodedata *nodedata = get_node_private_data(c);

  if((get_time() - gg->timeGG) >= nodedata->timeout) {
    return 1;
  }
  return 0;
}

/* *********************************************** */
/* *********************************************** */
int neighborhood_timeout(void *data, void *args) {

  struct gg_neighborhood *neighborhood = (struct gg_neighborhood *) data;
  call_t *c = (call_t *) args;
  struct nodedata *nodedata = get_node_private_data(c);

  if((get_time() - neighborhood->node_ttl) >= ((nodedata->timeout)/2)) {
    return 1;
  }
  return 0;
}
/* ************************************************** */
/* ************************************************** */

int hello_callback(call_t *c, void *args) {

  struct nodedata *nodedata = get_node_private_data(c);
  call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
  destination_t destination = {BROADCAST_ADDR, {-1, -1, -1}};
  int size = das_getsize(nodedata->neighbors);


  packet_t *packet = packet_alloc(c, nodedata->overhead + sizeof(struct routing_header) + sizeof(struct hello) +  size * sizeof(struct gg_neighbor_hello));
  struct routing_header *header = (struct routing_header*) (packet->data + nodedata->overhead);
  struct hello *hello = (struct hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header));
  struct gg_neighbor *neighbor = NULL;


  /* set mac header */
  if (SET_HEADER(&c0, packet, &destination) == -1) {
    packet_dealloc(packet);
    return -1;
  }

  /* set routing header */
  header->dst = BROADCAST_ADDR;
  header->dst_pos.x = -1;
  header->dst_pos.y = -1;
  header->dst_pos.z = -1;
  header->src = c->node;
  header->src_pos.x = get_node_position(c->node)->x;
  header->src_pos.y = get_node_position(c->node)->y;
  header->src_pos.z = get_node_position(c->node)->z;
  header->type = HELLO_PACKET;
  header->hop = 1;
  header->sender = c->node;
  header->next_node = -1;
  header->next_node_pos.x = -1;
  header->next_node_pos.y = -1;
  header->next_node_pos.z = -1;


  int i = 0;

  /* check neighbors timeout  */
  das_selective_delete(nodedata->neighbors, neighbor_timeout, (void *) c);
  das_selective_delete(nodedata->GGlist, GG_timeout, (void *) c);
  onehoplist_time(c, NULL);

  hello->nbr = size;
  if(size > 0) {
    struct gg_neighbor_hello *neighbors = (struct gg_neighbor_hello *) (packet->data + nodedata->overhead + sizeof(struct routing_header) + sizeof(struct hello));
    das_init_traverse(nodedata->neighbors);
    while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
      neighbors[i].neighbors_id    = neighbor->id;
      neighbors[i].neg_pos.x       = neighbor->position.x;
      neighbors[i].neg_pos.y       = neighbor->position.y;
      neighbors[i].neg_pos.z       = neighbor->position.z;
      neighbors[i].neighbor_type   = neighbor->link_type;
      i++;
    }
  }


  /* send hello */
  TX(&c0, packet);
  nodedata->hello_tx++;

  /* schedules hello */
  scheduler_add_callback(get_time() + nodedata->period, c, hello_callback, NULL);
  return 0;



}

/* ************************************************** */
/* ************************************************** */

void onehoplist_time(call_t *c , void *args) {
  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL;

  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    das_selective_delete(neighbor->neighborlist, neighborhood_timeout, (void *) c);
    das_selective_delete(neighbor->witnessnode, neighborhood_timeout, (void *) c);
  }
  return;
}

/* ************************************************** */
/* ************************************************** */

void setwitness_node(call_t *c, void * args) {

  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL, *neighbor1 = NULL;
  struct gg_neighborhood *neighborhood = NULL, *neighborhood1 = NULL;
  position_t *pos = get_node_position(c->node);
  int i, *tab, size = das_getsize(nodedata->neighbors), temoin;

  tab = malloc(size*sizeof(int));
  for(i = 0; i < size; i++) {
    tab[i] = 0;
  }
  i = 0;
  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    tab[i] = neighbor->id;
    i++;
  }
  for(i = 0; i < size; i++) {
    neighbor1 = getnode(c, tab[i]);
    das_init_traverse(neighbor1->neighborlist);
    while((neighborhood = (struct gg_neighborhood *) das_traverse(neighbor1->neighborlist)) != NULL) {
      if((search_node(c, neighborhood->neighbor_idN) == 1)) {
        temoin = 0;
        das_init_traverse(neighbor1->witnessnode);
        while((neighborhood1 = (struct gg_neighborhood *) das_traverse(neighbor1->witnessnode)) != NULL) {
          if(neighborhood1->neighbor_idN == neighborhood->neighbor_idN) {
            neighborhood1->NGhood_pos.x = neighborhood->NGhood_pos.x;
            neighborhood1->NGhood_pos.y = neighborhood->NGhood_pos.y;
            neighborhood1->NGhood_pos.z = neighborhood->NGhood_pos.z;
            neighborhood1->link_typeN   = neighborhood->link_typeN;
            neighborhood1->node_ttl     = get_time();
            neighborhood1->dist         = neighborhood->dist;
            neighborhood1->dist_from_C  = distance(pos, &(neighborhood->NGhood_pos));
            temoin = 1;
            break;
          }
        }
        if(temoin == 0) {
          neighborhood1 = (struct gg_neighborhood *) malloc(sizeof(struct gg_neighborhood));
          if(neighborhood1 == NULL) {
            return;
          }
          neighborhood1->neighbor_idN = neighborhood->neighbor_idN;
          neighborhood1->NGhood_pos.x = neighborhood->NGhood_pos.x;
          neighborhood1->NGhood_pos.y = neighborhood->NGhood_pos.y;
          neighborhood1->NGhood_pos.z = neighborhood->NGhood_pos.z;
          neighborhood1->link_typeN   = neighborhood->link_typeN;
          neighborhood1->node_ttl     = get_time();
          neighborhood1->dist         = neighborhood->dist;
          neighborhood1->dist_from_C  = distance(pos, &(neighborhood->NGhood_pos));
          das_insert(neighbor1->witnessnode, (void *) neighborhood1);
        }
      }
    }
  }
  free(tab);
  return;
}

/* *********************************************** */
/* *********************************************** */
void GG_Neighbor(call_t *c, void *args) {
  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL, *neighbor1 = NULL;
  struct gg_neighborhood *neighborhood = NULL;
  int  *tab, i, size = 0;
  position_t pos;

  size = das_getsize(nodedata->neighbors);
  tab = (int *) malloc(size*sizeof(int));
  i = 0;
  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    neighbor->mark = 1;
    tab[i] =  neighbor->id;
    i++;
  }
  neighbor = NULL;
  for(i = 0; i  < size; i++) {
    neighbor1 = getnode(c, tab[i]);
    pos.x = neighbor1->middle_pos.x;
    pos.y = neighbor1->middle_pos.y;
    pos.z = neighbor1->middle_pos.z;
    if(das_getsize(neighbor1->witnessnode) > 0) {
      das_init_traverse(neighbor1->witnessnode);
      while((neighborhood = (struct gg_neighborhood *) das_traverse(neighbor1->witnessnode)) != NULL) {
        if(search_node(c, neighborhood->neighbor_idN) == 1) {
          if(distance(&pos, &(neighborhood->NGhood_pos)) < (neighbor1->distance/2.0)) {
            set_mark(c, neighbor1->id);
            //break;
          }
        }
      }
    }
  }
  free(tab);
  return;
}
/* *********************************************** */
/* *********************************************** */
void updateGG(call_t *c, void *args) {

  struct nodedata *nodedata = get_node_private_data(c);
  struct GGnode *gg = NULL;
  struct gg_neighbor *neighbor = NULL;
  int temoin = 0;

  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    temoin = 0;
    if((neighbor->link_type == SYMETRIK) && (getnode(c, neighbor->id)->mark == 1)) {

      das_init_traverse(nodedata->GGlist);

      while((gg = (struct GGnode *) das_traverse(nodedata->GGlist)) != NULL) {
        if(gg->GG_id == neighbor->id) {
          gg->GG_pos.x = neighbor->position.x;
          gg->GG_pos.y = neighbor->position.y;
          gg->GG_pos.z = neighbor->position.z;
          gg->timeGG = neighbor->time;
          temoin = 1;
          break;
        }
      }

      if(temoin == 0) {
        gg = (struct GGnode *) malloc(sizeof(struct GGnode));
        if(gg == NULL) {
          return;
        }
        gg->GG_id = neighbor->id;
        gg->GG_pos.x = neighbor->position.x;
        gg->GG_pos.y = neighbor->position.y;
        gg->GG_pos.z = neighbor->position.z;
        gg->timeGG = neighbor->time;
        gg->angle = 0.0;
        printf("%d insert %d - temoin %d - link %d - mark %d \n", c->node, neighbor->id, temoin, neighbor->link_type, getnode(c, neighbor->id)->mark);
        das_insert(nodedata->GGlist, (void *) gg);
        break;
      }
    }

  }

  return;
}


/* *********************************************** */
/* *********************************************** */
void set_mark(call_t *c, int node) {
  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL;

  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    if(neighbor->id == node) {
      neighbor->mark = -1;

      return;
    }
  }
  return;
}
/* *********************************************** */
/* *********************************************** */
struct gg_neighbor* getnode(call_t *c, int ggid) {

  struct nodedata *nodedata = get_node_private_data(c);
  struct gg_neighbor *neighbor = NULL, *nodeGG = NULL;

  das_init_traverse(nodedata->neighbors);
  while((neighbor = (struct gg_neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
    if(neighbor->id == ggid) {
      nodeGG = neighbor;
      break;
    }
  }
  if(nodeGG != NULL) {
    return nodeGG;
  }
  return NULL;
}

/* *********************************************** */
/* *********************************************** */
double Angular(position_t *position, position_t *pf_pos, position_t *dst_pos) {

  return (acos((pow(distance(pf_pos, position), 2) + pow(distance(pf_pos, dst_pos), 2) - pow(distance(position, dst_pos), 2)) / (2*distance(pf_pos, position)*distance(pf_pos, dst_pos)))*180.0 / M_PI);
}
/* *********************************************** */
/* *********************************************** */
struct GGnode *Get_GGNode(call_t *c, position_t *dst_pos) {

  struct nodedata *nodedata = get_node_private_data(c);
  position_t *pos = get_node_position(c->node);
  struct GGnode *gg = NULL, *found = NULL;
  double min = 360.0;


  das_init_traverse(nodedata->GGlist);
  while((gg = (struct GGnode *) das_traverse(nodedata->GGlist)) != NULL) {
    gg->angle = Angular(&gg->GG_pos, pos, dst_pos);
    if(gg->angle < min) {
      min = gg->angle;
      found = gg;
    }
  }
  return found;


}

/* ************************************************** */
/* ************************************************** */

void tx(call_t *c, packet_t *packet) {
  struct nodedata *nodedata = get_node_private_data(c);
  call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
  struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);

  printf("TX %d - j'envoie a %d\n", c->node, header->next_node);


  nodedata->data_tx++;
  TX(&c0, packet);
}


/* ************************************************** */
/* ************************************************** */
void forward(call_t *c, packet_t *packet) {
  struct nodedata *nodedata = get_node_private_data(c);
  call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
  struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);
  struct gg_neighbor *n_hop = NULL;
  struct entitydata *entitydata = get_entity_private_data(c);

  if(header->face_activated == 0)
     n_hop = get_nexthop_greedy(c, header);
  else
  {
    // On utilise Greedy que quand on s'est rapproché de la destination
    float d_srcFACE_dst, d_curr_dst;
    d_curr_dst = distance(get_node_position(c->node), &(header->dst_pos));
    d_srcFACE_dst = distance(&(header->face_initiator_pos), &(header->dst_pos));

    if(d_curr_dst <= d_srcFACE_dst && header->face_initiator != c->node )
    n_hop = get_nexthop_greedy(c, header);
  }
  if(n_hop == NULL)
  {
    n_hop = get_nexthop_FACE(c, header);
    entitydata->face_call++;

    if(header->face_activated == 0){
      entitydata->face_activation++;
      header->face_initiator = c->node;
      header->face_initiator_pos = *(get_node_position(c->node));
    }
    header->face_activated = 1;
    set_entity_private_data(c, entitydata);
    printf("%d - FACE neighbor : %d\n", c->node, n_hop->id);
  }else
  {
    header->face_activated = 0;
  }
  destination_t destination;

  /* delivers packet to application layer */
  if (n_hop == NULL) {
    header->hop = 0;
    printf("No neighbors\n");
  }

  /* update hop count */
  header->hop--;
  if (header->hop < 1) {
     if(header == 0){
        entitydata->nb_paquets_too_many_hops++;
        set_entity_private_data(c, entitydata);
     }
    nodedata->data_hop++;
    packet_dealloc(packet);
    return;
  }

  header->numseq++;
  header->next_node = n_hop->id;
  header->next_node_pos.x = get_node_position(n_hop->id)->x;
  header->next_node_pos.y = get_node_position(n_hop->id)->y;
  header->next_node_pos.z = get_node_position(n_hop->id)->z;

  /* Set mac header */
  destination.id = BROADCAST_ADDR;
  destination.position.x = -1;
  destination.position.y = -1;
  destination.position.z = -1;

  printf("I am %d and I forward packet from source %d  next hop %d sender %d destination %d \n", c->node, header->src, n_hop->id, header->sender, header->dst);
  header->sender = c->node;


  if (SET_HEADER(&c0, packet, &destination) == -1) {
    packet_dealloc(packet);
    return;
  }

  /* forwarding packet */
  nodedata->data_tx++;
  TX(&c0, packet);
}


/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {

  struct nodedata *nodedata = get_node_private_data(c);
  array_t *up = get_entity_bindings_up(c);
  int i = up->size;
  struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);

  switch(header->type) {
    case HELLO_PACKET:
    nodedata->hello_rx++;

    add_neighbor(c, packet);
    setwitness_node(c, NULL);
    GG_Neighbor(c, NULL);
    updateGG(c, NULL);

    packet_dealloc(packet);
    break;

    case DATA_PACKET :

    if(header->next_node != c->node && header->dst != c->node)
    return;

    if(header->next_node == c->node && header->dst != c->node){
      forward(c, packet);
      return;
    }

    nodedata->data_rx++;
    struct entitydata *entitydata = get_entity_private_data(c);
    entitydata->nb_rx++;
    entitydata->nb_hop += nodedata->hop - header->hop + 1;
    set_entity_private_data(c, entitydata);

    printf("Direct to the sink -  from %d and the source is %d\n",  header->sender, header->src);

    //	if ((header->dst != BROADCAST_ADDR) && (header->dst != c->node) ) {
    //            forward(c, packet);
    //            return;
    //        }

    while (i--) {
      call_t c_up = {up->elts[i], c->node, c->entity};
      packet_t *packet_up;
      if (i > 0) {
        packet_up = packet_clone(packet);
      } else {
        packet_up = packet;
      }
      RX(&c_up, packet_up);
    }
    break;

    default :
    break;
  }

  return;
}

/* ************************************************** */
/* ************************************************** */
int compare_headers(struct routing_header *first, struct routing_header *second)
{
  if(first->dst == second->dst && first->src == second->src && first->hop == second->hop+1 && second->next_node == first->sender && first->numseq + 1 == second->numseq){
    return 0;
  }else{
    return 1;
  }
}
/* ************************************************** */
/* ************************************************** */

int isPresentIn(void* das, int id)
{
  das_init_traverse(das);
  int* tmp = NULL;

  while ((tmp = (int *) das_traverse(das)) != NULL)
  {
    if(*tmp == id)
    return 1;
  }

  return 0;
}


/* ************************************************** */
/* ************************************************** */

routing_methods_t methods = {rx,
  tx,
  set_header,
  get_header_size,
  get_header_real_size};
