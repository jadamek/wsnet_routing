#include <stdio.h>
#include <time.h>
#include <include/modelutils.h>

/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "Send one data message",
    "Tony Ducrocq",
    "0.1",
    MODELTYPE_APPLICATION, 
    {NULL, 0}
};

/* ************************************************** */
/* ************************************************** */
struct _rsend_header {
   int id;
};

struct nodedata {
  int *overhead;
  int size;
  uint64_t period;
  double probability;
  nodeid_t destination;
  int geographical;

  /* stats */
  int data_tx;
  int data_rx;
};

struct entitydata {
  int data_tx;
  int data_rx;
};

/* ************************************************** */
/* ************************************************** */
int callmeback(call_t *c, void *args);
int tx(call_t *c, void *args);


/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) { 
  struct entitydata *entitydata = malloc(sizeof(struct entitydata));
  
  /* set default entity values */
  entitydata->data_tx = 0;
  entitydata->data_rx = 0;

  /* init random number generator*/
  srand( time(NULL) );

  /* save entity private data */
  set_entity_private_data(c, entitydata);
  return 0;
}

int destroy(call_t *c) {
  struct entitydata *entitydata = get_entity_private_data(c);;

  printf("entity tx : %d\n", entitydata->data_tx);
  printf("entity rx : %d\n", entitydata->data_rx);
  
  printf("delivery:%f\n", (double)entitydata->data_rx / (double)entitydata->data_tx);

  free(entitydata);
  return 0;
}

/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
  struct nodedata *nodedata = malloc(sizeof(struct nodedata));
  int i = get_entity_links_down_nbr(c);
  param_t *param;
  
  /* default values */
  nodedata->size         = 10;
  nodedata->period       = 2000000000ull;
  nodedata->probability  = 1;
  nodedata->data_tx      = 0;
  nodedata->data_rx      = 0;
  nodedata->destination  = -1;
  nodedata->geographical = 0;

  /* get parameters */
  das_init_traverse(params);
  while ((param = (param_t *) das_traverse(params)) != NULL) {
    
    if (!strcmp(param->key, "size")) {
      if (get_param_integer(param->value, &(nodedata->size))) {
	goto error;
      }
    }
    if (!strcmp(param->key, "period")) {
      if (get_param_time(param->value, &(nodedata->period))) {
	goto error;
      }
    }
    if (!strcmp(param->key, "probability")) {
      if (get_param_double(param->value, &(nodedata->probability))) {
	goto error;
      }
    }
    if (!strcmp(param->key, "destination")) {
      if (get_param_integer(param->value, &(nodedata->destination))) {
	goto error;
      }
    }
    if (!strcmp(param->key, "geographical")) {
      if (get_param_integer(param->value, &(nodedata->geographical))) {
	goto error;
      }
    }
    
    
    
  }
  
  /* alloc overhead memory */
  if (i) {
    nodedata->overhead = malloc(sizeof(int) * i);
  } else {
    nodedata->overhead = NULL;
  }
  
  set_node_private_data(c, nodedata);
  return 0;
  
 error:
  free(nodedata);
  return -1;
}

int unsetnode(call_t *c) {
  struct nodedata *nodedata = get_node_private_data(c);
  
  if (nodedata->overhead) {
        free(nodedata->overhead);
  }
  free(nodedata);
  return 0;
}


/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
  struct nodedata *nodedata = get_node_private_data(c);
  
  int i = get_entity_links_down_nbr(c);
  entityid_t *down = get_entity_links_down(c);

  uint64_t start = nodedata->period + ((rand() * nodedata->period) / RAND_MAX);
  
  /* get overhead */
  while (i--) {
    call_t c0 = {down[i], c->node, c->entity};
    if ((get_entity_type(&c0) != MODELTYPE_ROUTING) 
	&& (get_entity_type(&c0) != MODELTYPE_MAC)) {
            nodedata->overhead[i] = 0;
    } else {
      nodedata->overhead[i] = GET_HEADER_SIZE(&c0);
    }
  }
  
  /* eventually schedule callback */
  scheduler_add_callback(start, c, callmeback, NULL);
  
  return 0;
}

/* ************************************************** */
/* ************************************************** */
int callmeback(call_t *c, void *args) {
  struct nodedata *nodedata = get_node_private_data(c);
  struct entitydata *entitydata = get_entity_private_data(c);

  int i = get_entity_links_down_nbr(c);
  entityid_t *down = get_entity_links_down(c); 
  destination_t destination = {-2, {-1, -1, -1}};

  if (c->node != 11)
    return 0;

  destination.id = 1001;
  
  if ( nodedata->geographical ) {
    destination.position.x = get_node_position(destination.id)->x;
    destination.position.y = get_node_position(destination.id)->y;
    destination.position.z = get_node_position(destination.id)->z;
  }
  
  //printf("s:%d:%lu:%d\n", c->node, get_time(), destination.id);
  
  while (i--) {
    packet_t *packet = packet_create(c, nodedata->size + sizeof(struct _rsend_header) + nodedata->overhead[i], -1);
    call_t c0 = {down[i], c->node, c->entity};
    
    struct _rsend_header *rsend = (struct _rsend_header*)(packet->data+nodedata->overhead[0]);
    rsend->id = c->node;
    
    if ((get_entity_type(&c0) == MODELTYPE_ROUTING) 
	|| (get_entity_type(&c0) == MODELTYPE_MAC)) {
      if (SET_HEADER(&c0, packet, &destination) == -1) {
	packet_dealloc(packet);
	continue;
      }
    }
    
    /* send a packet */    
    TX(&c0, packet);
  }
  nodedata->data_tx++;
  entitydata->data_tx++;
  
  
  if (get_time() <= 55000000000)
    scheduler_add_callback(get_time() + nodedata->period, c, callmeback, NULL);
  
  return 0;
}
 
int ioctl(call_t *c, int option, void *in, void **out) {
  return 0;
}

/* ************************************************** */
/* ************************************************** */
int tx(call_t *c, void *args) {
  
  return 0;
}

/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {
  struct nodedata *nodedata = get_node_private_data(c);
  struct entitydata *entitydata = get_entity_private_data(c);

  nodedata->data_rx++;
  entitydata->data_rx++;

  packet_dealloc(packet);
}

/* ************************************************** */
/* ************************************************** */
application_methods_t methods = {rx};
