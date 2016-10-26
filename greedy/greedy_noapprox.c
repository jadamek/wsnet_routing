/**
 *  \file   greedy.c
 *  \brief  Greedy geographic routing
 *  \author Elyes Ben Hamida and Guillaume Chelius
 *  \date   2007
 **/
#include <stdio.h>

#include <include/modelutils.h>


/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "Greedy geographic routing",
    "Elyes Ben Hamida and Guillaume Chelius",
    "0.1",
    MODELTYPE_ROUTING, 
    {NULL, 0}
};


/* ************************************************** */
/* ************************************************** */
#define HELLO_PACKET 0
#define DATA_PACKET  1


/* ************************************************** */
/* ************************************************** */
struct routing_header {
    nodeid_t dst;
    position_t dst_pos;
    nodeid_t src;
    position_t src_pos;
    int hop;
    int type;
};

struct neighbor {
    int id;
    position_t position;
    uint64_t time;
};

struct nodedata {
    void *neighbors;
    int overhead;

    uint64_t start;
    uint64_t period;
    uint64_t timeout;
    int hop;
    int method;

    /* stats */
    int hello_tx;
    int hello_rx;
    int data_tx;
    int data_rx;
    int data_noroute;
    int data_hop;
};

static int sum_degree;
static int total_noroute;
static uint64_t total_hop;
static int nb_hop_msg;
static uint64_t nb_messages;
static double cost;

/* ************************************************** */
/* ************************************************** */
int advert_callback(call_t *c, void *args);
void energy(position_t *source, position_t *destination);

/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {
  sum_degree = 0;
  total_noroute = 0;
  cost = 0;
  return 0;
}

int destroy(call_t *c) {  
  printf("degree:%f\n", ((double)sum_degree) / ((double)get_node_count()));
  printf("noroute:%d\n", total_noroute);
  printf("route:%f\n", ((double)total_hop) / ((double)nb_hop_msg));
  printf("messages:%lu\n", nb_messages);
  printf("cost:%f\n", cost);
  return 0;
}


/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
    struct nodedata *nodedata = malloc(sizeof(struct nodedata));
    param_t *param;

    /* default values */
    nodedata->neighbors = das_create();
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
    nodedata->method = 0;
 
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
	if (!strcmp(param->key, "method")) {
            if (get_param_integer(param->value, &(nodedata->method))) {
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
    struct neighbor *neighbor;

    // count neighbors number
    sum_degree += das_getsize(nodedata->neighbors);
    total_noroute += nodedata->data_noroute;    

    while ((neighbor = (struct neighbor *) das_pop(nodedata->neighbors)) != NULL) {
        free(neighbor);
    }
    das_destroy(nodedata->neighbors);
    free(nodedata);
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    
    //printf("p:%03d:%lu:%f:%f:%f\n", c->node, get_time(), get_node_position(c->node)->x, get_node_position(c->node)->y, get_node_position(c->node)->z);
    //printf("%f %f\n", get_node_position(c->node)->x, -get_node_position(c->node)->y);

    /* get mac header overhead */
    nodedata->overhead = GET_HEADER_SIZE(&c0);
        
    /*  hello packet */
    if (nodedata->period > 0) {
        uint64_t start = get_time() + nodedata->start + get_random_double() * nodedata->period;
        scheduler_add_callback(start, c, advert_callback, NULL);
    }

    return 0;
}

int ioctl(call_t *c, int option, void *in, void **out) {
    return 0;
}

double orthogonal(position_t *s, position_t *d, position_t *n) {
  return fabs( 
	     distance(s, d) - 
	     (( ((n->x - s->x) * (d->x - s->x)) +  ((n->y - s->y) * (d->y - s->y)) + ((n->z - s->z) * (d->z - s->z)) ) / 
	      sqrt(pow((d->x - s->x), 2) + pow((d->y - s->y), 2) + pow((d->z - s->z), 2) ) )
	      );
}

double angle(position_t *a, position_t *b, position_t *c) { 
  double ca = pow(c->x - b->x, 2) +  pow(c->y - b->y, 2);
  double cb = pow(c->x - a->x, 2) +  pow(c->y - a->y, 2);
  double cc = pow(a->x - b->x, 2) +  pow(a->y - b->y, 2);
  
  double cosA = ( cb + cc - ca) / (2 * sqrt(cb) * sqrt(cc));

  double angle = acos(cosA);

  return angle ;
}

/* ************************************************** */
/* ************************************************** */
struct neighbor* get_nexthop(call_t *c, position_t *dst) {
    struct nodedata *nodedata = get_node_private_data(c);
    struct neighbor *neighbor = NULL, *n_hop = NULL;
    uint64_t clock = get_time();
    double dist = distance(get_node_position(c->node), dst);
    double d = dist;

    double orth_dist = dist;
    double orth_d = dist;   

    if( nodedata->method == 3) {
      dist = 10;
      d = 10;
    }

    /* parse neighbors */
    das_init_traverse(nodedata->neighbors);    
    while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {        
        if ((nodedata->timeout > 0)
            && (clock - neighbor->time) >= nodedata->timeout ) {
            continue;
        }
        
        /* choose next hop */
	switch (nodedata->method){
	case 0:
	  // closest to destination (Greedy)
	  if ((d = distance(&(neighbor->position), dst)) < dist) {
	    dist = d;
	    n_hop = neighbor;
	  }
	  break;
	case 1:
	  // closest to source with positive progression (NFP)
	  if ((d = distance(&(neighbor->position), get_node_position(c->node))) <= dist &&
	      distance(&(neighbor->position), dst) <= distance(get_node_position(c->node), dst)) {
	    dist = d;
	    n_hop = neighbor;
	  }
	  break;
	case 2:
	  // closest to destination orthogonal projection (MFR)
	  if ((d = orthogonal(get_node_position(c->node), dst, &(neighbor->position))) <= dist) {
	    if  ( d == dist ) {
	      if ((orth_d = distance(&(neighbor->position), dst)) < orth_dist ) {
		orth_dist = d;
		dist = d;
		n_hop = neighbor;
	      }
	    } else {
	      dist = d;
	      n_hop = neighbor;
	    }
	  }
	  break;
	case 3:
	  // minimize angle between Destination Source and Neighbor (Compass)
	  // We want the maximum cosinus 
	  if ((d = angle(get_node_position(c->node), dst, &(neighbor->position))) <= dist) {
	    if ( d == dist) {
	      if ((orth_d = distance(&(neighbor->position), dst)) < orth_dist ) {
		orth_dist = orth_d;
		n_hop = neighbor;
	      }
	    } else {
	      dist = d;
	      n_hop = neighbor;
	    }
	  }
	  break;
	default :
	  break;
	}
    }

    return n_hop;
}

void add_neighbor(call_t *c, struct routing_header *header) {
    struct nodedata *nodedata = get_node_private_data(c);
    struct neighbor *neighbor = NULL;

    /* check wether neighbor already exists */
    das_init_traverse(nodedata->neighbors);      
    while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {      
        if (neighbor->id == header->src) {
            neighbor->position.x = header->src_pos.x;
            neighbor->position.y = header->src_pos.y;
            neighbor->position.z = header->src_pos.z;
            neighbor->time = get_time();
            return;
        }
    }  

    neighbor = (struct neighbor *) malloc(sizeof(struct neighbor));
    neighbor->id = header->src;
    neighbor->position.x = header->src_pos.x;
    neighbor->position.y = header->src_pos.y;
    neighbor->position.z = header->src_pos.z;
    neighbor->time = get_time();
    das_insert(nodedata->neighbors, (void *) neighbor);
    return;
}


/* ************************************************** */
/* ************************************************** */
int set_header(call_t *c, packet_t *packet, destination_t *dst) {
    struct nodedata *nodedata = get_node_private_data(c);
    struct neighbor *n_hop = get_nexthop(c, &(dst->position));
    destination_t destination;
    struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};

    printf("x y\n%f %f\n", get_node_position(c->node)->x, -get_node_position(c->node)->y);

    /* if no route, return -1 */
    if (dst->id != BROADCAST_ADDR && n_hop == NULL) {
        nodedata->data_noroute++;
        return -1;
    }
    else if (dst->id == BROADCAST_ADDR) {
      n_hop->id = BROADCAST_ADDR;
    }

    energy(get_node_position(c->node), &(n_hop->position));
    /* set routing header */
    header->dst = dst->id;
    header->dst_pos.x = dst->position.x;
    header->dst_pos.y = dst->position.y;
    header->dst_pos.z = dst->position.z;
    header->src = c->node;
    header->src_pos.x = get_node_position(c->node)->x;
    header->src_pos.y = get_node_position(c->node)->y;
    header->src_pos.z = get_node_position(c->node)->z;
    header->type = DATA_PACKET;
    header->hop = nodedata->hop;

    /* Set mac header */
    destination.id = n_hop->id;
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
    struct neighbor *neighbor = (struct neighbor *) data;
    call_t *c = (call_t *) arg;
    struct nodedata *nodedata = get_node_private_data(c);
    if ((get_time() - neighbor->time) >= nodedata->timeout) {
        return 1;
    }
    return 0;
}

int advert_callback(call_t *c, void *args) {
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    destination_t destination = {BROADCAST_ADDR, {-1, -1, -1}};
    packet_t *packet = packet_create(c, nodedata->overhead + sizeof(struct routing_header), -1);
    struct routing_header *header = (struct routing_header*) (packet->data + nodedata->overhead);
        
    //printf("i:%03d:%lu:0\n", c->node, get_time());

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
    
    /* send hello */
    nb_messages++;
    TX(&c0, packet);
    nodedata->hello_tx++;

    /* check neighbors timeout  */
    das_selective_delete(nodedata->neighbors, neighbor_timeout, (void *) c);

    /* schedules hello */
    scheduler_add_callback(get_time() + nodedata->period, c, advert_callback, NULL);
    return 0;
}


/* ************************************************** */
/* ************************************************** */
void tx(call_t *c, packet_t *packet) {
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    
    nodedata->data_tx++;
    TX(&c0, packet);
}


/* ************************************************** */
/* ************************************************** */
void forward(call_t *c, packet_t *packet) {  
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    struct routing_header *header = (struct routing_header *) (packet->data + nodedata->overhead);
    struct neighbor *n_hop = get_nexthop(c, &(header->dst_pos));
    destination_t destination;    

    /* delivers packet to application layer */
    if (n_hop == NULL) {
      nodedata->data_noroute++;
      packet_dealloc(packet);

      return;

        array_t *up = get_entity_bindings_up(c);
        int i = up->size;
        
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

        return;
    }
    
    /* update hop count */
    header->hop--;
    if (header->hop == 0) {
      nodedata->data_hop++;
      nodedata->data_noroute++;
      packet_dealloc(packet);
      return;
    }
    
    energy(get_node_position(c->node), &(n_hop->position));

    printf("%f %f\n", get_node_position(c->node)->x, -get_node_position(c->node)->y);    
    //printf("on %f, %f forward to %f, %f\n", get_node_position(c->node)->x, get_node_position(c->node)->y, n_hop->position.x, n_hop->position.y);

    /* set mac header */
    destination.id = n_hop->id;
    destination.position.x = -1;
    destination.position.y = -1;
    destination.position.z = -1;
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
    case HELLO_PACKET :         
        nodedata->hello_rx++;
        add_neighbor(c, header);
        packet_dealloc(packet);
        break;

    case DATA_PACKET : 
        nodedata->data_rx++;

	if ((header->dst != BROADCAST_ADDR) && (header->dst != c->node) ) {
	  forward(c, packet);
	  return;
        }       
	
	printf("%f %f\n", get_node_position(c->node)->x, -get_node_position(c->node)->y);

	total_hop += nodedata->hop - header->hop + 1;
	nb_hop_msg++;

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

void energy (position_t *source, position_t *destination) {
  double dist = distance(source, destination);
  cost += pow(3, 8) + pow(dist, 4);
}

/* ************************************************** */
/* ************************************************** */
routing_methods_t methods = {rx, 
                             tx, 
                             set_header, 
                             get_header_size,
                             get_header_real_size};
