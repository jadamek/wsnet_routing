/**
 *  \file   802_15_4_u_csma_ca.c
 *  \brief  802.15.4 unslotted csma ca protocol
 *  \author Guillaume Chelius
 *  \date   2007
 **/
#include <stdio.h>

#include <include/modelutils.h>


/* ************************************************** */
/* ************************************************** */
#define STATE_IDLE	  1
#define STATE_BACKOFF 2
#define STATE_TX      3
#define STATE_TXING   4

#define COMP_TIME 50000


/* ************************************************** */
/* ************************************************** */
#ifdef BPSK_868
#define macMinLIFSPeriod      2000000 /* 40 symbols cf p30 ref 802.15.4-2006 */
#define macMinSIFSPeriod      600000  /* 12 symbols cf p30 ref 802.15.4-2006 */
#define aUnitBackoffPeriod    1000000 /* 20 symbols cf p159 ref 802.15.4-2006 */
#define EDThresholdMin        -82 /* radio->mindBm + 10 cf p66 ref 802.15.4-2006 */
#elif BPSK_902
#define macMinLIFSPeriod      1000000 /* 40 symbols cf p30 ref 802.15.4-2006 */
#define macMinSIFSPeriod      300000  /* 12 symbols cf p30 ref 802.15.4-2006 */
#define aUnitBackoffPeriod    500000  /* 20 symbols cf p159 ref 802.15.4-2006 */
#define EDThresholdMin        -82     /* radio->mindBm + 10 cf p66 ref 802.15.4-2006 */
#elif OQPSK_2400
#define macMinLIFSPeriod      640000  /* 40 symbols cf p30 ref 802.15.4-2006 */
#define macMinSIFSPeriod      192000  /* 12 symbols cf p30 ref 802.15.4-2006 */
#define aUnitBackoffPeriod    320000  /* 20 symbols cf p159 ref 802.15.4-2006 */
#define EDThresholdMin        -75     /* radio->mindBm + 10 cf p66 ref 802.15.4-2006 */
#endif /* OQPSK_2400O */
#define macMinBE              3 /* cf p164 ref 802.15.4-2006 */
#define macMinBEMin           0 /* cf p164 ref 802.15.4-2006 */
#define macMaxBE              5 /* cf p163 ref 802.15.4-2006 */
#define macMaxBEMin           3 /* cf p163 ref 802.15.4-2006 */
#define macMaxBEMax           8 /* cf p163 ref 802.15.4-2006 */
#define macMaxCSMABackoffs    4 /* cf p163 ref 802.15.4-2006 */
#define macMaxCSMABackoffsMin 0 /* cf p163 ref 802.15.4-2006 */
#define macMaxCSMABackoffsMax 5 /* cf p163 ref 802.15.4-2006 */



/* ************************************************** */
/* ************************************************** */
struct nodedata {
    uint64_t clock;
    int state;

    int MaxCSMABackoffs;
    int MinBE;
    int MaxBE;
    int BE;
    int NB;

    void *packets;
    buffer_entry_t *txbuf;

    double EDThreshold;
    int cs;
    int cca;
};


/* ************************************************** */
/* ************************************************** */
struct _802_15_4_header {
    nodeid_t src;
    nodeid_t dst;
};


/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "802.15.4 unslotted csma ca mac module",
    "Guillaume Chelius and Elyes Ben Hamida",
    "0.1",
    MODELTYPE_MAC, 
    {NULL, 0}
};


/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {
     return 0;
}

int destroy(call_t *c) {
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
    struct nodedata *nodedata = malloc(sizeof(struct nodedata));
    param_t *param;

    nodedata->clock = 0;
    nodedata->state = STATE_IDLE;
    nodedata->packets = das_create();
    nodedata->txbuf = NULL;
    nodedata->cca = 1;
    nodedata->cs = 1;
    nodedata->EDThreshold = EDThresholdMin;
    nodedata->MaxCSMABackoffs = macMaxCSMABackoffs;
    nodedata->MaxBE = macMaxBE;
    nodedata->MinBE = macMinBE;

    das_init_traverse(params);
    while ((param = (param_t *) das_traverse(params)) != NULL) {
        if (!strcmp(param->key, "cca")) {
            if (get_param_integer(param->value, &(nodedata->cca))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "cs")) {
            if (get_param_integer(param->value, &(nodedata->cs))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "max-csma-backoffs")) {
            if (get_param_integer(param->value, &(nodedata->MaxCSMABackoffs))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "max-backoff-exponent")) {
            if (get_param_integer(param->value, &(nodedata->MaxBE))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "min-backoff-exponent")) {
            if (get_param_integer(param->value, &(nodedata->MinBE))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "cca-threshold")) {
            if (get_param_double(param->value, &(nodedata->EDThreshold))) {
                goto error;
            }
        }
    }
    if (nodedata->EDThreshold < EDThresholdMin) {
        nodedata->EDThreshold = EDThresholdMin;
    }
    if ((nodedata->MaxCSMABackoffs < macMaxCSMABackoffsMin)
        || (nodedata->MaxCSMABackoffs > macMaxCSMABackoffsMax)) { 
        nodedata->MaxCSMABackoffs = macMaxCSMABackoffs;
    }
    if ((nodedata->MaxBE < macMaxBEMin)
        || (nodedata->MaxBE > macMaxBEMax)) { 
        nodedata->MaxBE = macMaxBE;
    }
    if ((nodedata->MinBE < macMinBEMin)
        || (nodedata->MinBE > nodedata->MaxBE)) { 
        nodedata->MinBE = macMinBE;
    }


    set_node_private_data(c, nodedata);
    return 0;

 error:
    free(nodedata);
    return -1;
}

int unsetnode(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);
    buffer_entry_t *buff_ent;
    while ((buff_ent = (buffer_entry_t *)das_pop(nodedata->packets)) != NULL) {
        packet_dealloc(buff_ent->packet);
	free(buff_ent);
    }
    das_destroy(nodedata->packets);    
    if (nodedata->txbuf) {
        packet_dealloc(nodedata->txbuf->packet);
	free(nodedata->txbuf);
    }
    free(nodedata);
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int check_channel_busy(call_t *c) {
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};

    if (nodedata->cs && (radio_get_cs(&c0) >= nodedata->EDThreshold)) {
        return 1;
    }
    if (nodedata->cca && (radio_get_noise(&c0) >= nodedata->EDThreshold)) {
        return 1;
    }
    return 0;
}

int state_machine(call_t *c, void *args) {
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    uint64_t timeout;
    uint64_t backoff;
    packet_t *packet;	

    if (nodedata->clock != get_time()) {
        return 0;
    }

    switch (nodedata->state) {
		
    case STATE_IDLE:
        if (nodedata->txbuf == NULL){
            nodedata->txbuf = (buffer_entry_t*)das_pop_FIFO(nodedata->packets);
            if (nodedata->txbuf == NULL) {
                return 0;
            }
        }
        
        if (nodedata->MaxCSMABackoffs != 0) {
            nodedata->state = STATE_BACKOFF;
            nodedata->BE = nodedata->MinBE;
            nodedata->NB = 0;
            backoff = get_random_double() * (pow(2, nodedata->BE) - 1) * aUnitBackoffPeriod;
			
            nodedata->clock = get_time() + backoff;  
            nodedata->txbuf->event = scheduler_add_callback(nodedata->clock, c, state_machine, NULL);
            return 0;	
        } else {
            nodedata->state = STATE_TX;
            nodedata->clock = get_time();  
            nodedata->txbuf->event = scheduler_add_callback(nodedata->clock, c, state_machine, NULL);
            return 0;
        }		

    case STATE_BACKOFF:
        if (check_channel_busy(c)) {
            if ((++nodedata->BE) > nodedata->MaxBE) {
                nodedata->BE = nodedata->MaxBE;
            }
/*            if (++nodedata->NB >= nodedata->MaxCSMABackoffs) {
printf("[RDO] Too many backoffs, dropping packet at %d\n", c->node);
                packet_dealloc(nodedata->txbuf->packet);            
                nodedata->txbuf = NULL;

                nodedata->state = STATE_IDLE;
                nodedata->clock = get_time();
                state_machine(c,NULL);
                return 0;
            }*/
            backoff = get_random_double() * (pow(2, nodedata->BE) - 1) * aUnitBackoffPeriod;
            nodedata->clock = get_time() + backoff;
            nodedata->txbuf->event = scheduler_add_callback(nodedata->clock, c, state_machine, NULL);
            return 0;
        }
        nodedata->state = STATE_TX;
        nodedata->clock = get_time();  
        state_machine(c,NULL);
        return 0;
        
    case STATE_TX:
        packet = nodedata->txbuf->packet;
	free(nodedata->txbuf);
        nodedata->txbuf = NULL;
        timeout = packet->size * 8 * radio_get_Tb(&c0) + macMinSIFSPeriod;
        TX(&c0, packet); 
        nodedata->state = STATE_TXING;
        nodedata->clock = get_time() + timeout;
        scheduler_add_callback(nodedata->clock, c, state_machine, NULL);
        return 0;

    case STATE_TXING:
        nodedata->state = STATE_IDLE;
        nodedata->clock = get_time();
        state_machine(c,NULL);
        return 0;

    default:
        break;
    }

    return 0;
}
     

/* ************************************************** */
/* ************************************************** */
int set_header(call_t *c, packet_t *packet, destination_t *dst) {
    struct _802_15_4_header *header = (struct _802_15_4_header *) packet->data;
    header->dst = dst->id;
    header->src = c->node;
    return 0;
}

int get_header_size(call_t *c) {
    return (sizeof(struct _802_15_4_header));
}

int get_header_real_size(call_t *c) {
    return (sizeof(struct _802_15_4_header));
}


/* ************************************************** */
/* ************************************************** */
void tx(call_t *c, packet_t *packet) {
    struct nodedata *nodedata = get_node_private_data(c);
    buffer_entry_t *buff_ent = malloc(sizeof(buffer_entry_t));
    buff_ent->packet = packet;
    
    nodedata->clock = get_time() + COMP_TIME;  
    buff_ent->event = scheduler_add_callback(nodedata->clock, c, state_machine,
	NULL);
    das_insert(nodedata->packets, (void*)buff_ent); 
}


/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {
    //struct nodedata *nodedata = get_node_private_data(c);
    struct _802_15_4_header *header = (struct _802_15_4_header *) packet->data;
    array_t *up = get_entity_bindings_up(c);
    int i = up->size;

    /* cf p171 ref 802.15.4-2006: discard packet while in backoff */
/*    if (nodedata->state != STATE_IDLE) {
printf("[RDO] Node %d trying to send, dropping received packet\n", c->node);
        packet_dealloc(packet);
        return;
        
    }*/

    if ((header->dst != c->node) && (header->dst != BROADCAST_ADDR)) {
        packet_dealloc(packet);
        return;
    }
    
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


/* ************************************************** */
/* ************************************************** */

//returns buffer
void* get_buffer(call_t *call)
{
    struct nodedata *node_data = get_node_private_data(call);
    if(node_data->txbuf != NULL)
	das_insert(node_data->packets, (void*)node_data->txbuf);
    return node_data->packets;
}

//sets buffer
void set_buffer(call_t *call, void *new_buffer)
{
    struct nodedata *node_data = get_node_private_data(call);
    das_destroy(node_data->packets);

    //if mate found, buffer entry already deallocated in routing module
    if(node_data->txbuf && node_data->txbuf->packet == NULL)
    {
	free(node_data->txbuf);
	node_data->txbuf = NULL;

	node_data->state = STATE_IDLE;
	node_data->clock = get_time();
	scheduler_add_callback(node_data->clock, call, state_machine, NULL);
    }
    else if(node_data->txbuf)
	das_pop(new_buffer);

    node_data->packets = new_buffer;
    set_node_private_data(call, node_data);
    return;
}


/* ************************************************** */
/* ************************************************** */
mac_methods_t methods
 = {rx, 
                         tx,
                         set_header, 
                         get_header_size,
                         get_header_real_size,
			 get_buffer,
			 set_buffer};

