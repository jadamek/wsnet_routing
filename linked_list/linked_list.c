#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "linked_list.h"

#define NEW(type) malloc(sizeof(type))

//---------------------------------------------------------------------
// * Create List
//---------------------------------------------------------------------
// Allocates and returns a pointer to a new empty list.
//---------------------------------------------------------------------
linked_list* create_linked_list(){
    linked_list* list = NEW(linked_list);
    list->front = NULL;
    list->back = NULL;
    list->size = 0;
    list->traverser = NULL;
    list->node_traverser = NULL;

    return list;
}

//---------------------------------------------------------------------
// * Get List Size
//---------------------------------------------------------------------
// Returns the number of elements in the list (0 for empty).
//---------------------------------------------------------------------
int list_size(linked_list* list){
    if(list == NULL) return -1;
    return list->size;
}

//---------------------------------------------------------------------
// * Get List Item At Position
//---------------------------------------------------------------------
// Returns the item in the list 'position' elements from the beginning
// of the container or NULL if out of bounds.
//---------------------------------------------------------------------
void* list_at(linked_list* list, int position){
    if(list == NULL) return NULL;
    void *item = NULL;

    list_node* node = list_node_at(list, position);
    if(node != NULL) item = node->data;

    return item;
}

//---------------------------------------------------------------------
// * Get List Item At Position
//---------------------------------------------------------------------
// Returns the node in the list 'position' elements from the beginning
// of the container or NULL if out of bounds.
//---------------------------------------------------------------------
list_node* list_node_at(linked_list* list, int position){
    if(list == NULL) return NULL;

    list_node* node = NULL;
    int i;

    if(list == NULL) return node;

    if(position <= list->size / 2){
        // Search front to back
        i = 0;
	node = list->front;

        while(node != NULL && i != position){
	    node = node->next;
	    i++;
	}
    }
    else{
        // Search back to front
        i = list->size - 1;
	node = list->back;

        while(node != NULL && i != position){
	    node = node->prev;
	    i--;
	}
    }

    return node;
}

//=====================================================================
// - Item creation/addition

//---------------------------------------------------------------------
// * Push Back
//---------------------------------------------------------------------
// Adds a new item to the end of the list
//---------------------------------------------------------------------
void list_push_back(linked_list* list, void* item){
    if(list == NULL) return;

    list_node* new_node = NEW(list_node);
    new_node->data = item;
    new_node->prev = list->back;
    new_node->next = NULL;

    if(list->back != NULL) list->back->next = new_node;
    list->back = new_node;
    if(list->front == NULL) list->front = new_node;
    list->size++;
}

//---------------------------------------------------------------------
// * Push Front
//---------------------------------------------------------------------
// Adds a new item to the start of the list
//---------------------------------------------------------------------
void list_push_front(linked_list* list, void* item){
    if(list == NULL) return;

    list_node* new_node = NEW(list_node);
    new_node->data = item;
    new_node->next = list->front;
    new_node->prev = NULL;

    if(list->front != NULL) list->front->prev = new_node;
    list->front = new_node;
    if(list->back == NULL) list->back = new_node;
    list->size++;
}

//---------------------------------------------------------------------
// * Add Item (Left Of)
//---------------------------------------------------------------------
// Adds a new item to the left of (before) a given node in the list
//---------------------------------------------------------------------
void list_add_left(linked_list* list, list_node* neighbor, void* item){
    if(list == NULL || neighbor == NULL) return;

    list_node* new_node = NEW(list_node);
    new_node->data = item;
    new_node->next = neighbor;
    new_node->prev = neighbor->prev;
    if(neighbor->prev != NULL) neighbor->prev->next = new_node;
    neighbor->prev = new_node;

    if(list->front == neighbor) list->front = new_node;
    list->size++;
}

//---------------------------------------------------------------------
// * Add Item (Right Of)
//---------------------------------------------------------------------
// Adds a new item to the right of (after) a given node in the list
//---------------------------------------------------------------------
void list_add_right(linked_list* list, list_node* neighbor, void* item){
    if(list == NULL || neighbor == NULL) return;

    list_node* new_node = NEW(list_node);
    new_node->data = item;
    new_node->next = neighbor->next;
    new_node->prev = neighbor;
    if(neighbor->next != NULL) neighbor->next->prev = new_node;
    neighbor->next = new_node;

    if(list->back == neighbor) list->back = new_node;
    list->size++;
}

//=====================================================================
// - Traversal (item-based and node-based)

//---------------------------------------------------------------------
// * Initialize Traversal
//---------------------------------------------------------------------
// Returns the by-value traverser position to the front of the list
//---------------------------------------------------------------------
void list_start_traversal(linked_list* list){
    if(list == NULL) return;

    list->traverser = list->front;
}

//---------------------------------------------------------------------
// * Initialize Traversal (By Node)
//---------------------------------------------------------------------
// Returns the by-node traverser position to the front of the list
//---------------------------------------------------------------------
void list_start_node_traversal(linked_list* list){
    if(list == NULL) return;

    list->node_traverser = list->front;
}

//---------------------------------------------------------------------
// * Traverse One Item
//---------------------------------------------------------------------
// Returns the item currently referenced by the list traverser, then
// steps the traverser to the next node (if it exists)
//---------------------------------------------------------------------
void* list_traverse(linked_list* list){
    if(list == NULL) return NULL;
    if(list->traverser == NULL) return NULL;

    void* item = list->traverser->data;
    list->traverser = list->traverser->next;

    return item;
}

//---------------------------------------------------------------------
// * Traverse One Node
//---------------------------------------------------------------------
// Returns the node currently referenced by the list traverser, then
// steps the traverser to the next node (if it exists) 
//---------------------------------------------------------------------
list_node* list_node_traverse(linked_list* list){
    if(list == NULL) return NULL;
    if(list->node_traverser == NULL) return NULL;

    void* node = list->node_traverser;
    list->node_traverser = list->node_traverser->next;

    return node;
}

//=====================================================================
// - Removal (Active)

//---------------------------------------------------------------------
// * Delete Item
//---------------------------------------------------------------------
// Searches for the item in the list, then removes it and frees it from
// memory
//---------------------------------------------------------------------
void list_delete_item(linked_list* list, void* item){
    if(item != NULL) free(item);
    list_remove(list, item);
}

//---------------------------------------------------------------------
// * Delete Node
//---------------------------------------------------------------------
// Removes the given node from the its list, and frees the item it
// housed
//---------------------------------------------------------------------
void list_delete_node(linked_list* list, list_node* node){
    if(node->data != NULL) free(node->data);
    list_remove_node(list, node);
}

//---------------------------------------------------------------------
// * Clean (Delete All)
//---------------------------------------------------------------------
// Removes all items from the list, freeing (deleting) them
//---------------------------------------------------------------------
void list_clean(linked_list* list){
    if(list == NULL) return;

    list_node* node = list->front, *to_remove;

    while(node != NULL){
        if(node->data != NULL) free(node->data);
        to_remove = node;
        node = node->next;

	free(to_remove);
    }
}

//=====================================================================
// - Removal (Passive)

//---------------------------------------------------------------------
// * Pop Back
//---------------------------------------------------------------------
// Removes the last item in the list, returning it
//---------------------------------------------------------------------
void* list_pop_back(linked_list* list){
    if(list == NULL) return NULL;
    if(list->back == NULL) return NULL;

    void* item = list->back->data;
    list_remove_node(list, list->back);

    return item;
}

//---------------------------------------------------------------------
// * Pop Front 
//---------------------------------------------------------------------
// Removes the first item in the list, returning it
//---------------------------------------------------------------------
void* list_pop_front(linked_list* list){
    if(list == NULL) return NULL;
    if(list->front == NULL) return NULL;

    void* item = list->front->data;
    list_remove_node(list, list->front);

    return item;
}

//---------------------------------------------------------------------
// * Pop Item At Position
//---------------------------------------------------------------------
// Removes the item at the specified positions, returning it
//---------------------------------------------------------------------
void* list_pop_at(linked_list* list, int position){
    if(list == NULL) return NULL;

    list_node* node = list_node_at(list, position);
    if(node == NULL) return NULL;

    void* item = node->data;
    list_remove_node(list, node);

    return item;
}

//---------------------------------------------------------------------
// * Remove Item
//---------------------------------------------------------------------
// Searches for the item in the list, then removes it
//---------------------------------------------------------------------
void list_remove(linked_list* list, void* item){
    if(list == NULL) return;

    list_node* node;
    list_start_node_traversal(list);
    while((node = list_node_traverse(list)) != NULL){
        if(node->data == item) break;
    }

    list_remove_node(list, node);
}

//---------------------------------------------------------------------
// * Remove Node
//---------------------------------------------------------------------
// Removes the given node from the its list
//---------------------------------------------------------------------
void list_remove_node(linked_list* list, list_node* node){
    if(list == NULL) return;

    if(node != NULL){
	if(node->prev != NULL) node->prev->next = node->next;
	if(node->next != NULL) node->next->prev = node->prev;

	if(list->front == node) list->front = node->next;
	if(list->back == node) list->back = node->prev;
	list->size--;

        free(node);
    }
}

//---------------------------------------------------------------------
// * Clear (Remove All)
//---------------------------------------------------------------------
// Removes all items from the list, but does not free items themselves
//---------------------------------------------------------------------
void list_clear(linked_list* list){
    if(list == NULL) return;

    list_node* node = list->front, *to_remove;

    while(node != NULL){
        to_remove = node;
        node = node->next;

	free(to_remove);
    }
}
