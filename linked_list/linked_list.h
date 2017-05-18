//=====================================================================
// ** Linked List
//=====================================================================
// Implements a linked list of generic pointers (void*), supporting
// memory-proactive and passive removal. Specifically: "Active" removal
// and clearing methods free the item(s) being removed. "Passive"
// removes the items from the list only, with no memory frees.
//=====================================================================

//---------------------------------------------------------------------
// * Single linked list node structure 

typedef struct ll_node{
    void*           data;
    struct ll_node* next;
    struct ll_node* prev;
} list_node;

//---------------------------------------------------------------------
// * Linked list main data structure

typedef struct{
    list_node*		front;
    list_node*		back;
    int			size;
    list_node*		traverser;
    list_node*		node_traverser;

} linked_list;

//-----------------------------------
// - Linked List Methods
//-----------------------------------
linked_list*	create_linked_list();
int		list_size(linked_list* list);
void*           list_at(linked_list* list, int position);
list_node*      list_node_at(linked_list* list, int position);

// - Item creation/addition
void            list_push_back(linked_list* list, void* item);
void            list_push_front(linked_list* list, void* item);
void            list_add_left(linked_list* list, list_node* neighbor, void* item);
void            list_add_right(linked_list* list, list_node* neighbor, void* item);

// - Traversal (item-based and node-based)
void            list_start_traversal(linked_list* list);
void            list_start_node_traversal(linked_list* list);
void*           list_traverse(linked_list* list);
list_node*      list_node_traverse(linked_list* list);

// - Removal (Active)
void            list_delete_item(linked_list* list, void* item);
void            list_delete_node(linked_list* list, list_node* node);
void            list_clean(linked_list* list);

// - Removal (Passive)
void*           list_pop_back(linked_list* list);
void*           list_pop_front(linked_list* list);
void*           list_pop_at(linked_list* list, int position);
list_node*      list_pop_node_at(linked_list* list, int position);
void            list_remove(linked_list* list, void* item);
void            list_remove_node(linked_list* list, list_node* node);
void            list_clear(linked_list* list);
