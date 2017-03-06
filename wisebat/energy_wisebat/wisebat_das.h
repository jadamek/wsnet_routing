
#include <include/modelutils.h>

typedef struct _das_elt {
    void            *data;
    struct _das_elt *next;
    struct _das_elt *previous;
} das_elt_t;

typedef struct _das {
    int       size;
    das_elt_t *trav;
    das_elt_t *elts;
    das_elt_t *elts_end; 
} das_t;

/** \typedef das_select_func_t
 * \brief The prototype of a select function
 **/
typedef int (* das_select_func_t)(void *, void *);

/**
 * \brief Remove objects selected by a delete function from the data structure.
 * \param das the opaque pointer to the data structure.
 * \param delete the function that selects objects to be removed.
 * \param arg an argument passed to the delete function.
 **/
void das_selective_delete(void *d, das_delete_func_t delete, void *arg);



void *das_create_iterator(void *d);

void *das_iterator_next(void *d, void **i);





//----------------------------------


int das_selective_get(void *d, das_select_func_t select, void* args, void **get_data) {

    das_t *das = (das_t *) d;
    das_elt_t *elt = das->elts;

    while (elt != NULL) {
        if(select(elt->data, args))
        {
            *get_data = elt->data;
            return 1;
        }
        elt = elt->next;
    }
    return 0;
}

/* Added by Quentin Bramas <quentin@bramas.fr> */
void *das_create_iterator(void *d) {
    return NULL;
}

/* Added by Quentin Bramas <quentin@bramas.fr> */
void *das_iterator_next(void *d, void **i) {
    das_t *das = (das_t *) d;
    das_elt_t ** el = (das_elt_t **)i;
    if((*el) == das->elts_end)
    {
        return NULL;
    }
    if((*el) == NULL) {
        (*el) = das->elts;
    }
    else {
        (*el) = (*el)->next;
    }
    return (*el)->data;
}

