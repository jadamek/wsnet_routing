// ideal_battery.c
// ideal non-energy reducing battery
// James Robinson
// 8/5/2016

#include <include/modelutils.h>

////////////////////////////////////////////////////////////////////////////////
// Model Information

model_t model =
{
    "Ideal Battery",
    "James Robinson",
    "0.1",
    MODELTYPE_ENERGY,
    {NULL, 0}
};

////////////////////////////////////////////////////////////////////////////////
// Init

int init(call_t *call, void *param)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destroy

int destroy(call_t *call)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set Node

int setnode(call_t *call, void *params)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Unset Node

int unsetnode(call_t *call)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Bootstrap

int bootstrap(call_t *call)
{
    return 0;
}

int ioctl(call_t *call, int option, void *in, void**out)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Energy Functions

void consume_tx(call_t *call, uint64_t duration, double txdBm)
{
    return;
}

void consume_rx(call_t *call, uint64_t duration)
{
    return;
}

void consume_idle(call_t *call, uint64_t duration)
{
    return;
}

void consume(call_t *call, double energy)
{
    return;
}

double energy_consumed(call_t *call)
{
    return -1;
}

double energy_remaining(call_t *call)
{
    return -1;
}

double energy_status(call_t *call)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Exported Functions

energy_methods_t methods =
{
    consume_tx,
    consume_rx,
    consume_idle,
    consume,
    energy_consumed,
    energy_remaining,
    energy_status
};
