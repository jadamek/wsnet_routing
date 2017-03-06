WiSeBat Model for WSNet
=======================

WiSeBat is wireless sensor battery model for WSNet Simulator.

It takes place in the energy layout and offer many features accesible from other layers by IOCTL communications.

Warning!! curently the radio models of WSNet must be changed to work with WiSeBat. The radio half1d.c is given in the radio_model folder. Replace the file in the wsnet source by the given file, redo make and make install and you can use this radio. Otherwise your radio will always be in the IDLE state.

Example: the application layer can register a temperature sensor, define a consumption function (that can depends on its mode and on the battery voltage) and then change its mode during the execution of the simulation.

Usage
--------

Define the battery used by your device:

```xml
<entity name="battery" library="energy_realistic_battery" >
  <init energy="25" voltage-characteristic="[0:3,20:2.8,25:1]" capacity-characteristic="[0.3:25,1:20,4:15]" internal-resistance="[0.3:25,1:20,4:15]" cut-off-voltage="2"/>
</entity>
...
<node id="0" as="sensor">
  <for entity="battery" log="wisebat.log" log-level="3" energy="7" />
</node>
```


And use it in your application layer:
```c
    struct nodedata{
      // add a sensor to the node data
      battery_component_t temperature_component;
      // ...
    }

    int bootstrap(call_t * c) {
      // ...
      nodedata->temperature_component->name = "temperature";
      nodedata->temperature_component->consume = temperature_consumption;
      battery_register_component(c, nodedata->temperature_component, MODE_OFF);
      // ...
    }

    //anywhere in your application you can start the sensor with the following command
    battery_set_component_mode(c, nodedata->temperature_sensor, MODE_ON);
    // and stop it with
    battery_set_component_mode(c, nodedata->temperature_sensor, MODE_OFF);
    // or use different modes
    battery_set_component_mode(c, nodedata->temperature_sensor, MODE_SLEEP);
    // or a custom one (greater than 128 so that it will never overlap the default ones)
    #define MODE_TEMP_READ_PRECISION_5 129
    battery_set_component_mode(c, nodedata->temperature_sensor, MODE_TEMP_READ_PRECISION_5);
```

Documentation
-------------------
### Entity Options

`energy`: the capacity of the battery (in mAh) default = 100

`internal-resistance`: in internal resistance of the battery (in Ohm)

`nominal-discharge-current`: the nominal discharge current of the battery (in mA)

`cut-off-voltage`: the cut-off-voltage voltage of your application (in V)

The three last options can be constant value or list of couples. For example `[0:3,20:2.8,25:1]` means 3 volt when the residual is 0, 2.8 volt when the residual is 20, and 1 volt when the residual is 25.


### Node Options

`log`: the filename where the battery log will be saved (no log if it is empty) default = ""

`log-level`: the level of the log (0, 1, 2, 3) default = 3

* 0: no log
* 1: log only residual and voltage with the interval `log-interval`
* 2: log every change in residual and voltage (equivalent to log-level=1 with log-interval=1)
* 3: log every change in residual and voltage. Also log every instantaneous current of every component.

`log-interval`: the time interval for logging with log-level = 1


### Components Functions

All the function use `IOCTL` communication with the energy layer, and return `IOCTL_OK` (0) if it works.


* `int battery_register_component (call_t *c, component_t *component, component_mode_t mode)`
    * `c` the application call
    * `component` the pointer to the component you want to register
    * `mode` the first mode used by the component (usally `MODE_OFF`)


* `int battery_set_component_mode (call_t *c, component_t *component, component_mode_t mode)`
    * `c` the application call
    * `component` the pointer to the component to change. The component **has to be previously registered**.
    * `mode` the mode used by the component

### Callbacks

You can add callbacks with the function `battery_register_callback(call_t *c, battery_callback_type_t callback_type, int (*callback)(call_t*, component_context_t*, void*))`

there are currently two type of callbacks:

* BATTERY_AFTER_COMPONENTS_UPDATE: called after the update of alls components and before the converstion in equivalent current. The last arg is of type double* and represent the instantaneous current of all the components. You can update it as you want.

* BATTERY_BEFORE_COMPONENT_CONSUME: called before calling the consume function of each component. It is useful if you want to change the context (for example the voltage). The last argument is of type component_t* and is the component on which the consume function will be called.

See example 2 for an example to create a power manager: constant voltage for components but a little increase in the current.


#Example

###Example 1

You will find an example in the example folder. The example consist of two nodes that do nothing exept consuming energy. The application used is the battery_test. A load is applied on the battery at regular interval 15ms on 200ms off for node 1 and 150ms on 2s off for node 1. The first node log the battery residual and voltage (log level 1). The latter log in addition every instantaneous current draw by the load.

A small tool is available in the same folder to visualized the evolution: plot.html
The html page require a file given in the url parameter.
For the first node: 
  `plot.html?file=example.0.wisebat.log`

For the last node: 
  `plot.html?file=example.1.wisebat.log&current_of=consumer&startTime=0&endTime=8`
`startTime` and `endTime` are in second.

###Example 2

This example consists of one sender node and one receiver. The sender read the sensor value and send the value to the receiver at regular interval of 30 seconds. between two cycle, the cpu is in sleep mode. The library scenario_tool.h helps create a communication cycle.

see the current draw of the components:

`file:///home/quentin/wisebat/examples/communication_cycle/plot.html?file=sender.wisebat.log&startTime=29.92&endTime=29.98&current_of=cpu,sensor,radio`



#WSNet Install

```
  sudo apt-get install gcc libtool make autoconf pkg-config openjdk-6-jdk libglib2.0-0 libglib2.0-dev libxml2 libxml2-dev  libgsl0-dev subversion
  svn checkout svn://scm.gforge.inria.fr/svn/wsnet && cd wsnet
  ./bootstrap
  ./configure
  make
  sudo make install
```