# dwpl

DwPL is a modified version of Dijsktra's algorithm that works on a layered graph structure.
Link to the [dwpl repository](https://github.com/apolicky/dwpl).

## How to build

The program uses the [SFML](https://www.sfml-dev.org/) library for graphical purposes.
Download the library by package manager, on deb based systems:

```console
apt-get install libsfml-dev
```

The program is build with `make` command and uses `g++` compiler with c++17 standard.
To unpack and compile the program do the following steps:

```console
unzip dwpl.zip
cd dwpl/bin
make
```

## Benchmark preparation

After the program is compiled, it can be run.
The program works with an edge-based graph, i.e. edges that are in a neighborhood
relation with other edges. There are a few such graphs in the `data` folder
with names ending with the suffix `.edgs`.

However, the user is not limited to this data. Any dataset following the schema
mentioned bellow, and also described in the file `data/templates/template.edgs`.

```console
[edgeId] [x1] [y1] [x2] [y2] [neighborId ...]
```

The data in the mentioned folder are in the said schema.
The original data comes from [this](https://www.cs.utah.edu/~lifeifei/research/tpq/SF.cnode)
and also [this](https://www.cs.utah.edu/~lifeifei/research/tpq/SF.cedge) website.
It can be converted to the edge-based format by a simple python
script located in `data/processing/shave_data.py`,
more specifically its function `to_edge_based()`.

The `data` folder contains also files with the suffix `.layers`
Those are the pre-processed layers for the layered graph.
Their primary use is to get the program up and running quickly.

### Editting the data

If any `.edgs` file does get edited, the corresponding `.layers` file
needs to be deleted to avoid using invalid information. A new `.layers` file
gets created and saved for the following use during the start up.

## Starting the program

The program can be run from the command line in two ways:

```console
./main                  // data/sf2k.edgs file is used by default
./main -f input.edgs    // input.edgs file is used.
```

### Workflow

The program operates in three phases: graph loading, pre-simulation phase, and post-simulation phase.

#### Graph loading

The graph from an `.edgs` file (possibly given as a command-line argument) is loaded into memory and is the base layer of the graph structure used by DwPL.
Next, the layers of the graph must be created. If the layers already existed and were stored
in `data/*.layers` file, they would be used to save some time.
Then comes the pre-simulation phase.

#### Pre-simulation phase

In this phase the graph and all of its layers are loaded in the memory.
The simulation can be run with the predefined configuration.
The configuration can be changed by commands listed **COMMANDS** .

The graph with its layers can also be drawn by the `draw` command.
Its paremeters are mentioned in the previously mentioned section.

The command `run` moves the program in the post-simulation phase.
At this moment the routes on which the vehicles will travel are created (depending on the configuration),
the vehicles are created and sent on their way.

#### Post-simulation phase

At this phase all vehicles have completed their routes, whether successfully or unsuccessfully.
Information about their routes are printed to the console.
Records of their routes still remain in memory,
more specifically the routes they have used.

With this, the `routes` parameter for the `draw` command makes the program draw the edges
the vehicles used.

The previous phase can be accessed by commands `clear` and `reset`.
The `clear` command clears the data about vehicles and routes.
It also deletes the created routes (pairs of endpoints), the seed of the random generator
remains the same. So the command `run` will perform a new simulation
where the configuration remains the same and the vehicles will follow different routes.
The `reset` command also clears the data about vehicles and routes.
The random generator is given the same seed as last time. The `run` command
will run the same simulation as last time, thus having the same result.

## Sample use of the program

Suppose the user wants to run a simulation on the San Francisco graph with 2k edges,
with 50 vehicles that drive between 5 pairs of endpoints and are navigated by DwPL.
The user should do the following steps.

```console
./main -f ../data/sf2k.edgs 
draw 4              # let the program draw for example the fifth layer
conf                # list the current conf - navigation type, #vehicles, #routes
v 50                # set the #vehicles to 50
r 5                 # set the #routes to 5, there will be 10 vehicles on each route
conf                # the configuration can be checked again
run                 # run the simulation 
draw routes         # render the routes vehicles used
reset | clear       # delete all vehicles, keep the config and run {the same | new} simulation
[ctrl] [+] D        # exit the program
```

## Commands

The program waits for commands in the console or executes the commands.
This list contains all commands of this program,
their description, and possibly their parameters and default values.

* `run` runs the configured simulation.
* `v`  sets the number of vehicles to new value. Expects one non-negative integer. Default value: **30**
* `r` sets the number of routes. Vehicles will travel between the given number of pairs of locations. If **0** then #routes == #vehicles. Default value: **0**
* `n` sets the type of navigation to one of following: d -- Dijkstra, a .. A*, **dwpl** .. Dijsktra w/ precision limit
* `conf` lists the specification of simulation.
* `clear` deletes existing routes, deletes congestions, keeps the random generator the same. Performing `run` starts new simulation, different routes will be created.
* `reset` deletes existing routes, deletes congestions, resets the random generator. Performing `run` should end with the same result as last time.
* `help` prints help.
* `draw` without argument draws the base layer of the loaded graph. With **int** draws layer with that number. With **routes** draws the routes vehicles drove, if there are any.

## Parameters

The program also works with various configuration parameters that are listed bellow.

* `dwpl_jumps` the precision limit for DwPL. Specifies the number of jumps DwPL has to make on path of each layer. It is located in `main.cpp(33)`
* `hca_k` the clustering parameter _k_, located in `Preprocessing.hpp(18)`
* `realtime` indicates whether vehicles find their routes with respect to reservations and current network state or not, located in `main.cpp(32)`
* `a_star_jumps` equivalent of `dwpl_jumps` for A* and Dijkstra. Specifies after how many hyperedges the algorithm tries to foind better routes, located in `Navigation.hpp(39)`
* `specify_more` specifies how many more hyperedges need to be explored on the base layer, located in `Navigation.hpp(40)`
* `vehs_per_unit` capacity of edges, located in `Navigation.hpp(41)`
* `routes` how many routes will the vehicles use in the simulation, same as using command `r`
* `vehicles` how many vehicles will be part of the simulation, same as using command `v`
* `nav_type` what navigation algorithm will be used in the simulation, same as command `v`
