# braveNewHair

### About The Project

This is a curly hair simulation based on the [technical paper]( https://graphics.pixar.com/library/CurlyHairB/paper.pdf) written for Merida's hair in Pixar's *Brave*.

The presentation for our project with simulation demos is linked [here](https://docs.google.com/presentation/d/10f5GF-52E32JyRj2TxI6A--lrAVFjGNAD9MdQ_NBh_s/edit?usp=sharing). 

### Filestructure

This project currently has two branches: 
* **main:** basic collision code, has jello_multihair.cpp to simulate lock of hair 

* **collisions:** has subsampled better collision algorithm

Starter code from [USC Jello Simulation Assignment](http://barbic.usc.edu/cs520-s20/assign1/) was used for this project, which is why there are many references to 'jello.'

The following are basic descriptions and roles of the files in this directory: 
* **createWorld.cpp:** script to generate 'world' files (located in world/) that contain information about hair strand's initial positions, velocities, and simulation hyperparameters like timesteps and coefficients
* **input.cpp** file that contains functions to read the world files and initialize hair structs. This file has key mappings. 
* **jello.cpp** file that contains initialization of jello structs (hair strand data structures). This is the main wrapper file. 
* **jello.h** header file with basic jello world struct initialization
* **physics.cpp** file that contains main hair simulation logic (integrator, force computation, spring logic, collisions, etc.)
* **showCube.cpp** file that contains OpenGL code to render hair

### How to Run 
To view a simulation, a world file needs to be created with the desired parameters. This file will be generated into the world/ folder. Then we run the main jello script with this file as input and view the simulation. 
```
make
./createWorld
./jello world/temp.w
```

To modify simulation parameters, go into **createWorld.cpp** main function and modify parameters. To change the number of points/particles on the hair - modify the macro in **createWorld.cpp** and **jello.h**. 
