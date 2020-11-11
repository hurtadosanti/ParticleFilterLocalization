# Particle Filter Localization Project
Project for the Self-Driving Car Nanodegree Program  

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Your robot has been transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its 
initial location, and lots of (noisy) sensor and control data.

In this project we have implement a 2 dimensional particle filter in C++. The particle filter is given a map and 
some initial localization information (analogous to what a GPS would provide). At each time step the filter also 
get observation and control data.

## Tasks
The only file you should modify is [particle_filter.cpp](src/particle_filter.cpp) in the src directory. 
The file contains the scaffolding of a 
ParticleFilter class and some associated methods.

## Inputs to the Particle Filter
You can find the input to the particle filter in the data directory.

### The Map
map_data.txt includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. 
Each row has three columns:

    x position
    y position
    landmark id

Map data provided by 3D Mapping Solutions GmbH.

### Simulator input 
All other data the simulator provides, such as observations and controls. The values provided by the simulator to the 
program are:

- Noisy position data from the simulator
        
        ["sense_x"]
        ["sense_y"]
        ["sense_theta"]

- Previous velocity and yaw rate to predict the particle's transitioned state

        ["previous_velocity"]
        ["previous_yawrate"]

-  Receive noisy observation data from the simulator, in a respective list of x/y values

        ["sense_observations_x"]
        ["sense_observations_y"]

### Particle Filter Outputs
- best particle values used for calculating the error evaluation

        ["best_particle_x"]
        ["best_particle_y"]
        ["best_particle_theta"]

Optional message data used for debugging particle's sensing and associations
- Respective (x,y) sensed positions ID label

        ["best_particle_associations"]

- Respective (x,y) sensed positions

        ["best_particle_sense_x"] <= list of sensed x positions
        ["best_particle_sense_y"] <= list of sensed y positions

## Contributing
[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html)

### Build
#### Linux
There are two scripts provided to build and clean the server:

- [Clean binaries](clean.sh)
- [Build binary](build.sh)

#### Docker
To build, compile and run the code we use a [docker image](Dockerfile) together with CLion.
- To build the image run

        docker build -t dev/env .
        
- To run the image

        docker run -p 127.0.0.1:2222:22 -p 127.0.0.1:4567:4567 --name particle-filter-env --rm dev/env 

The code can be copy using ssh, then use cmake to setup and make to build. Finally run the particle_filter executable.

For more details of the Clion integration go to the post 
[Using Docker with CLion](https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/)
#### Execute

- Execute the [run.sh](run.sh) script
- Run the [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) and select the **Project 3**.

## References
- [Project Definition](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

## License
MIT License Copyright (c) 2016-2018 Udacity, Inc.