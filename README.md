## Purpose

This simulator is designed to test UAV coverage problems. It incorporates a realistic energy model and physics model 
as well as lots of options for configuration.
 
## Getting started
The project is built with Maven, so you'll need that before starting. First navigate to an empty directory where you
 want to initialize the project, then:

```
git clone <this project>
cd path-coverage-simulator
```

To compile and run the project (respectively) use:
```
mvn compile
mvn exec:java
```

Most Java IDEs will compile and run the project for you, but you're welcome to use a plain text editor and the 
commands above 
if that's what you prefer.


## Creating a path planning algorithm
Path planning algorithms are represented here as subclasses of Drone. For instance, BoustrophedonDrone is a drone
 that covers the area according to the boustrophedon cellular decomposition (Choset, 1997). A drone gets program
  control in a few ways: when traversal starts (`onBegin`), at regular intervals (`onTick`), when it has reached
   its most recent waypoint (`onAwaiting`), or when a few other external events happen (`onBroadcastReceived
   `, `onEnergyDepleted`, etc.) Traversal ends either when the drone has depleted all its energy, or states that it
    is finished (`isDone`).

A drone can be either offline or online, and the simulator has a few examples of both. An offline drone preplans its
 route and does not change based on external factors, whereas an online drone computes new parts of the route as it
  flies. 

## Project layout
Sources are in `src/`, generated documentation is in `doc/`, non-code resources (images, CSV files, and the like) are in
 `res/`, and libraries not grabbed by Maven are in `lib/` (there are currently none). 

As far as source code structure goes:
* The `frame` package defines the frontend and input for the system.
* The `geom` package includes a number of geometric primitives useful when working with path planning algorithms.
* The `phys` package implements the motion and energy use models of the simulated drone.
* The `drone` package includes all implemented path planning algorithms.s

## Detailed documentation
Documentation by class is generated into the `doc/` directory; you can see it all at `doc/index.html` (best viewed
 in a browser).

## Projects
* Coverage path planning portions of this simulator are the product of Mel (Miles) Krusniak's work at the University 
of Missouri, and the associated code belongs to those projects. 
* MAP-ELITES, Particle Swarm Optimization, and brute-force optimization code for risk-based path planning, as well 
  as the changes to the code base that enable these algorithms, belong to Mel Krusniak's senior thesis (Yale 2022). 

* The infrastructure enabling these simulations is associated with both projects and can be repurposed for a variety 
  of other planning problems. Feel free to modify it at will!