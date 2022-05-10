package com.mmkrusniak.phys;

import com.mmkrusniak.geom.Point;

/**
 * Routes are high-level drone instructions which specify a desired location or orientation and a way to get there.
 * (For instance, you might want to move a drone to the point (100, 200) at full throttle - best represented by
 * RouteThrough - or cautiously, stopping right over that point - best represented by RouteTo.)
 *
 * Subclasses are expected to describe how to change the drone's motion state appropriately to create the desired
 * motion. Most types of routes are also created with some sort of target location, such that the route is completed
 * when the drone has more or less reached that location, at which point some other action should be performed.
 */
public abstract class Route {
    /**
     * Gives the target location of the route, if there is one; otherwise returns null.
     * @return The target location of the route.
     */
    public abstract Point getTarget();

    /**
     * Sets the target location of the route, if applicable.
     * @param target The new target location for the route.
     */
    public abstract void setTarget(Point target);

    /**
     * Applies the route to a drone's motion state, changing its acceleration (and perhaps other motion attributes)
     * in accordance with the route. Note that physics in this simulation operates on discrete time steps, meaning
     * that the drone will skip directly from its current location to its location at time + t. If this skipping
     * presents issues, decrease t.
     * @param state A drone's motion state.
     * @param t The amount of time over which to apply the route.
     */
    public abstract void apply(MotionState state, double t);
}
