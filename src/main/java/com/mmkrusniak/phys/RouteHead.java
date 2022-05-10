package com.mmkrusniak.phys;

import com.mmkrusniak.geom.Point;

/**
 * RouteHead is a drone instruction which requests only a heading change. At the moment we do not account for drones'
 * rotational velocity and acceleration - we assume that heading changes are instantaneous. These types of "heading"
 * routes are therefore assumed not to have any sort of target position; they complete immediately.
 */
public class RouteHead extends Route {

    private final double targHeading;

    /**
     * Constructs a route that turns the drone to face a certain heading.
     * @param heading The heading towards which a drone is to face.
     */
    public RouteHead(double heading) {
        targHeading = heading;
    }

    /**
     * Gives the target location of the route. Heading routes are untargeted (they take a single step to complete,
     * and are considered "done" at any location), so this always returns null.
     * @return null.
     */
    @Override
    public Point getTarget() {
        return null;
    }

    /**
     * Sets the target location of the route. Heading routes are untargeted (they take a single step to complete,
     *      * and are considered "done" at any location), so this has no effect.
     * @param target Not used.
     */
    @Override
    public void setTarget(Point target) { }

    /**
     * Applies the route onto a drone's motion. For a RouteHead this amounts to simply setting the drone's heading to
     * the one specified.
     * @param state A drone's motion state.
     * @param t The amount of time over which to apply the route.
     */
    @Override
    public void apply(MotionState state, double t) {
        state.setHeading(targHeading);
    }
}
