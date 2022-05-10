package com.mmkrusniak.phys;

import com.mmkrusniak.geom.Point;

/**
 * RouteThrough is a simple drone instruction which requests a velocity change directly towards a point, without
 * significant slowing as the drone approaches the point. This is useful for passing over points as part of a plow
 * path where each point of interest lies roughly in a straight line.
 */
public class RouteThrough extends Route {

    private Point target;

    /**
     * Initializes a new RouteThrough a given point.
     * @param p A target location for the route.
     */
    public RouteThrough(Point p) {
        if(p == null) throw new IllegalArgumentException("RouteThrough cannot have null target");
        this.target = p;
    }

    /**
     * Yields the current target of the route. In a RouteThrough, this does not change unless done so externally.
     * @return The current target of the route.
     */
    @Override
    public Point getTarget() {
        return target;
    }

    /**
     * Sets the route's target location.
     * @param target The new target location for the route.
     */
    @Override
    public void setTarget(Point target) {
        this.target = target;
    }

    /**
     * Applies the route on a drone's motion state for a given period of time. For a RouteThrough, this essentially
     * amounts to a full throttle push towards the target.
     * @param state A drone's motion state.
     * @param t The amount of time over which to apply the route.
     */
    @Override
    public void apply(MotionState state, double t) {
        double dir = state.getLocation().bearing(target);
        state.set(2, 0, 20 * Math.cos(dir));
        state.set(2, 1, 20 * Math.sin(dir));

        double zError = Math.abs(target.z() - state.z());
        if(state.z() > target.z() + 0.5) state.setVelZ(-Math.min(2, zError));
        else if(state.z() < target.z() - 0.5) state.setVelZ(Math.min(2, zError));
        else state.setVelZ(0);
    }
}
