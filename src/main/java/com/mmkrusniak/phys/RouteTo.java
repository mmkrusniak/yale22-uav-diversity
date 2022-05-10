package com.mmkrusniak.phys;

import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Point;

/**
 * RouteTo implements a drone motion protocol that aims to stop near-perfectly at a designated point. The idea is
 * that while in some cases it's okay to simply pass over a point at any velocity, sometimes it's important to stop
 * right on that point - for instance, when making a hard turn.
 *
 * This class implements a very rough PID-style control system to stop the drone approximately on the point. It's not
 * a very smart control system; it's just intended to be enough to prevent the drone from waving around points
 * without actually hitting them correctly.
 */
public class RouteTo extends Route {

    // It's PID loop time
    // It turns out that we can get usually along fine with just the proportional term though.
    private final double kP;
    private final double kI;
    private final double kD;

    private Point target;
    private Point start;
    private double sum_error;
    private double elapsed_time;

    /**
     * Constructs a new RouteTo a given target. Also grabs the appropriate physics options.
     * @param p A target location.
     */
    public RouteTo(Point p) {
        if(p == null) throw new IllegalArgumentException("RouteTo cannot have null target");
        kP = Option.get("physics", "kp", Double.class);
        kI = Option.get("physics", "ki", Double.class);
        kD = Option.get("physics", "kd", Double.class);
        if(p.z() == 0) target = new Point(p.x(), p.y(), 50);
        else target = p;
    }

    /**
     * Gives the current target location of the route. In a RouteTo, this does not change unless done so externally.
     * @return The current target location of the route.
     */
    @Override
    public Point getTarget() {
        return target;
    }

    /**
     * Sets the target location for the route.
     * @param target The new target location for the route.
     */
    @Override
    public void setTarget(Point target) {
        this.target = target;
    }

    /**
     * Applies the route on the drone's motion for a given period of simulation time. RouteTo does this with a
     * PID-like loop (although it should be said that only the proportional component is crucial.
     * @param state A drone's motion state.
     * @param t The amount of time over which to apply the route.
     */
    @Override
    public void apply(MotionState state, double t) {
        Point loc = state.getLocation();
        if(start == null) start = loc;
        double dir = state.getLocation().bearing(target);
        double error = state.getLocation().sqDistance2D(target);

        elapsed_time += t;
        sum_error += error * t;

        double zError = Math.abs(target.z() - state.z());
        if(state.z() > target.z() + 0.5) state.setVelZ(-Math.min(2, zError));
        else if(state.z() < target.z() - 0.5) state.setVelZ(Math.min(2, zError));
        else state.setVelZ(0);

        state.setAcc2D(dir, Math.min(kP*error + kI * sum_error * kD*sum_error/elapsed_time, 20));
    }
}
