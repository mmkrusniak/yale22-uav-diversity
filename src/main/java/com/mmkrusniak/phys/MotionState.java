package com.mmkrusniak.phys;

import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

/**
 *
 * There are a number of convenience getters and setters that manage specific components of motion, enough that I
 * won't document them individually, but they follow a pretty clear naming convention: i.e., x() and setX() manage the
 * first dimension of position, velY() and setVelY() manage the second dimension of velocity, accN() and setAccN()
 * manage the nth dimension of acceleration, etc.
 */
@SuppressWarnings("unused") // Some of the lesser getters and setters are unused currently but important to have anyway
public class MotionState {
    private final double[][] state;
    private double heading = 0.0;
    private final List<MotionListener> listeners;
    private MotionType currentMotion = MotionType.CONSTANT;

    /**
     * Defines three types of drone motion useful when calculating energy usage.
     */
    public enum MotionType { ACCELERATING, CONSTANT, DECELERATING }

    /**
     * Initializes a motion model in dim dimensions up to the (depth-1)th derivative of motion. In 2020, REU drone work
     * only analyzed drone motion up to depth 3 (acceleration), which was sufficient to match empirical behavior but
     * could be improved. Practically speaking dim is not greater than 3, although it is conceivable that heading
     * could be encoded as an additional dimension instead of the static version currently used (which is more
     * convenient but less descriptive), and indeed, some older parts of this code may still use that convention in
     * sngle points.
     * @param dim Number of dimensions of motion to model.
     * @param depth Number of derivatives of motion to model (position only is 1, position and velocity is 2, etc)
     */
    public MotionState(int dim, int depth) {
        listeners = new CopyOnWriteArrayList<>();
        state = new double[depth][dim];
    }

    /**
     * Initializes a motion model with (depth-1) derivatives of motion and a number of dimensions according to a
     * starting point. The state is initialized with that specified starting location.
     * @param start A starting location; its number of dimensions specifies the dimension of the model.
     * @param depth Number of derivatives of motion to model (position only is 1, position and velocity is 2, etc)
     */
    public MotionState(Point start, int depth) {
        this(start.dim(), depth);
        setLocation(start);
    }

    /**
     * Advances the motion model t simulation seconds into the future. The simulation is discrete with time-steps of
     * t, although some interpolation is performed. (If you find that drones are skipping or spinning over target
     * points, consider taking a close look at this section.)
     *
     * This applies physics, such that each level of motion gains the level one above it (position gains velocity,
     * velocity gains acceleration, etc.) Drag is also applied here. Finally, the motion is interpolated to determine
     * which motion listeners should be triggered.
     * @param t The number of simulation seconds by which to advance the motion.
     */
    public void advance(double t) {

        // When I say "take a close look" it's because there are lots of subtle bugs that can originate here, since
        // things that go wrong here tend to rise all the way to the top of the project. I can't promise that this is
        // all perfect, unfortunately, so you're on your own here.

        double[] oldLoc = state[0].clone();
        double oldVel = vel2D();

        // Apply drag
        double drag = Option.get("physics", "drag", Double.class);
        double vel = vel2D();
        double dir = new Line(getLocation(), new Point(state[0][0]+state[1][0], state[0][1]+state[1][1])).measure();
        state[1][0] -= drag * Math.cos(dir) * vel * vel * t;
        state[1][1] -= drag * Math.sin(dir) * vel * vel * t;

        // Basic physics: position gets velocity*t, velocity gets acceleration*t*t, etc.
        // in theory we can do further derivatives, but practically we don't.
        for (int i = state.length-1; i > 0; i--) {
            for (int j = 0; j < state[i].length; j++) {
                state[i-1][j] += state[i][j]*t;
            }
        }



        // Welcome to the finest level of granularity in the project!
        // Here we've got to deal with what happens when we skip over a point we cared about during physics.
        // Thankfully, down here in the powdery bits we can treat everything like a line anyway.
        // We do have to sort our listeners first so that we do them in the right order.
        Line path = new Line(getLocation(), new Point(oldLoc));
        List<MotionListener> activatedListeners = new ArrayList<>();
        List<MotionListener> removable = new ArrayList<>();
        for(MotionListener ml: listeners) {
            if(ml.point == null) {
                ml.action.accept(getLocation());
                removable.add(ml);
            } else if(ml.point.distance3D(path) < ml.dist) {
                activatedListeners.add(ml);
                removable.add(ml);
            }
        }
        listeners.removeAll(removable);

        activatedListeners.sort((a, b) -> (int) (b.point.distance3D(getLocation()) - a.point.distance3D(getLocation())));
        double[] savedPosition = state[0];
        for(MotionListener ml: activatedListeners){
            Point p = path.closest(ml.point);
            p = new Point(p.x(), p.y(), z());
            setLocation(p);
            ml.action.accept(path.closest(p));
        }
        state[0] = savedPosition;

        // So, at worst, the lowest level of granularity scales O(nlogn)... based on, of all things, the number of
        // motion listeners. Which is greatly upsetting. But realistically there are only a constant-ish number of
        // motion listeners - in fact, I think it's pretty much always just one. Then it's O(m/t), where m
        // is the number of drones and t is the physics granularity, which makes much more sense.

        if(Util.approx(Math.abs(vel2D()), Math.abs(oldVel), 0.1*t)) currentMotion = MotionType.CONSTANT;
        else if(Math.abs(vel2D()) > Math.abs(oldVel)) currentMotion = MotionType.ACCELERATING;
        else currentMotion = MotionType.DECELERATING;
    }

    /**
     * Returns the location (in all dimensions) currently represented by the motion state.
     * @return The location represented by the motion state.
     */
    public Point getLocation() {
        return new Point(state[0]);
    }

    /**
     * Sets the location (in as many dimensions as the input) represented by the motion state.
     * @param p A new location for the motion state.
     */
    public void setLocation(Point p) {
        state[0] = new double[state[0].length];
        for (int i = 0; i < p.dim(); i++) {
            state[0][i] = p.n(i);
        }
    }


    public double x() { return get(0, 0); }
    public double y() { return get(0, 1); }
    public double z() { return get(0, 2); }
    public double n(int n) { return get(0, n); }
    public double velX() { return get(1, 0); }
    public double velY() { return get(1, 1); }
    public double velZ() { return get(1, 2); }
    public double velN(int n) { return get(1, n); }
    public double accX() { return get(2, 0); }
    public double accY() { return get(2, 1); }
    public double accZ() { return get(2, 2); }
    public double accN(int n) { return get(2, n); }

    public void setX(double v) { set(0, 0, v); }
    public void setY(double v) { set(0, 1, v); }
    public void setZ(double v) { set(0, 2, v); }
    public void setN(int n, double v) { set(0, n, v); }
    public void setVelX(double v) {set(1, 0, v); }
    public void setVelY(double v) {set(1, 1, v); }
    public void setVelZ(double v) {set(1, 2, v); }
    public void setVelN(int n, double v) { set(1, n, v); }
    public void setAccX(double v) { set(2, 0, v); }
    public void setAccY(double v) { set(2, 1, v); }
    public void setAccZ(double v) { set(2, 2, v); }
    public void setAccN(double v) { set(2, 3, v); }

    /**
     * Sets an arbitrary component of the motion state in the (deriv-1)th derivative and the (var)th dimension. If
     * either the specified dimension or derivative are not tracked, an error is thrown.
     * @param deriv The level of motion to modify.
     * @param var The dimension of motion to modify.
     * @param value The new value for the specified component of motion.
     * @throws ArrayIndexOutOfBoundsException if either the specified dimension or derivative are not tracked.
     */
    public void set(int deriv, int var, double value) {
        if(state.length <= deriv) throw new ArrayIndexOutOfBoundsException("Not calulating derivative " + deriv);
        if(state[deriv].length <= var) throw new ArrayIndexOutOfBoundsException("Not tracking motion in dim " + var);
        state[deriv][var] = value;
    }

    /**
     * Yields an arbitrary component of the motion state in the (deriv-1)th derivative and the (var)th dimension. If
     * either the specified dimension or derivative are not tracked, an error is thrown.
     * @param deriv The level of motion to get.
     * @param var The dimension of motion to get.
     * @throws ArrayIndexOutOfBoundsException if either the specified dimension or derivative are not tracked.
     */
    public double get(int deriv, int var) {
        if(state.length <= deriv) throw new ArrayIndexOutOfBoundsException("Not calulating derivative " + deriv);
        if(state[deriv].length <= var) throw new ArrayIndexOutOfBoundsException("Not tracking motion in dim " + var);
        return state[deriv][var];
    }

    /**
     * Sets the XY heading of the motion state, which for the convenience of this project is tracked independently of
     * the rest of the motion state.
     * @param heading A new heading.
     */
    public void setHeading(double heading) { this.heading = heading; }

    /**
     * Gives the XY heading of the motion state, which for the convenience of this project is tracked independently of
     * the rest of the motion state.
     * @return The current XY heading represented by the motion state.
     */
    public double getHeading() { return heading; }

    /**
     * Convenience method which sets the XY acceleration represented by the state in a direction.
     * @param direction A direction of XY motion, in radians.
     * @param magnitude A magnitude of XY acceleration.
     */
    public void setAcc2D(double direction, double magnitude) {
        state[2][0] = magnitude * Math.cos(direction);
        state[2][1] = magnitude * Math.sin(direction);
    }

    /**
     * Convenience method which sets the XY velocity represented by the state in a direction.
     * @param direction A direction of XY motion, in radians.
     * @param magnitude A magnitude of XY velocity.
     */
    public void setVel2D(double direction, double magnitude) {
        state[1][0] = magnitude * Math.cos(direction);
        state[1][1] = magnitude * Math.sin(direction);
    }

    /**
     * Adds a listener to the motion state. When the state gets within dist XYZ distance of the given point,
     * the action is triggered and removed.
     * @param p Any point in the same number of dimensions as the motion state.
     * @param dist Some distance from the point at which to trigger the action.
     * @param action An action (which is given the current location of the state as an argument) to execute.
     */
    public void addListener(Point p, double dist, Consumer<Point> action) {
        listeners.add(new MotionListener(action, p, dist));
    }

    /**
     * Convenience class to manage all attributes of listeners on the motion state.
     */
    private static class MotionListener {
        private final Point point;
        private final Consumer<Point> action;
        private final double dist;

        private MotionListener(Consumer<Point> action, Point point, double dist) {
            this.action = action;
            this.point = point;
            this.dist = dist;
        }
    }

    /**
     * Gives the motion state as a string, showcasing its current components.
     * @return The motion state as a string.
     */
    public String toString() {
        StringBuilder s = new StringBuilder();
        for (double[] deriv : state) {
            for (double var : deriv) {
                s.append(var);
                s.append(" ");
            }
            s.append("\n");
        }
        return s.toString();
    }

    /**
     * Convenience function which gives the total XY velocity.
     * @return The XY velocity represented by the motion state.
     */
    public double vel2D() {
        double result = 0;
        for(double d: state[1]) result += d*d;
        return Math.sqrt(result);
    }

    /**
     * Convenience function which gives the total XY acceleration.
     * @return The XY acceleration represented by the motion state.
     */
    public double acc2D() {
        double result = 0;
        for(double d: state[2]) result += d*d;
        return Math.sqrt(result);
    }

    /**
     * Gives the current type of motion represented by the motion state: accelerating, deccelerating, or constant.
     * This is convenient for energy calculations.
     * @return The current type of motion of the motion state.
     */
    public MotionType getCurrentMotion() {
        return currentMotion;
    }
}
