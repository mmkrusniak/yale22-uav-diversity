package com.mmkrusniak.drone;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Input;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.phys.EnergyModel;
import com.mmkrusniak.phys.MotionState;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public abstract class Drone {

    // From the spec sheet on the Iris Quadcopter:
    public static final double FOV_HEIGHT               = Math.toRadians(78.8);              // radians
    public static final double FOV_WIDTH                = Math.toRadians(105.1);             // radians
    public static final double MAX_TRAVEL_DISTANCE      = 10549.44;                          // meters
    public static final double ENERGY_CAPACITY          = 219780.0;                          // joules
    public static final double COMM_RANGE               = 6900;                              // meters

    private final int id; // for broadcasting
    private double energyConsumed;
    private final double energyBudget;
    private double energyDraw;
    private EnergyModel energyModel;

    private DroneTeam team;

    private double pathScore;

    /** A list of broadcasts this drone has received. May or may not be useful. */
    private final List<Broadcast> pendingTransmissions = new CopyOnWriteArrayList<>();
    private final List<Broadcast> broadcastHistory = new CopyOnWriteArrayList<>();

    /** A map of neighbors to their respective distances away from this drone.
     * This is probably not the most efficient thing to do, as we are storing extra integers.
     */

    // Information about the area of interest
    public Area area;
    private final List<Capture> captureHistory = new CopyOnWriteArrayList<>();
    private boolean running;

    // The route the drone has taken, once it has been calculated for the first time
    private final List<com.mmkrusniak.geom.Point> trace = new CopyOnWriteArrayList<>();
    private MotionState motionState;
    private Route currentRoute;

    private double time;

    public Drone(Area area, int id) {
        this(area, id, ENERGY_CAPACITY);
    }

    public Drone(Area area, int id, double energyBudget) {
        this.area = area;
        this.id = id;
        this.energyBudget = energyBudget;
        this.energyConsumed = 0;
        this.motionState = new MotionState(3, 3);
        reset();
    }

    public com.mmkrusniak.geom.Polygon getPolygon() {
        return area.getHull();
    }
    public Area getArea() {
        return area;
    }
    public com.mmkrusniak.geom.Point getLocation() {
        return motionState.getLocation();
    }
    public double getHeading() {
        return motionState.getHeading();
    }
    public List<Capture> getCaptureHistory() {
        return captureHistory;
    }
    public int getID() {
        return id;
    }

    protected Capture scan() {
//        if(getLocation().z() == 0) System.err.println("[Drone] Warning: " + this + " scanning at altitude 0; will " +
//                "have no effect.");
        double w = 2 * getLocation().z() * Math.tan(FOV_WIDTH/2);
        double h = 2 * getLocation().z() * Math.tan(FOV_HEIGHT/2);
        com.mmkrusniak.geom.Polygon p = new com.mmkrusniak.geom.Polygon(
                new com.mmkrusniak.geom.Point(getLocation().x() + w/2, getLocation().y() + h/2),
                new com.mmkrusniak.geom.Point(getLocation().x() + w/2, getLocation().y() - h/2),
                new com.mmkrusniak.geom.Point(getLocation().x() - w/2, getLocation().y() - h/2),
                new com.mmkrusniak.geom.Point(getLocation().x() - w/2, getLocation().y() + h/2)
        ).rotate(motionState.getHeading()+Math.PI/2);

        // Edge case: If the drone is done traversing, it never receives anything in a scan.
        // This is because calculating what points fall in a scan is time-consuming.
        if(isDone()) return new Capture(p, new ArrayList<>(), getLocation());
        List<Detectable> detected = area.getDetectables(p, getLocation().z());
        Capture c = new Capture(p, detected, getLocation());
        captureHistory.add(c);
        return c;
    }
    public void move(com.mmkrusniak.geom.Point p) {
        // If you're moving to a two-dimensional location, chances are you meant to stay at the current altitude
        if(p.dim() == 2) p = p.extend(getLocation().z());
        this.currentRoute = new RouteTo(p);
        motionState.addListener(p, 5, q -> currentRoute = null);
    }
    public void move(Route r) {
        this.currentRoute = r;
        motionState.addListener(r.getTarget(), 5, q -> currentRoute = null);
    }
    public void reset() {
        if(area == null) return;
        trace.clear();
        captureHistory.clear();
        com.mmkrusniak.geom.Point start = area.getStart(id);
        motionState = new MotionState(new com.mmkrusniak.geom.Point(start.x(), start.y(), 0), 3);
        this.energyConsumed = 0;
        this.energyModel = EnergyModel.iris();
        trace.add(getLocation());
        this.running = false;
        this.time = 0;
        this.pathScore = 0;
        this.motionState.setZ(30);
    }

    public void setHeading(double h) { motionState.setHeading(h);}
    public void joinTeam(DroneTeam team) {
        this.team = team;
    }

    public void reset(Area area) {
        if(area == null) return;
        this.area = area;
        reset();
    }

    public abstract void onBegin();
    public abstract void onTick();
    public abstract void onAwaiting();
    public abstract void onMissionCancel();
    public abstract void onEnergyDepleted();
    public abstract void onBroadcastReceived(Broadcast broadcast);
    public abstract boolean isDone();
    public abstract void visualize(Graphics g);
    public abstract boolean predict(Detectable d);

    public void proceed(double t) {
        if(! running && (energyBudget - energyConsumed) > 0) {
            onBegin();
            onAwaiting();
            running = true;
        } else {
            if((energyBudget - energyConsumed) < 0) {
                onEnergyDepleted();
                running = false;
            } else  {
                pathScore += t * area.getDensity(getLocation())*10000;
                time += t;
                // If there are a lot of links in the trace, rendering gets slow, so we'll remove them if they're
                // collinear. (But only if there are lots, otherwise important points can disappear!)
                if(trace.size() > 10 && trace.get(trace.size()-5).distance3D(new Line(trace.get(trace.size()-10),
                        getLocation())) < .01) trace.remove(trace.size()-5);
                trace.add(getLocation());
                onTick();
                if(currentRoute != null) currentRoute.apply(motionState, t);
                else {
                    motionState.setAcc2D(0, 0);
                    onAwaiting();
                }
                energyModel.constrain(motionState);
                energyDraw = energyModel.getCost(motionState, t)/t;
                energyConsumed += energyDraw * t;
                motionState.advance(t);
            }
        }
    }

    @Override
    public String toString() {
        return this.getClass().getSimpleName();
    }
    public void render(Graphics g1) {
        if(area == null) return;
        Graphics2D g = (Graphics2D) g1;
        g.setFont(new Font("Helvetica", Font.PLAIN, 14));
        g.setStroke(new BasicStroke(3));

        // 71's a nice prime number to let us wrap hues and have them be unique-ish when we have five or so drones
        Color color = getColor();
        Color colorPale = Color.getHSBColor((getID()*71)%256/256.0f, 0.2f, 0.6f);
        colorPale = new Color(colorPale.getRed(), colorPale.getGreen(), colorPale.getBlue(), 40);

        if(trace.isEmpty()) {
            g.setColor(Color.RED.darker());
            g.drawString("Error: That algorithm cannot handle this area.", 5, (int) (area.getHeight()-5));
            return;
        }

        if(Option.get("graphics", "captures", Boolean.class)) {
            for (Capture p : captureHistory) {
                for (Detectable d : p.detectables) {
                    if (predict(d)) g.setColor(Color.CYAN);
                    else g.setColor(Color.ORANGE);
                    g.drawOval(d.ix() - 6, d.iy() - 6, 12, 12);

                    if (Input.mouse.sqDistance2D(d) < 100) {
                        if (predict(d)) g.setColor(new Color(0, 80, 80));
                        else g.setColor(new Color(100, 40, 0));

                        double scale = g.getTransform().getScaleX();
                        g.scale(1 / scale, 1 / scale);
                        g.drawString(
                                String.format("(%dm: %2.0f%%)", (int) d.detectedFrom(), d.confidence() * 100),
                                Input.mouseRaw.x + 5,
                                (int) (Input.mouseRaw.y + d.detectedFrom() + 20)
                        );
                        g.scale(scale, scale);
                    }
                }
            }
            g.setColor(colorPale);
            for (Capture p : captureHistory) {
                g.fillPolygon(p.view.toAWTPolygon());
            }
        }

        g.setColor(Color.BLACK);
        if(currentRoute != null && currentRoute.getTarget() != null) currentRoute.getTarget().render(g);

        g.setColor(color);
        if(Option.get("graphics", "trace", Boolean.class)) {
            for (Line l : Line.arrayFromPoints(trace.toArray(new Point[0]))) {
//                g.setStroke(new BasicStroke((int) (l.a().z() / 10)));
                l.render(g);
            }
        }

        if(Option.get("graphics", "broadcast_indicators", Boolean.class)) {
            g.setStroke(new BasicStroke(1));
            int rad = (int) COMM_RANGE / 2;
            g.drawOval(getLocation().ix() - rad, getLocation().iy() - rad, 2 * rad, 2 * rad);
            if (renderTransmission > 0) {
                renderTransmission--;
                g.drawOval(getLocation().ix() - 40, getLocation().iy() - 40, 2 * 40, 2 * 40);
            }
            g.fillOval(getLocation().ix(), getLocation().iy(), 5, 5);
        }

        visualize(g);
        g.setColor(color);
        getLocation().render(g);
    }

    public void forceHalt() { energyConsumed = energyBudget+1; }

    public Color getColor() {
        return Color.getHSBColor((getID()*71)%256/256.0f, 0.6f, 0.6f);
    }

    /** Get my collection of neighbors.
     * In the real world, this would be faster as a drone would just flood a 'hello' message
     * and receive responses from various drones almost simultaneously, but since we're working in
     * a simulator this is more represented by trying to unicast to every drone.
     * Assume that every drone has the same communication range.
     * Runtime depends on the collection.
     * i.e. HashSet will run in theta(n) time, and a MaxHeap will run in theta(n log n) time,
     * where n is the number of drones in the swarm. */
    public List<Drone> getNeighbors() {
        Collection<Drone> neighbors = new ArrayList<>();
        for(Drone d: team) {
            if(d == this) continue; // Don't include the source drone
            if (d.getLocation().sqDistance2D(getLocation()) <= COMM_RANGE*COMM_RANGE) {
                neighbors.add(d);
            }
        }
        return new ArrayList<>(neighbors);
    }

    public double getEnergyRemaining() { return (energyBudget - energyConsumed); }

    public String toHTML() {
        return (String.format(
                "<h2>Drone #%d</h2>" +
                        "<h3>Physics</h3>" +
                        "Elapsed time: %d s<div>" +
                        "Position: " + getLocation() + "<div>" +
                        "Velocity: %.2f m/s<div>" +
                        "Acceleration: %.2f m/s<sup>2</sup><div>" +
                        "Target: %s<div>" +
                        "<h3>Energy</h3>" +
                        "Draw: %.2f W<div>" +
                        "Consumed: %.2f J<div>" +
                        "Remaining: %.2f J (%.2f%%)<div>" +
                        "Budget: %.2f J<div>" +
//                        "<h3>Predictions</h3>" +
//                        "Precision: %.2f<div>" +
//                        "Recall: %.2f<div>" +
//                        "F1 score: %.2f<div>" +
                        "<h3>Trajectory</h3>" +
                        "Path score: %.5f<div>" +
                        "Expected path score: %.5f<div>"
                , getID(), (int) time, motionState.vel2D(), motionState.acc2D(),
                (currentRoute == null || currentRoute.getTarget() == null)?
                        "None" : currentRoute.getTarget().toString(),
                energyDraw, energyConsumed, getEnergyRemaining(), 100*getEnergyRemaining()/energyBudget, energyBudget,
//                getPrecision(), getRecall(), getF1Score()
                getPathScore(), approxPathScore(trace)
                ));
    }

    /** Send a message to a destination drone from a source drone (AKA this).
     *  This is a decentralized method, where drones are working independently.
     *
     * This is somewhat of a simplified version of ad-hoc on demand distance vector routing (AODV),
     * mainly because AODV looks like it would be difficult to implement, especially on the first try.
     * However, this should be a reliable framework that AODV can be built out of, if we decide to do so.
     * @param broadcast The broadcast we're sending.
     * This is the DFS (Depth-First Search) implementation. */
    private int renderTransmission = 0;
    public boolean transmit(Broadcast broadcast) {
        renderTransmission = 30;
        if (pendingTransmissions.contains(broadcast) || broadcastHistory.contains(broadcast)) return false;
        if(broadcast.getDestination() == null || broadcast.getDestination() == this) {
            broadcastHistory.add(broadcast);
            onBroadcastReceived(broadcast);
        }
        if(broadcast.getDestination() == this) return true;
        pendingTransmissions.add(broadcast);
        boolean success = false;
        for (Drone drone : getNeighbors()) {
            if(drone.transmit(broadcast)) success = true;
        }
        return success;
    }

    public double getTime() {
        return time;
    }

    public double getEnergyBudget() { return energyBudget; }

    public double getPathScore() {
        return pathScore;
    }

    public double approxPathScore(List<Point> points) {
        for(Point p: points) {
            if(! area.contains(p) && area.getHull().distance(p) > 30) return Double.POSITIVE_INFINITY;
        }

        // Ensure start and end are in the list
        if(! points.get(0).equals(area.getStart(getID()))) {
            ArrayList<Point> newPoints = new ArrayList<>();
            newPoints.add(area.getStart(getID()));
            newPoints.addAll(points);
            points = newPoints;
        }
        if(! points.get(0).equals(area.getDest(getID()))) {
            ArrayList<Point> newPoints = new ArrayList<>(points);
            newPoints.add(area.getDest(getID()));
            points = newPoints;
        }

        double score = 0;
        for(Line l: Line.arrayFromPoints(points.toArray(new Point[0]))) {
            for(Point p: l.toSubpoints(1)) {
                double density = area.getDensity(p);
                if(! area.contains(p)) density = 1;
                score += 10000*density;
            }
            // a relatively small incentive to keep points equally spaced
            score += l.length() * l.length();
        }
        return score; // varies positively with true path score?
    }

    public double getPrecision() {
        int truePos = 0;
        int falsePos = 0;
        for(Detectable d: area.getDetectables()) {
            if(predict(d) && d.real()) truePos++;
            if(predict(d) && ! d.real()) falsePos++;
        }
        if(truePos + falsePos == 0) return 0;
        return ((double) truePos) / (truePos + falsePos);
    }
    public double getRecall() {
        int truePos = 0;
        int falseNeg = 0;
        for(Detectable d: area.getDetectables()) {
            if(predict(d) && d.real()) truePos++;
            if(! predict(d) && d.real()) falseNeg++;
        }
        if(truePos + falseNeg == 0) return 0;
        return ((double) truePos) / (truePos + falseNeg);
    }
    public double getF1Score() {
        double p = getPrecision();
        double r = getRecall();
        return (2*p*r)/(p+r);
    }

    public List<Detectable> getTeamDetectionHistory() {
        List<Detectable> result = new ArrayList<>();
        for(Drone drone: team) {
            for (Capture capture : drone.getCaptureHistory()) {
                result.addAll(capture.detectables);
            }
        }
        return result;
    }

    public int getTeamSize() { return team.size(); }
    public Route getCurrentRoute() { return currentRoute; }
}
