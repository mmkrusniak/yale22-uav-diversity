package com.mmkrusniak.drone.cooperative;

import com.mmkrusniak.drone.variable_height.VariableHeightDrone;
import com.mmkrusniak.frame.Area;
import com.mmkrusniak.frame.Detectable;
import com.mmkrusniak.frame.Option;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.geom.PolygonFactory;
import com.mmkrusniak.geom.Util;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.Broadcast;
import com.mmkrusniak.drone.Drone;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteTo;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * HotspotterDrone is an implementation of a height-variant, object-density-aware cooperative drone. The premise is to
 * split drones into three roles - one which explores high-density areas at a high altitude, one which visits
 * detected objects at a low altitude, and one which explores low-density areas.
 *
 * This version was implemented by Miles Krusniak, Angel Flores, and Alec James.
 *
 * Read the related 2020 Mizzou REU paper for more details.
 */
public class HotspotterDrone extends VariableHeightDrone {

    private static final int TRAVERSE = 0;
    private static final int OBSERVE = 1;
    private static final int EXPLORE = 2;

    private static final int HOTSPOT_SCALE = 10;
    private static final double HOTSPOT_THRESHOLD = 0.1;
    private static final double HOTSPOT_TOLERANCE = 0.04;

    private double cellWidth;
    private double cellHeight;

    private int numObserveDrones;
    private int numExploreDrones;
    private int numTraverseDrones;


    private int job;
    private List<com.mmkrusniak.geom.Polygon> hotspots = new ArrayList<>();
    private List<Route> path = new ArrayList<>();
    private final List<Broadcast> pendingBroadcasts = new ArrayList<>();

    public HotspotterDrone(Area area, int id) {
        super(area, id);
    }

    public HotspotterDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    @Override
    public void onBegin() {
        hotspots.clear();
        tree = new ArrayList<>();
        visibleTree = new ArrayList<>();
        path.clear();
        pendingBroadcasts.clear();
        int n = getTeamSize();
        int id = getID();
        int w = (int) getArea().getWidth()/HOTSPOT_SCALE;
        int h = (int) getArea().getHeight()/HOTSPOT_SCALE;

        byte[][] densityData = new byte[w][h];
        // So, we do have code to get a polygon out of a very tight grid... but it's *very* old code :P
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
                com.mmkrusniak.geom.Point p = new com.mmkrusniak.geom.Point(x*HOTSPOT_SCALE + 5, y*HOTSPOT_SCALE + 5);
                if(Util.approx(getArea().getDensity(p), HOTSPOT_THRESHOLD, HOTSPOT_TOLERANCE)
                || (getArea().getDensity(p) > HOTSPOT_THRESHOLD && getPolygon().distance(p) < 20)) {
                    densityData[x][y] = PolygonFactory.BORDER;
                } else if(getArea().getDensity(p) < HOTSPOT_THRESHOLD) densityData[x][y] = PolygonFactory.OUTSIDE;
                else densityData[x][y] = PolygonFactory.INSIDE;
            }
        }
        for(com.mmkrusniak.geom.Polygon poly: PolygonFactory.listFromData(densityData, 0.95)) {
            List<com.mmkrusniak.geom.Point> transform = new ArrayList<>();
            for(com.mmkrusniak.geom.Point p: poly.toPoints()) transform.add(new com.mmkrusniak.geom.Point(p.x()*HOTSPOT_SCALE, p.y()*HOTSPOT_SCALE));
            if(transform.size() > 2) hotspots.add(new com.mmkrusniak.geom.Polygon(transform));

        }

        // If there are no hotspots, we fall back to considering the entire area
        if(hotspots.isEmpty()) hotspots.add(getPolygon());

        double p = 0.5; // What percentage of drones are interior (traverse or observe)?
        double q = 0.3; // What percentage of drones are traverse, of those that are interior?

        double hotspotArea = 0;
        for(com.mmkrusniak.geom.Polygon poly: hotspots) hotspotArea += poly.area();
        p = Util.constrain(hotspotArea / getPolygon().area() * 4, 0.0, 1.0);


        numTraverseDrones = (int) (p * q * getTeamSize()) + 1;
        numObserveDrones = Math.max(0, (int) (p * getTeamSize()) - numTraverseDrones);
        numExploreDrones = getTeamSize() - numObserveDrones - numTraverseDrones;


        if(id < numObserveDrones) job = OBSERVE;
        else if(id < numObserveDrones + numTraverseDrones) job = TRAVERSE;
        else job = EXPLORE;

        if(job == EXPLORE) setupExterior();
        if(job == TRAVERSE) setupInterior();
        if(job == OBSERVE) {
            com.mmkrusniak.geom.Polygon target = hotspots.get(getID() % hotspots.size());
            path.add(new RouteTo(target.closest(getLocation())));
        }
    }


    public void setupInterior() {
        int indexWithinTraversers = getID() - numObserveDrones;
        com.mmkrusniak.geom.Polygon target = hotspots.get(getID() % hotspots.size());
        int numDronesOnTarget = numTraverseDrones / hotspots.size();
        if(hotspots.indexOf(target) < numTraverseDrones % hotspots.size()) numDronesOnTarget++;

        List<Route> totalPath = new ArrayList<>(
                DroneUtil.optimizePlan(
                        DroneUtil.plow(
                                target, getArea().getStart(getID()), DroneUtil.getCruiseAltitude(target,
                                        numDronesOnTarget*Drone.MAX_TRAVEL_DISTANCE),
                                -target.girth().measure())));
        totalPath.add(0, new RouteHead(target.base().measure()));
        int iStart = totalPath.size() / numDronesOnTarget * indexWithinTraversers;
        for (int i = 0; i < iStart; i++) {
            // only the last heading really counts, but we'll add them all
            if(totalPath.get(i).getTarget() == null) path.add(totalPath.get(i));
        }
        for(int i = iStart; i < iStart + totalPath.size()/numDronesOnTarget; i++) path.add(totalPath.get(i));
    }
    public void setupExterior() {
        double alt = DroneUtil.getCruiseAltitude(getPolygon())/numExploreDrones;
        double areaWidth = getPolygon().getCartesianBounds().width;
        double areaHeight = getPolygon().getCartesianBounds().height;
        cellWidth = 2*DroneUtil.getScanWidth(alt);
        cellHeight = 2*DroneUtil.getScanHeight(alt);

        Integer[][] grid = new Integer[(int) (areaWidth / cellWidth +1)][(int) (areaHeight / cellHeight +1)];

        for (int x = 0; x < grid.length; x++) {
            for (int y = 0; y < grid[x].length; y++) {
                com.mmkrusniak.geom.Point p = new com.mmkrusniak.geom.Point(x* cellWidth + cellWidth /2, y* cellHeight + cellHeight /2);
                boolean inArea = false;
                if(area.getHull().encloses(p) || area.getHull().distance(p) < cellWidth/4) inArea = true;
                for(com.mmkrusniak.geom.Polygon hotspot: hotspots) if(hotspot.encloses(p)) inArea = false;
                if(inArea) {
                    grid[x][y] = -2*(int) p.distance2D(area.getHull().closest(p));
                }
            }
        }


        int currentDrone = numTraverseDrones + numObserveDrones;
        com.mmkrusniak.geom.Point start = getArea().getStart(getID());
        GridNode current = new GridNode(
                Util.constrain((start.x() + cellWidth / 2) / cellWidth, 0, grid.length-1),
                Util.constrain((start.y() + cellHeight / 2) / cellHeight, 0, grid[0].length-1)
            );
        List<List<GridNode>> allTrees = new ArrayList<>();
        for (int i = numTraverseDrones + numObserveDrones; i <= getTeamSize(); i++) {
            allTrees.add(new ArrayList<>());
            if(i != currentDrone) allTrees.get(i-numTraverseDrones-numObserveDrones).add(new GridNode(current));
        }
        List<GridNode> singleTree = allTrees.get(currentDrone - numTraverseDrones - numObserveDrones);

        while (current != null) {
            grid[current.ix()][current.iy()] = null;
            singleTree.add(0, current);

            currentDrone = (currentDrone + 1);
            if(currentDrone >= getTeamSize()) currentDrone -= numExploreDrones;
            singleTree = allTrees.get(currentDrone - numTraverseDrones - numObserveDrones);

            GridNode bestPoint = null;
            GridNode bestStem = null;
            int bestValue = Integer.MIN_VALUE;

            // radiate out from the current point
            for (int rad = 0; bestPoint == null && rad < grid.length; rad++) {
                double branchPenalty = 0;
                for (GridNode stem : singleTree) {
//                    branchPenalty += getPolygon().getCartesianWidth() / 10;
                    List<com.mmkrusniak.geom.Point> candidates = Util.adjacent(grid, stem, rad);

                    for (com.mmkrusniak.geom.Point p : candidates) {
                        if (p.ix() >= grid.length || p.iy() >= grid[p.ix()].length || grid[p.ix()][p.iy()] == null)
                            continue;
                        if (bestPoint == null || (grid[p.ix()][p.iy()] + branchPenalty) > bestValue) {
                            bestPoint = new GridNode(p);
                            bestValue = grid[p.ix()][p.iy()];
                            bestStem = stem;
                        }
                    }
                }
            }
            current = bestPoint;
            if (bestStem != null) {
                bestStem.children.add(bestPoint);
                com.mmkrusniak.geom.Point a = new com.mmkrusniak.geom.Point(bestStem.x()*cellWidth + cellWidth/2, bestStem.y()*cellHeight + cellHeight/2);
                com.mmkrusniak.geom.Point b = new com.mmkrusniak.geom.Point(bestPoint.x()*cellWidth + cellWidth/2, bestPoint.y()*cellHeight + cellHeight/2);
                if(tree == null) tree = new ArrayList<>();
                if(currentDrone  == getID()) tree.add(new Line(a, b));
            }
        }


        renderGraph.clear();
        List<GridNode> ourTree = allTrees.get(getID() - numTraverseDrones - numObserveDrones);
        System.out.println("Using tree #" + (getID() - numTraverseDrones - numObserveDrones) + " of size " + ourTree.size());
        for(com.mmkrusniak.geom.Point p: ourTree.get(ourTree.size()-1).pathAround(new ArrayList<>(), 0)) path.add(new RouteTo(new com.mmkrusniak.geom.Point(
                p.x()*cellWidth + cellWidth/2,
                p.y()*cellHeight + cellHeight/2, alt)));
        path.add(0, new RouteHead(Math.PI/2));
    }

    private final List<Line> renderGraph = new ArrayList<>();
    private class GridNode extends com.mmkrusniak.geom.Point {
        List<GridNode> children = new ArrayList<>();
        GridNode(double... coords) { super(coords); }
        GridNode(com.mmkrusniak.geom.Point p) { super(p.x(), p.y(), p.z()); }
        boolean visited = false;
        private List<com.mmkrusniak.geom.Point> pathAround(List<com.mmkrusniak.geom.Point> existing, double dirIn) {
            if (visited) {
                System.err.println("Error: already visited this node!");
                return existing;
            }

            for(GridNode g: children) renderGraph.add(new Line(this, g));
            visited = true;

            List<GridNode> q1 = new ArrayList<>();
            List<GridNode> q2 = new ArrayList<>();
            List<GridNode> q3 = new ArrayList<>();
            List<GridNode> q4 = new ArrayList<>();
            for(GridNode g: children) if(Util.within(1*Math.PI/4, 3*Math.PI/4, bearing(g))) q2.add(g);
            for(GridNode g: children) if(Util.within(3*Math.PI/4, 5*Math.PI/4, bearing(g))) q3.add(g);
            for(GridNode g: children) if(Util.within(5*Math.PI/4, 7*Math.PI/4, bearing(g))) q4.add(g);
            for(GridNode g: children) if(Util.within(7*Math.PI/4, 2*Math.PI,   bearing(g))) q1.add(g);
            for(GridNode g: children) if(Util.within(0, Math.PI/4, bearing(g))) q1.add(g);

            com.mmkrusniak.geom.Point returnPoint;
            for (double r = dirIn; r < dirIn+Math.PI*2; r+= Math.PI/2) {
                double a = r % (Math.PI*2);
                double b = (r+Math.PI/2) % (Math.PI*2);
                if(Util.within(Math.PI/4, 3*Math.PI/4, a)) {
                    returnPoint = new com.mmkrusniak.geom.Point(x() - 0.25, y()+0.25);
                    for(GridNode g: q2) {
                        g.pathAround(existing, g.bearing(this));
                        existing.add(returnPoint);
                    }
                }
                else if(Util.within(3*Math.PI/4, 5*Math.PI/4, a)) {
                    returnPoint = new com.mmkrusniak.geom.Point(x() - 0.25, y()-0.25);
                    for(GridNode g: q3) {
                        g.pathAround(existing, g.bearing(this));
                        existing.add(returnPoint);
                    }
                }
                else if(Util.within(5*Math.PI/4, 7*Math.PI/4, a)) {
                    returnPoint = new com.mmkrusniak.geom.Point(x() + 0.25, y() - 0.25);
                    for (GridNode g : q4) {
                        g.pathAround(existing, g.bearing(this));
                        existing.add(returnPoint);
                    }
                }
                else {
                    returnPoint = new com.mmkrusniak.geom.Point(x() + 0.25, y() + 0.25);
                    for (GridNode g : q1) {
                        g.pathAround(existing, g.bearing(this));
                        existing.add(returnPoint);
                    }
                }

                existing.add(returnPoint);
            }
            return existing;
        }
    }

    @Override
    public void onTick() {
        List<Broadcast> removable = new ArrayList<>();
        for(Broadcast b: pendingBroadcasts) {
            if(transmit(b.rebroadcast(getTime()))) removable.add(b);
        }
        pendingBroadcasts.removeAll(removable);
    }

    @Override
    public void onAwaiting() {
        List<Detectable> detected = scan().detectables;

        if(! detected.isEmpty()) {
            Drone observer = getLocalObserveDrone(getLocation(), 100.0);
            if(observer != null) {
                pendingBroadcasts.add(new Broadcast(this, observer, "goto", detected.toArray(new com.mmkrusniak.geom.Point[0]), getTime()));
            } else if(job != OBSERVE) {
                if(! path.isEmpty() && path.get(0).getTarget() != null)
                    DroneUtil.heuristicTSP(detected, getLocation(), path.get(0).getTarget());
                for(Detectable d: detected) {
                    if(d.detectedFrom() <= Math.max(altitudeNeeded(d), 12)) continue;
                    path.add(0, new RouteTo(new com.mmkrusniak.geom.Point(d.x(), d.y(), Math.max(10, altitudeNeeded(d)))));
                }
            }
        }

        if(! path.isEmpty()) {
            Route next = path.remove(0);
            if(next == null) return;
            move(next);
        } else if(job == OBSERVE) {
            Drone d = nearestNonObserveDrone();
            if(d != null) move(new RouteTo(new com.mmkrusniak.geom.Point(d.getLocation().x(), d.getLocation().y(), 10)));
        } else job = OBSERVE;
    }

    private Drone nearestNonObserveDrone() {
        Drone nearest = null;
        for(Drone d: getNeighbors()) {
            if(! (d instanceof HotspotterDrone)) continue;
            HotspotterDrone hd = (HotspotterDrone) d;
            if(hd.job == OBSERVE) continue;
            if(nearest == null || nearest.getLocation().distance3D(getLocation()) > d.getLocation().distance3D(getLocation()))
                nearest = d;
        }
        return nearest;
    }

    private List<Drone> getObserveDrones() {
        List<Drone> result = new ArrayList<>();
        for(Drone d: getNeighbors()) {
            if(d instanceof HotspotterDrone && ((HotspotterDrone) d).job == OBSERVE) result.add(d);
        }
        return result;
    }

    private Drone getLocalObserveDrone(com.mmkrusniak.geom.Point point, double dist) {
        if(numObserveDrones == 0) return null;
        if(job == OBSERVE) return null;

        Drone best = null;
        for(com.mmkrusniak.geom.Polygon poly: hotspots) if(poly.encloses(point)) {
            for(Drone d: getObserveDrones()) {
                if((d.isDone() || poly.encloses(d.getLocation()) || poly.distance(d.getLocation()) < dist) &&
                        (best == null || d.isDone() || point.distance3D(d.getLocation()) < point.distance3D(best.getLocation()))) {
                    best = d;
                }
            }
        }
        return best;
    }

    @Override
    public void onMissionCancel() {

    }

    @Override
    public void onEnergyDepleted() {

    }

    @Override
    public void onBroadcastReceived(Broadcast broadcast) {
        // hypothetically any drone can receive a goto message
        if(broadcast.getHeader().equals("goto")) {
            List<com.mmkrusniak.geom.Point> graph = new ArrayList<>(Arrays.asList((com.mmkrusniak.geom.Point[]) broadcast.getPayload()));
            for(Route r: path) if(r.getTarget() != null) graph.add(r.getTarget());
            if(getCurrentRoute() != null && getCurrentRoute().getTarget() != null) graph.add(getCurrentRoute().getTarget());
            DroneUtil.heuristicTSP(graph, getLocation(), getArea().getStart(getID()));
            path.clear();
            for(com.mmkrusniak.geom.Point p: graph) path.add(new RouteTo(new com.mmkrusniak.geom.Point(p.x(), p.y(), 10)));
            move(path.remove(0));
        }
    }

    @Override
    public boolean isDone() {
        return job == OBSERVE && path.isEmpty();
    }


    private long renderTree = 0;
    private List<Line> tree;
    private List<Line> visibleTree;
    @Override
    public void visualize(Graphics g) {
//        double alt = DroneUtil.getCruiseAltitude(getPolygon())/numExploreDrones;
//        cellWidth = 2*DroneUtil.getScanWidth(alt);
//        cellHeight = 2*DroneUtil.getScanHeight(alt);
//
        g.setColor(Color.BLACK);
        for(Polygon poly: hotspots) poly.render(g);
        g.setColor(getColor());
//
//        if(job == EXPLORE) {
//            for (Line l : renderGraph) {
//                Line r = new Line(
//                        new Point(
//                                l.a().x() * cellWidth + cellWidth / 2,
//                                l.a().y() * cellHeight + cellHeight / 2),
//                        new Point(
//                                l.b().x() * cellWidth + cellWidth / 2,
//                                l.b().y() * cellHeight + cellHeight / 2)
//                );
//                r.render(g);
//            }
//        }
//
//        if(job == OBSERVE) {
//            for(Route r: path) {
//                Point p = r.getTarget();
//                if(p == null) continue;
//                p.render(g);
//            }
//        }

        if(visibleTree == null) visibleTree = new ArrayList<>();
        for(Line l: visibleTree) l.render(g);

        if(renderTree < System.currentTimeMillis() + 0.1) {
            if(tree == null || tree.isEmpty()) return;
            visibleTree.add(tree.remove(0));
            renderTree = System.currentTimeMillis();
        }
    }

    // TODO: This should really be moved to HeightVariantDrone
    private double altitudeNeeded(Detectable d) {
        double k = d.detectedFrom() * Math.abs(d.confidence() - Option.get("detection", "threshold", Double.class));
        return Math.max(k, 10);
    }

    @Override
    public String toHTML() {
        String result = super.toHTML();
        String jobString = "???";
        if(job == OBSERVE) jobString = "Observe";
        if(job == EXPLORE) jobString = "Explore";
        if(job == TRAVERSE) jobString = "Traverse";
        result += String.format("<div><h3>Hotspotter info</h3><div>Job: %s", jobString);
        Drone d = getLocalObserveDrone(getLocation(), 100.0);
        result += String.format("<div>Local observe drone: %s<div>", (d == null)? "None" : "" + d.getID());
        return result;
    }
}
