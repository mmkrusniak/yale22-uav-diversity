package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Angle;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static com.mmkrusniak.drone.DroneUtil.optimizePlan;
import static com.mmkrusniak.drone.DroneUtil.plow;
/**
 * An offline drone which splits complex polygons into simpler ones by splitting concave vertices using a
 * minimalized version of Jiao's cellular decomposition. It performs well but the decomposition is very
 * time-intensive.
 */
public class JiaoDrone extends OfflineDrone {

    protected List<com.mmkrusniak.geom.Polygon> regions;
    protected com.mmkrusniak.geom.Polygon whole;
    private static final boolean VERBOSE = false;

    /**
     * Initializes the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public JiaoDrone(Area area, int id) {
        super(area, id);
    }

    /**
     * Initializes the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public JiaoDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    /**
     * Finishes off the decomposition process before planning out points.
     */
    public void preplan() {
        List<com.mmkrusniak.geom.Polygon> clone = new ArrayList<>(regions);
        regions = reorder(clone, clone.remove(0)).regions;
    }

    /**
     * Gives a preplanned route this drone will take (as it is an offline drone).
     * @return The route for the drone to take.
     */
    public List<Route> generatePlan() {
        if(getArea() == null) return null;
        whole = getPolygon();
        regions = decompose(whole, 400);
        preplan();

        List<Route> result = new ArrayList<>();

        com.mmkrusniak.geom.Point current = getArea().getStart(getID());
        for(com.mmkrusniak.geom.Polygon poly: regions) {
            result.add(new RouteHead(getPolygon().base().measure()));
            result.addAll(optimizePlan(plow(poly, current, DroneUtil.getCruiseAltitude(getArea()), -poly.girth().measure())));
            current = result.get(result.size()-1).getTarget();
        }
        return result;
    }

    /**
     * Splits a complex region into more easily traversible subregions. It is a recursive algorithm.
     * @param polygon A polygon to decompose.
     * @param depth The maximum depth to which to recurse. Decomposition also will stop before this when all subregions
     *              are acceptable.
     * @return A list of polygons which are easily traversed and whose union forms the original polygon.
     */
    public List<com.mmkrusniak.geom.Polygon> decompose(com.mmkrusniak.geom.Polygon polygon, int depth) {
        long lastStartTime = System.nanoTime();
        long maxTime = 2 * 1000 * 1000 * 1000;

        int n = polygon.numSides();

        Angle[] angles = polygon.toAngles();
        com.mmkrusniak.geom.Point[] points = polygon.toPoints();
        com.mmkrusniak.geom.Polygon[] bestSubregions = null;

        if(isAcceptable(polygon) || depth <= 0 || (polygon.getPolygonalWidth() < DroneUtil.getScanWidth(10))) {
            List<com.mmkrusniak.geom.Polygon> result = new ArrayList<>();
            result.add(polygon);
            return result;
        }

        double bestSumWidth = Double.MAX_VALUE;

        for(int i = 0; i < polygon.numSides(); i++) {
//            if(System.nanoTime() > lastStartTime + maxTime) {
//                System.err.println("Timeout");
//                break; // timeout
//            }
            if(! angles[i].isConcave()) {
                if(VERBOSE) System.out.printf("Angle %d (%1.2f rad) of the %dgon isn't concave\n", i, angles[i].measure(), polygon.numSides());
                continue;
            } else if(VERBOSE) System.out.printf(">> Trying angle %d (%1.2f rad)\n", i, angles[i].measure());


            for(int k = 0; k < polygon.toLines().length; k++) {
//                if(System.nanoTime() > lastStartTime + maxTime) {
//                    System.err.println("Timeout");
//                    break; // timeout
//                }
                Line l = polygon.toLines()[k];
                if(VERBOSE) System.out.println(">>>> parallel to edge " + k + " on the " + polygon.numSides() + "gon");
                if(l.isParallel(angles[i].ab()) || l.isParallel(angles[i].bc())) {
                    if(VERBOSE) System.out.println("Angle " + i + " can't use a parallel line " + k + " to split");
                    continue;
                }

                com.mmkrusniak.geom.Point guide = new com.mmkrusniak.geom.Point(points[i].x() + l.dx(),points[i].y() + l.dy());

                // If the guide is on the wrong side of the concave angle, a backwards ray will be generated.
                // So let's make sure that's not true.
                if(angles[i].containsPoint(guide, 0)) guide = new com.mmkrusniak.geom.Point(points[i].x() - l.dx(),points[i].y() - l.dy());

                Line splitLine = new Line(points[i], guide, Line.RAY);
                com.mmkrusniak.geom.Point intersection = polygon.intersection(splitLine);

                // There are a few reasons why an arbitrary line can't be used as a splitting line for a polygon.

                // First: The line doesn't intersect with any in the polygon. Although the rules we've already set
                // should prevent this, lines which are parallel are considered non-intersecting immediately, and fuzziness
                // on the definition of "parallel" can cause intersecting lines to be considered "parallel enough".
                if(intersection == null) {
                    if(VERBOSE) System.out.println("Angle " + i + " has no intersection parallel to edge " + k);
                    continue;
                }

                // WARNING: splitIndex indicates the index of the *line* intersected.
                // The index of the intersection, if it is added to the polygon, is one greater than that.
                int splitIndex = polygon.indexOf(intersection);

                if(splitIndex == (i-1+n)%n || splitIndex == i) {
                    if(VERBOSE) System.out.println("Angle " + i + " not allowed to connect to side " + splitIndex + ": too close");
                    continue;
                }

                com.mmkrusniak.geom.Polygon clone = polygon.addPoint(intersection, splitIndex+1);

                // WARNING: splitting into the polygon before i means that i now refers to a totally different vertex!

                com.mmkrusniak.geom.Polygon[] subregions = clone.split((splitIndex < i)? i+1 : i, splitIndex+1);

                if(VERBOSE) {
                    System.out.println("Angle " + i + " and a point in edge " + splitIndex + " creating a " + subregions[0].numSides() + "gon and a " + subregions[1].numSides() + "gon");
                    System.out.println(subregions[0]);
                    System.out.println(subregions[1]);
                }
                if(subregions[0].numSides() < 3 || subregions[1].numSides() < 3) {
                    if(VERBOSE) System.out.println("Angle " + i + " created an invalid polygon");
                    continue;
                }

                double sumWidth = subregions[0].getPolygonalWidth() + subregions[1].getPolygonalWidth();

                if(sumWidth < bestSumWidth) {
                    if(VERBOSE) System.out.println("Split from angle " + i + " to edge " + splitIndex + " parallel to edge " + k + " is the best so far: width sum " + sumWidth);
                    bestSubregions = subregions;
                    bestSumWidth = sumWidth;
                } else if(VERBOSE) System.out.println("Split from angle " + i + " to edge " + splitIndex + " parallel to edge " + k + " is subpar: width sum " + sumWidth);
            }
        }

        if(bestSubregions != null) {
            if(VERBOSE) System.out.printf("Decomposed into a %dgon and a %dgon\n", bestSubregions[0].numSides(), bestSubregions[1].numSides());
            List<com.mmkrusniak.geom.Polygon> result = new ArrayList<>();


            result.addAll(decompose(bestSubregions[0], depth-1));
            result.addAll(decompose(bestSubregions[1], depth-1));
            return result;
        }

        //well darn, guess we give up

        List<com.mmkrusniak.geom.Polygon> result = new ArrayList<>();
        result.add(polygon);
        return result;
    }
    /**
     * Paints a visualization of the drone on the supplied graphics instance. Good to modify for debugging; otherwise
     * does nothing to reduce visual clutter.
     * @param g A Java graphics canvas on which to paint.
     */
    public void visualize(Graphics g) {

    }

    /**
     * Gives the end condition for the decomposition. For Huang's algorithm, the polygon must only be simply
     * traversable (not necessarily concave).
     * @param p A polygon.
     * @return Whether the algorithm needs to decompose that polygon.
     */
    protected boolean isAcceptable(com.mmkrusniak.geom.Polygon p) { return ! p.isConcave(); }

    /**
     * An arrangement of polygonal regions associated with some cost.
     */
    protected static class Reordering {
        List<com.mmkrusniak.geom.Polygon> regions = new ArrayList<>();
        double cost = 0.0;
    }

    /**
     * Reorders a list of subregions into a more-easily-traversed set - more adjacent, perhaps.
     * @param regions A list of subregions.
     * @param start The location at which traversal will start.
     * @return A reordering of the subregions which is easier to traverse.
     */
    protected Reordering reorder(List<com.mmkrusniak.geom.Polygon> regions, com.mmkrusniak.geom.Polygon start) {

        if(regions.size() == 0) {
            Reordering result = new Reordering();
            result.regions.add(start);
            return result;
        }

        Reordering best = null;
        Reordering attempt;
        List<com.mmkrusniak.geom.Polygon> cloneList = new ArrayList<>(regions);

        for(Polygon p: regions) {

            cloneList.remove(p);
            attempt = reorder(cloneList, p);

            if(p.sharedPoints(start).size() >= 2) attempt.cost += 1.0; // adjacent!
            else attempt.cost += start.getCenter().distance2D(p.getCenter()); // not adjacent...

            if(best == null || attempt.cost < best.cost) {
                attempt.regions.add(start);
                best = attempt;
            }
            cloneList.add(p);
        }

        return best;
    }
}
