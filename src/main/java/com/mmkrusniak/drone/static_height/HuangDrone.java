package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Angle;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.drone.DroneUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * An offline drone which splits complex polygons into simpler ones by splitting concave vertices using Huang's
 * cellular decomposition. It is similar to Jiao's but somewhat more thorough (it is exponentially
 * recursive rather than polynomial). It performs well but the decomposition is very time-intensive.
 */
public class HuangDrone extends CombinatoryJiaoDrone {
    private static final boolean VERBOSE = false;

    /**
     * Initializes the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public HuangDrone(Area area, int id) {
        super(area, id);
    }

    /**
     * Initializes the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public HuangDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    /**
     * Gives the end condition for the decomposition. For Huang's algorithm, the polygon must only be simply
     * traversable (not necessarily concave).
     * @param p A polygon.
     * @return Whether the algorithm needs to decompose that polygon.
     */
    @Override
    public boolean isAcceptable(Polygon p) {
        return p.isSimplyTraversable();
    }

    /**
     * Joins the regions created by cellular decomposition.
     */
    @Override
    public void recompose() {
        for (int i = 0; i < regions.size(); i++) {
            for (int j = i; j < regions.size(); j++) {
                Polygon a = regions.get(i);
                Polygon b = regions.get(j);
                if (a == b) continue;
                if (a.sharedPoints(b).size() == 2
                        && a.girth().isParallel(b.girth(), 0.5)) {
                    Polygon combination = a.combine(b);
                    if(combination.isSimplyTraversable()) {
                        regions.remove(a);
                        regions.remove(b);
                        regions.add(a.combine(b));
                        i = 0;
                        j = 0;
                    }
                }
            }
        }
    }

    // Huang's algorithm is deeply recursive (exponential), so we need to keep track of more state.
    private static class Decomposition {
        List<Polygon> regions;
        double widthSum;

        Decomposition(List<Polygon> regions, double widthSum) {
            this.regions = regions;
            this.widthSum = widthSum;
        }
    }

    /**
     * Splits a complex region into more easily traversible subregions. It is a recursive algorithm.
     * @param polygon A polygon to decompose.
     * @param depth The maximum depth to which to recurse. Decomposition also will stop before this when all subregions
     *              are acceptable.
     * @return A list of polygons which are easily traversed and whose union forms the original polygon.
     */
    @Override
    public List<Polygon> decompose(Polygon polygon, int depth) {
        lastStartTime = System.nanoTime();
        maxTime = 2 * 1000 * 1000 * 1000;
        Decomposition decomposition = complexDecompose(polygon, depth);
        if(decomposition == null) return null;
        return decomposition.regions;
    }


    // The actual details are abstracted here. I'm not willing to spend the time to document them, but it could use a
    // rewrite.
    private long lastStartTime;
    private long maxTime;
    private Decomposition complexDecompose(Polygon polygon, int depth) {

        int n = polygon.numSides();

        Angle[] angles = polygon.toAngles();
        Point[] points = polygon.toPoints();
        Decomposition bestDecomposition = null;

        if (isAcceptable(polygon) || depth <= 0 || (polygon.getPolygonalWidth() < DroneUtil.getScanWidth(10))) {
            List<Polygon> result = new ArrayList<>();
            result.add(polygon);
            return new Decomposition(result, polygon.getPolygonalWidth());
        }

        for (int i = 0; i < polygon.numSides(); i++) {

            if(System.nanoTime() > lastStartTime + maxTime) {
                System.err.println("Timeout");
                break; // timeout
            }

            if (! polygon.isConcave()) {
                if (VERBOSE)
                    System.out.printf("Angle %d (%1.2f rad) of the %dgon is already OK\n", i, angles[i].measure(), polygon.numSides());
                continue;
            } else if (VERBOSE) System.out.printf(">> Trying angle %d (%1.2f rad)\n", i, angles[i].measure());


            for (int k = 0; k < polygon.toLines().length; k++) {

                if(System.nanoTime() > lastStartTime + maxTime){
                    System.err.println("Timeout");
                    break; // timeout
                }

                Line l = polygon.toLines()[k];
                if (VERBOSE) System.out.println(">>>> parallel to edge " + k + " on the " + polygon.numSides() + "gon");
                if (l.isParallel(angles[i].ab()) || l.isParallel(angles[i].bc())) {
                    if (VERBOSE) System.out.println("Angle " + i + " can't use a parallel line " + k + " to split");
                    continue;
                }

                Point guide = new Point(points[i].x() + l.dx(), points[i].y() + l.dy());

                // If the guide is on the wrong side of the concave angle, a backwards ray will be generated.
                // So let's make sure that's not true.
                if (angles[i].containsPoint(guide, 0)) guide = new Point(points[i].x() - l.dx(),
                        points[i].y() - l.dy());

                Line splitLine = new Line(points[i], guide, Line.RAY);
                Point intersection = polygon.intersection(splitLine);

                // There are a few reasons why an arbitrary line can't be used as a splitting line for a polygon.

                // First: The line doesn't intersect with any in the polygon. Although the rules we've already set
                // should prevent this, lines which are parallel are considered non-intersecting immediately, and fuzziness
                // on the definition of "parallel" can cause intersecting lines to be considered "parallel enough".
                if (intersection == null) {
                    if (VERBOSE) System.out.println("Angle " + i + " has no intersection parallel to edge " + k);
                    continue;
                }

                // WARNING: splitIndex indicates the index of the *line* intersected.
                // The index of the intersection, if it is added to the polygon, is one greater than that.
                int splitIndex = polygon.indexOf(intersection);

                if (splitIndex == (i - 1 + n) % n || splitIndex == i) {
                    if (VERBOSE)
                        System.out.println("Angle " + i + " not allowed to connect to side " + splitIndex + ": too close");
                    continue;
                }

                Polygon clone = polygon.addPoint(intersection, splitIndex + 1);

                // WARNING: splitting into the polygon before i means that i now refers to a totally different vertex!

                Polygon[] subregions = clone.split((splitIndex < i) ? i + 1 : i, splitIndex + 1);

                if (VERBOSE) {
                    System.out.println("Angle " + i + " and a point in edge " + splitIndex + " creating a " + subregions[0].numSides() + "gon and a " + subregions[1].numSides() + "gon");
                    System.out.println(subregions[0]);
                    System.out.println(subregions[1]);
                }
                if (subregions[0].numSides() < 3 || subregions[1].numSides() < 3) {
                    if (VERBOSE) System.out.println("Angle " + i + " created an invalid polygon");
                    continue;
                }

                // Time to actually decompose. Strap in and pray!
                if(VERBOSE) System.out.println("Angle " + i + " and edge " + k + " are promising. Now decomposing a " + subregions[0].numSides() + "gon and a " + subregions[1].numSides() + "gon.");

                List<Polygon> potentialResult = new ArrayList<>();
                Decomposition dec1 = complexDecompose(subregions[0], depth-1);
                Decomposition dec2 = complexDecompose(subregions[1], depth-1);

                // this is bad, this is bad
                if(dec1 == null || dec2 == null) continue;

                potentialResult.addAll(dec1.regions);
                potentialResult.addAll(dec2.regions);
                Decomposition decomposition = new Decomposition(potentialResult, dec1.widthSum + dec2.widthSum);


                if (bestDecomposition == null || decomposition.widthSum < bestDecomposition.widthSum) {
                    if (VERBOSE)
                        System.out.println("Split from angle " + i + " to edge " + splitIndex + " parallel to edge " + k + " is the best so far: width sum " + decomposition.widthSum);
                    bestDecomposition = decomposition;
                } else if (VERBOSE)
                    System.out.println("Split from angle " + i + " to edge " + splitIndex + " parallel to edge " + k + " is subpar: width sum " + decomposition.widthSum);
            }
        }

        if(bestDecomposition != null) {
            if(VERBOSE) System.out.println("Decomposed successfully! Depth = " + depth + "\n");
            return bestDecomposition;
        }

        //well darn

        // This is not good, but we'll struggle along anyway
        return null;
    }


    /**
     * Reorders a list of subregions into a more-easily-traversed set - more adjacent, perhaps.
     * @param regions A list of subregions.
     * @param start The location at which traversal will start.
     * @return A reordering of the subregions which is easier to traverse.
     */
    public Reordering reorder(List<Polygon> regions, Polygon start) {

        if(regions.size() == 0) {
            Reordering result = new Reordering();
            result.regions.add(start);
            return result;
        }

        Reordering best = null;
        Reordering attempt;
        List<Polygon> cloneList = new ArrayList<>(regions);

        for(Polygon p: regions) {

            cloneList.remove(p);
            attempt = reorder(cloneList, p);

            attempt.cost += start.getCenter().distance2D(p.getCenter()); // not adjacent...

            if(best == null || attempt.cost < best.cost) {
                attempt.regions.add(start);
                best = attempt;
            }
            cloneList.add(p);
        }

        return best;
    }
}
