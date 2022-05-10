package com.mmkrusniak.drone.static_height;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Angle;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Util;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.drone.DroneUtil;
import com.mmkrusniak.phys.Route;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static com.mmkrusniak.drone.DroneUtil.*;

/**
 * An offline drone which splits complex polygons into simpler ones using a vertical sweeping line, then covers each
 * with a plow traversal. It is among the most effective offline coverage algorithms and is time and space efficient.
 *
 * For detailed information, read Choset et al 1998 "Coverage of Known Spaces: The Boustrophedon Cellular
 * Decomposition."
 */
public class BoustrophedonDrone extends OfflineDrone {

    /**
     * Initializes the drone with the default energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     */
    public BoustrophedonDrone(Area area, int id) {
        super(area, id);
    }

    /**
     * Initializes the drone with a custom energy budget.
     * @param area Area to traverse.
     * @param id Unique ID of the drone.
     * @param energyBudget Energy budget of the drone.
     */
    public BoustrophedonDrone(Area area, int id, double energyBudget) {
        super(area, id, energyBudget);
    }

    /**
     * Splits a complex region into more easily traversible subregions according to the boustrophedon cellular
     * decomposition (Choset et al., 1998). It is a recursive algorithm.
     * @param polygon A polygon to decompose.
     * @param depth The maximum depth to which to recurse. Decomposition will stop before this when all subregions
     *              are convex.
     * @return A list of polygons which are easily traversed (convex or approaching convex) and whose union forms the
     * original polygon.
     */
    public List<com.mmkrusniak.geom.Polygon> decompose(com.mmkrusniak.geom.Polygon polygon, int depth) {
        List<com.mmkrusniak.geom.Polygon> result = new ArrayList<>();

        for(Angle a: polygon.toAngles()) {
            if(depth <= 0) break;
            if(! a.isConcave()) continue;
            if(Util.within(a.a().x(), a.c().x(), a.b().x(), 1)) continue;
            if(Util.approx(a.a().x(), a.b().x(), 0.1)) continue;
            if(Util.approx(a.c().x(), a.b().x(), 0.1)) continue;

            com.mmkrusniak.geom.Point splitPoint = polygon.above(a.b());
            if(splitPoint.equals(a.b())) {
                splitPoint = polygon.below(a.b());
            }

            if(polygon.topmost(a.b().x()).equals(polygon.bottommost(a.b().x()))) continue;

            if(splitPoint == null) {;
                splitPoint = polygon.bottommost(a.b().x());
            }

            com.mmkrusniak.geom.Polygon clone = polygon.addPoint(splitPoint, polygon.indexOf(splitPoint)+1);
            com.mmkrusniak.geom.Polygon[] subregions = clone.split(clone.indexOf(a.b()), clone.indexOf(splitPoint));

            result.addAll(decompose(subregions[0], depth-1));
            result.addAll(decompose(subregions[1], depth-1));
            return result;

        }
        // Nothing to do!
        result.add(polygon);
        return result;
    }

    /**
     * Gives a preplanned route this drone will take (as it is an offline drone).
     * @return The route for the drone to take.
     */
    @Override
    public List<Route> generatePlan() {
        List<com.mmkrusniak.geom.Polygon> regions = decompose(getPolygon(), 400);
        List<com.mmkrusniak.geom.Point> resultPoints = new ArrayList<>();
        List<List<com.mmkrusniak.geom.Point>> missions = new ArrayList<>();

        for(com.mmkrusniak.geom.Polygon p: regions) {
            List<com.mmkrusniak.geom.Point> subPlan = plow(p, getLocation(), DroneUtil.getCruiseAltitude(getArea()));
            missions.add(subPlan);
        }

        for(List<Point> mission: missions) resultPoints.addAll(mission);

        List<Route> result = optimizePlan(resultPoints);
        result.add(0, new RouteHead(Math.PI/2));
        return result;
    }

    /**
     * Paints a visualization of the drone on the supplied graphics instance. Good to modify for debugging; otherwise
     * does nothing to reduce visual clutter.
     * @param g A Java graphics canvas on which to paint.
     */
    @Override
    public void visualize(Graphics g) {
        // Nothing really
    }
}
