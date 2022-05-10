package com.mmkrusniak.drone;

import com.mmkrusniak.frame.Area;
import com.mmkrusniak.geom.Line;
import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Polygon;
import com.mmkrusniak.phys.RouteHead;
import com.mmkrusniak.phys.Route;
import com.mmkrusniak.phys.RouteThrough;
import com.mmkrusniak.phys.RouteTo;

import javax.swing.text.BadLocationException;
import javax.swing.text.html.HTMLDocument;
import javax.swing.text.html.HTMLEditorKit;
import java.io.IOException;
import java.io.Reader;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * This class provides a variety of useful functionality for creating new drone path algorithms.
 */
public class DroneUtil {

    /**
     * Yields the apparent width (in meters) of an image taken at the given altitude.
     * @param alt An altitude, in meters.
     * @return The apparent width (in meters) of an image taken at the given altitude.
     */
    public static double getScanWidth(double alt) {
        return 2 * alt * Math.tan(Drone.FOV_WIDTH/2);
    }

    /**
     * Yields the apparent height (in meters) of an image taken at the given altitude. (Note that height refers to
     * distance on the y-axis, while altitude refers to distance on the z axis.)
     * @param alt An altitude, in meters.
     * @return The apparent height (in meters) of an image taken at the given altitude.
     */
    public static double getScanHeight(double alt) {
        return 2 * alt * Math.tan(Drone.FOV_HEIGHT/2);
    }

    /**
     * Yields the required altitude (in meters) for an image taken at that altitude to have the given apparent height.
     * @param height A (y-axis) apparent image height, in meters.
     * @return The altitude required to yield an image of that apparent height, in meters.
     */
    public static double getScanAlt(double height) {
        return height / (2.0 * Math.tan(Drone.FOV_HEIGHT/2));
    }

    /**
     * Divides a list of points which is taken to describe a drone path into a denser list, where points are placed
     * on the path in regular intervals such that images taken at those points at the given altitude line up
     * perfectly (when their smaller dimension is parallel to the path).
     * @param points A list of points to be considered as a drone path.
     * @param alt A drone altitude.
     * @return A denser list of points such that images taken at each point perfectly line up.
     */
    public static List<Point> subdivide(List<Point> points, double alt) {
        double imageHeight = getScanHeight(alt);

        List<Point> result = new ArrayList<>();
        for(Line l: Line.arrayFromPoints(points.toArray(new Point[0])))
            result.addAll(l.toSubpoints(imageHeight));
        result.add(points.get(points.size()-1));
        return result;
    }

    /**
     * Gives a simple back-and-forth plow path through the given polygon from the start point at a certain altitude.
     * @param poly An area through which to plow.
     * @param start A starting location.
     * @param alt An altitude at which to cover the area.
     * @return A simple plow coverage path through the polygon, as a list of points.
     */
    public static List<Point> plow(Polygon poly, Point start, double alt) {
        if(alt < 0) alt = 0;
        double rowWidth = getScanWidth(alt-2);
        double padding = 0;
        double separation = getScanHeight(alt-2);

        Point current = poly.closest(start);
        List<Point> result = new ArrayList<>();
        result.add(new Point(current.x(), current.y(), alt));

        // Step 1: Are we headed left to right, or right to left?
        // Whichever's closest to the current point.
        if(current.sqDistance2D(poly.leftmost()) > current.sqDistance2D(poly.rightmost())) {
            current = poly.rightmost();
            rowWidth = -rowWidth;
        } else current = poly.leftmost();
        result.add(new Point(current.x(), current.y()+padding, alt));

        // Step 2: Plow!
        Point next;
        boolean firstTrackFlag = true;
        double x = current.x();
        while(poly.topmost(x) != null
                || poly.bottommost(x) != null) {

            // At the end we can have up to rowWidth/2 left, thus this edge case
            if(poly.topmost(x) == null || poly.bottommost(x) == null || firstTrackFlag) {
                x -= rowWidth/2;
                firstTrackFlag = false;
            }

            // Northbound step
            if(poly.topmost(x) != null) {
                Point top = new Point(x, poly.topmost(x).y() + padding, alt);
                result.addAll(new Line(result.remove(result.size()-1), top).toSubpoints(separation));
                result.add(top);
            } if(poly.topmost(x + rowWidth) != null) {
                next = new Point(
                        x + rowWidth,
                        poly.topmost(x + rowWidth).y() + padding,
                        alt);
                result.add(next);
                x += rowWidth;
            }

            // Southbound step
            if(poly.bottommost(x) != null) {
                Point bottom = new Point(x, poly.bottommost(x).y() + padding, alt);
                result.addAll(new Line(result.remove(result.size()-1), bottom).toSubpoints(separation));
                result.add(bottom);
            } if(poly.bottommost(x + rowWidth) != null) {
                next = new Point(
                        x + rowWidth,
                        poly.bottommost(x + rowWidth).y() - padding,
                        alt);
                result.add(next);
            }
            x += rowWidth;
        }
        return result;
    }

    /**
     * Gives a simple back-and-forth plow path through the given polygon from the start point at a certain altitude
     * which will occur at an angle to the polygon.
     * @param poly An area through which to plow.
     * @param current A starting location.
     * @param theta An angle at which to do the plow motion
     * @param alt An altitude at which to cover the area.
     * @return A simple plow coverage path through the polygon, as a list of points.
     */
    public static List<Point> plow(Polygon poly, Point current, double alt, double theta) {
        Polygon rotated = poly.rotate(theta);
        List<Point> result = plow(rotated, current.rotate(theta, poly.getCenter()), alt);
        return rotatePath(result, -theta, poly.getCenter());
    }

    /**
     * Optimizes a drone plan which is given as a list of points. This generates routes through points which are
     * lined up (so the drone doesn't stop at each one individually) and routes directly to points which are not (to
     * prevent overshooting). The first and last two points are always direct routes.
     * @param plan A drone plan, given as a list of points.
     * @return An optimized drone plan, given as a list of Routes.
     */
    public static List<Route> optimizePlan(List<Point> plan) {
        List<Route> result = new ArrayList<>();
        result.add(new RouteTo(plan.get(0)));
        result.add(new RouteTo(plan.get(1)));
        for (int i = 2; i < plan.size()-2; i++) {
            if(plan.get(i).distance2D(new Line(plan.get(i-1), plan.get(i+2))) < 5) {
                result.add(new RouteThrough(plan.get(i)));
            } else result.add(new RouteTo(plan.get(i)));
        }
        result.add(new RouteTo(plan.get(plan.size()-2)));
        result.add(new RouteTo(plan.get(plan.size()-1)));
        return result;
    }

    /**
     * Optimizes a drone plan which is given as a list of routes. This sets the appropriate routes to be RoutesTo and
     * RoutesThrough, and leaves heading changes as-is. Analogous to optimizePlan. This is useful if you are creating
     * a drone based on anothers' already-optimized routes with small changes, so you can easily re-optimize it after
     * the changes.
     * @param plan A drone plan given as a list of Routes.
     * @return An optimized drone plan as a list of Routes.
     */
    public static List<Route> optimizeRoutes(List<Route> plan) {
        List<Route> result = new ArrayList<>();
        result.add(headOrTo(plan.get(0)));
        result.add(headOrTo(plan.get(1)));
        for (int i = 2; i < plan.size()-2; i++) {
            if(plan.get(i).getTarget() == null) result.add(plan.get(i));
            else if(isTightTurn(plan.get(i-1), plan.get(i), plan.get(i+2))) {
                result.add(new RouteThrough(plan.get(i).getTarget()));
            } else result.add(new RouteTo(plan.get(i).getTarget()));
        }
        result.add(headOrTo(plan.get(plan.size()-2)));
        result.add(headOrTo(plan.get(plan.size()-1)));
        return result;
    }

    private static boolean isTightTurn(Route a, Route b, Route c) {
        if(a.getTarget() == null || b.getTarget() == null || c.getTarget() == null) return true;
        return b.getTarget().distance2D(new Line(a.getTarget(), c.getTarget())) < 5;
    }

    private static Route headOrTo(Route source) {
        if(source instanceof RouteHead || source instanceof RouteTo) return source;
        return new RouteTo(source.getTarget());
    }

    /**
     * Rotates an entire path (as a list of points) about a point.
     * @param points A path to rotate, as a list of points.
     * @param theta An amount by which to rotate the path.
     * @param c A point about which to rotate the path.
     * @return The input path, rotated by theta about the given point.
     */
    public static List<Point> rotatePath(List<Point> points, double theta, Point c) {
        List<Point> result = new ArrayList<>();
        for (Point point : points) result.add(point.rotate(theta, c));
        return result;
    }

    /**
     * Performs a simple heuristic TSP ordering on the given points.
     *
     * I'm not sure this is working as expected, as nothing seems to use the return value. Maybe this should be
     * double-checked for functionality?
     * @param points A set of points.
     * @param start A starting point.
     * @param end An ending point.
     * @return A slightly more ordered set of points.
     */
    public static List<Point> heuristicTSP(List<? extends Point> points, Point start, Point end) {


        final Point finalEnd = end;
        points.sort((Comparator<Point>) (point, t1) -> (int) (point.distance2D(finalEnd) - t1.distance2D(finalEnd)));

        List<Point> result = new ArrayList<>();
        result.add(start);
        result.add(end);


        for(Point p: points) {
            int bestIndex = 1;
            double bestLength = Double.MAX_VALUE;
            for(int j = 1; j < result.size()-1; j++) {
                result.add(j, p);
                double d = pathLength(result);
                if(d < bestLength) {
                    bestLength = d;
                    bestIndex = j;
                }
                result.remove(p);
            }
            result.add(bestIndex, p);
        }
        result.remove(start);
        result.remove(end);
        return result;
    }

    /**
     * Gives the total length of a path, entered as a list of points.
     * @param points A path.
     * @return The total length of the path.
     */
    public static double pathLength(List<Point> points) {
        double sum = 0;
        for(Line l: Line.arrayFromPoints(points.toArray(new Point[0]))) {
            sum += l.length();
        }
        return sum;
    }

    /**
     * Parses a string into a HTML document. Useful for displaying drones on the frontend.
     * @param string A string of valid HTML.
     * @return A displayable Java HTML document.
     */
    public static HTMLDocument stringToHTML(String string) {
        Reader stringReader = new StringReader(string);
        HTMLEditorKit htmlKit = new HTMLEditorKit();
        HTMLDocument htmlDoc = (HTMLDocument) htmlKit.createDefaultDocument();
        try {
            htmlKit.read(stringReader, htmlDoc, 0);
        } catch (IOException | BadLocationException e) {
            e.printStackTrace();
        }
        return htmlDoc;
    }
    public static double getCruiseAltitude(Area area) {
        return getCruiseAltitude(area.getHull());
    }
    public static double getCruiseAltitude(Area area, double maxDist) {
        return getCruiseAltitude(area.getHull(), maxDist);
    }


    public static double getCruiseAltitude(Polygon polygon) {
        return getCruiseAltitude(polygon, Drone.MAX_TRAVEL_DISTANCE);
    }

    public static double getCruiseAltitude(Polygon polygon, double maxDist) {
        double w = polygon.getPolygonalWidth();
        double h = polygon.getPolygonalHeight();
        double areaRatio = polygon.area() / (w*h);
        double a = (h*w*areaRatio) / ((maxDist-h)*2*Math.tan(Drone.FOV_WIDTH/2));
        return Math.max(10, a * 1.5);
    }
}