package com.mmkrusniak.geom;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A two-dimensional n-gon. It's useful in coverage planning algorithms and area decomposition. Like
 * all geometry components, it is immutable; once a Polygon is created, it cannot be changed, nor can the Points that
 * compose it be modified (as they are also immutable).
 *
 * The polygon is internally represented as an array of points, but it is recommended to use {@link Polygon#toAngles
 * toAngles} or {@link Polygon#toLines toLines} as applicable - the indexing remains the same.
 *
 * The coordinate system is y-down, x-right, as reflected in the methods here.
 */
@SuppressWarnings({"unused", "invert"})
public class Polygon implements Cloneable {

    // This could be final, except that f'ing `clone` has to access it.
    // But I swear, Polygon really is immutable.
    private com.mmkrusniak.geom.Point[] points;

    /**
     * Intialize a Polygon from an array of points. The array cannot be null; nor can any point be null.
     * @param points A non-null array of {@link com.mmkrusniak.geom.Point Point}s.
     * @throws NullPointerException if the array or any Point in it is null.
     */
    public Polygon(com.mmkrusniak.geom.Point... points) {
        // Better to throw up here than later even if we don't access any points, then we guarantee they're all nonnull
        for(com.mmkrusniak.geom.Point p: points) if(p == null) throw new NullPointerException();
        this.points = points.clone();
    }

    /**
     * Initialize a Polygon from a list of points.
     * @param points A non-null list of {@link com.mmkrusniak.geom.Point Point}s.
     * @throws NullPointerException if the array or any Point in it is null.
     */
    public Polygon(List<com.mmkrusniak.geom.Point> points) {
        for (com.mmkrusniak.geom.Point p: points) if(p == null) throw new Error(toString());
        this.points = new com.mmkrusniak.geom.Point[points.size()];
        for (int i = 0; i < points.size(); i++) this.points[i] = points.get(i);
    }

    //TODO not all generated polygons are proper.
    /**
     * Initialize a random simple n-gon. Each corner is generated to be {@code rad} units away from a center point,
     * with an additional varying  of up to plus {@code var}. The leftmost point will sit at x=0 and the topmost
     * point will sit at y=0.
     * The polygon may have small concavities. Not all possible polygons can be
     * generated in this way.
     * Due to a bug some generated polygons are improper.
     * @param n Number of points in the generated polygon.
     * @param rad Minimum distance from each point to a center point. It's analogous to the radius of a circle; when
     *            var=0, all points of the polygon lie on a circle of radius {@code rad}.
     * @param var Amount of variance allowed from rad for each vertex. Higher values create spikier polygons.
     */
    public Polygon(int n, double rad, double var) {
        Polygon poly;
        double[] angles = new double[n];
        double sum = 0.0;
        for (int i = 0; i < angles.length; i++) {
            angles[i] = Math.random();
            sum += angles[i];
        }
        for(int i = 0; i < angles.length; i++) angles[i] = angles[i]*Math.PI*2/sum;

        com.mmkrusniak.geom.Point[] points = new com.mmkrusniak.geom.Point[n];

        double angle = 0;
        for(int i = 0; i < angles.length; i++) {
            angle += angles[i];
            points[i] = new com.mmkrusniak.geom.Point(
                    ((2*var*Math.random()-var) + rad) * Math.cos(-angle),
                    ((2*var*Math.random()-var) + rad) * Math.sin(-angle)
            );
        }

        poly = new Polygon(points);

        double dx = poly.leftmost().x();
        double dy = poly.topmost().y();
        for (int i = 0; i < points.length; i++) points[i] = new com.mmkrusniak.geom.Point(points[i].x() - dx, points[i].y() - dy);

        this.points = points;
    }

    /**
     * Shortcut for {@link Polygon Polygon(n, 100, 300)}, which generates reasonable polygons for testing.
     * @param n Number of points in the generated polygon.
     */
    public Polygon(int n) {
        this(n, 100, 300);
    }

    /**
     * Returns the number of sides in the polygon. Note that when an edge contains colinear points, each segment is
     * counted separately.
     * @return The number of sides in the polygon.
     */
    public int numSides() { return points.length; }

    /**
     * Returns an array of the points which define this Polygon.
     * @return The points which define this Polygon.
     */
    public com.mmkrusniak.geom.Point[] toPoints() {
        return points.clone();
    }

    /**
     * Returns an array of Lines which represent the sides of this polygon. They are in order, such that the line
     * indexed at i and the line indexed at i+1 share a point.
     * @return The lines which define the sides of this Polygon.
     */
    public Line[] toLines() {
        Line[] result = new Line[points.length];
        for(int i = 0; i < points.length; i++) {
            result[i] = new Line(points[i], points[(i+1)%points.length]);
        }
        return result;
    }
    /**
     * Returns an array of Angles which represent the vertices of this polygon. They are in order, such that the angle
     * indexed at i and the angle indexed at i+1 share a line segment.
     * The angles are indexed such that the vertex of the first angle is the first vertex of the polygon.
     * @return The angles which define the vertices of this Polygon.
     */
    public Angle[] toAngles() {
        int n = points.length;
        Angle[] result = new Angle[n];
        for(int i = 0; i < points.length; i++) {
            result[i] = new Angle(points[(i+n-1)%n], points[i], points[(i+1)%n]);
        }
        return result;
    }

    /**
     * Gives the convex hull of this Polygon. The convex outline is the result when all concavities of a polygon
     * have been filled. It's a bit like if you stretched a rubber band around the polygon; the shape of the rubber
     * band is the hull.
     * @return The convex hull of this polygon.
     */
    public Polygon getConvexHull() {
        if(cachedConvexHull != null) return cachedConvexHull;
        List<com.mmkrusniak.geom.Point> points = new ArrayList<>(Arrays.asList(toPoints()));

        int i = 0;
        Polygon result = new Polygon(points);
        while(result.isConcave()) {
            if (result.toAngles()[i].isConcave() || result.toAngles()[i].isStraight()) {
                points.remove(i);
                result = new Polygon(points);
                i %= points.size();
            } else i = (i+1) % points.size();
        }
        cachedConvexHull = result;
        return result;
    }
    // It's O(n)ish to calculate, but it's really useful, so let's cache it
    private Polygon cachedConvexHull = null;

    /**
     * Given a vertex in the polygon, returns the two vertices in the polygon which are adjacent to it. If the given
     * point is not a vertex of the polygon, returns null.
     * @param p A point which is a vertex of the polygon.
     * @return An array of two points which are connected to {@code p} in the polygon.
     */
    public com.mmkrusniak.geom.Point[] related(com.mmkrusniak.geom.Point p){
        for(Angle a: toAngles()) if(a.b().equals(p)) return new com.mmkrusniak.geom.Point[]{a.a(), a.c()};
        return null;
    }

    /**
     * Returns a polygon identical to this one, but with an additional point inserted before the point which
     * currently occupies index {@code v}. The original polygon will not be changed.
     * @param point A vertex to add to the polygon.
     * @param v Index before which to add the point (which will become the index of that point).
     * @return A polygon with that vertex added.
     */
    public Polygon addPoint(com.mmkrusniak.geom.Point point, int v) {
        v %= (points.length+1);
        com.mmkrusniak.geom.Point[] result = new com.mmkrusniak.geom.Point[points.length + 1];
        for(int i = 0; i < result.length; i++) {
            if(i <  v) result[i] = points[i];
            if(i == v) result[i] = point;
            if(i >  v) result[i] = points[i-1];
        }
        return new Polygon(result);
    }

    /**
     * Returns a polygon identical to this one, but with the vertex at index {@code v} removed. The original polygon
     * will not be changed.
     * @param v Index of the vertex to be removed.
     * @return A polygon with that vertex removed.
     */
    public Polygon removePoint(int v) {
        com.mmkrusniak.geom.Point[] result = new com.mmkrusniak.geom.Point[points.length - 1];
        for(int i = 0; i < result.length; i++) {
            if(i <  v) result[i] = points[i];
            if(i >  v) result[i] = points[i+1];
        }
        return new Polygon(result);
    }

    /**
     * Returns the left point of the base of the polygon.
     * @return The left point of the base of the polygon.
     */
    public com.mmkrusniak.geom.Point getLeftBasicPoint() {
        Line base = base();
        return  (base.a().y() > base.b().y())? base.a() : base.b();
    }

    /**
     * Returns the right point of the base of the polygon.
     * @return The right point of the base of the polygon.
     */
    public com.mmkrusniak.geom.Point getRightBasicPoint() {

        Line base = base();
        return (base.a().y() > base.b().y())? base.b() : base.a();
    }

    /**
     * Returns two Polygons which are the result of dividing this one over the line defined by the vertices indexed
     * {@code v1} and {@code v2}. The resulting two Polygons will geometrically share an edge (though programmatically,
     * they are totally independent objects).
     * @param v1 The first vertex to split by.
     * @param v2 The second vertex to split by.
     * @return Two Polygons created by splitting this one over {@code v1} and {@code v2}.
     */
    public Polygon[] split(int v1, int v2) {

        v1 %= numSides();
        v2 %= numSides();

        com.mmkrusniak.geom.Point[] poly1 = new com.mmkrusniak.geom.Point[Math.abs(v1-v2)+1];
        com.mmkrusniak.geom.Point[] poly2 = new com.mmkrusniak.geom.Point[points.length - poly1.length + 2];

        if(poly1.length <= 2 || poly2.length <= 2) {
            throw new IllegalArgumentException("Invalid polygon split: " + v1 + " to " + v2 + ".\n" + toString());
        }


        int i1 = 0;
        int i2 = 0;
        for(int i = 0; i < points.length; i++) {
            if(i == v1 || i == v2) {
                poly1[i1] = points[i];
                i1++;
                poly2[i2] = points[i];
                i2++;
            } else if(Util.within(v1, v2, i)) {
                poly1[i1] = points[i];
                i1++;
            } else {
                poly2[i2] = points[i];
                i2++;
            }
        }

        Polygon result1 = new Polygon(poly1);
        Polygon result2 = new Polygon(poly2);

        return new Polygon[]{result1, result2};
    }

    /**
     * Gives all intersections of a line with this polygon. (If the line is colinear with a line in the polygon, only
     * one (or zero) points are given.)
     * @param l A line.
     * @return An array of Point intersections with that line.
     */
    public com.mmkrusniak.geom.Point[] intersections(Line l) {

        List<com.mmkrusniak.geom.Point> result = new ArrayList<>();

        for (Line s : toLines()) {
            com.mmkrusniak.geom.Point p = s.intersection(l);
            if (p != null) result.add(p);
        }

        return result.toArray(new com.mmkrusniak.geom.Point[0]);
    }

    /**
     * Gives the first intersection of a line with the polygon (that is, the intersection closest to {@code l.a()}.
     * Best used with rays.
     * @param l A line.
     * @return The first intersection of that line with the polygon.
     */
    public com.mmkrusniak.geom.Point intersection(Line l) {
        // Important distinction: This one returns the intersection
        // which is closest to the first point on the line.
        // Works well for rays.

        double bestDist = Double.MAX_VALUE;
        com.mmkrusniak.geom.Point best = null;

        com.mmkrusniak.geom.Point[] intersections = intersections(l);
        if(intersections.length == 1) return intersections[0];
        for(com.mmkrusniak.geom.Point p : intersections) {
            double k = p.sqDistance2D(l.a());

            if(k < bestDist && k > 1) {
                bestDist = k;
                best = p;
            }
        }
        return best;
    }

    /**
     * Gives a (uniformly) random point within the polygon.
     * @return A random point within the polygon.
     */
    public Point random() {
        Point p;
        do {
            p = new Point(Math.random()*getCartesianWidth(), Math.random()*getCartesianHeight());
        } while(! encloses(p));
        return p;
    }

    /**
     * Give a polygon which is the result of rotating this one.
     * @param theta The amount to rotate the polygon, in radians.
     * @return A rotated polygon.
     */
    public Polygon rotate(double theta) {
        com.mmkrusniak.geom.Point[] result = new com.mmkrusniak.geom.Point[points.length];
        com.mmkrusniak.geom.Point c = getCenter();

        for(int i = 0; i < points.length; i++) {
            result[i] = new com.mmkrusniak.geom.Point(
                    ((points[i].x()-c.x()) * Math.cos(theta) - (points[i].y()-c.y()) * Math.sin(theta)) + c.x(),
                    ((points[i].x()-c.x()) * Math.sin(theta) + (points[i].y()-c.y()) * Math.cos(theta)) + c.y());
        }
        return new Polygon(result);
    }

    /**
     * Gives the index of a point as a vertex in the polygon, or, if it is not a vertex, the index of the side which
     * the point
     * is closest to.
     * @param p A point.
     * @return The index of p in the polygon, either as a vertex or as the side it is closest to.
     */
    public int indexOf(com.mmkrusniak.geom.Point p) {

        double min = Double.MAX_VALUE;
        int minI = 0;
        Line[] sides = toLines();

        for(int i = 0; i < numSides(); i++) if(points[i].equals(p)) return i;

        for(int i = 0; i < sides.length; i++) {
            double d = p.distance2D(sides[i]);
            if(d < min) {
                min = d;
                minI = i;
            }
        }
        return minI;
    }


    /**
     * Gives whether this Polygon is concave. Many algorithms behave differently with concave polygons than with
     * convex ones.
     * @return Whether this polygon is concave.
     */
    public boolean isConcave() {
        if(cachedConcavity != null) return cachedConcavity; // bless autoboxing
        cachedConcavity = false;
        for(Angle a: toAngles()) if(a.isConcave()) cachedConcavity = true;
        return cachedConcavity;
    }
    // O(n) and useful; let's cache it
    Boolean cachedConcavity = null;


    /**
     * Gives this Polygon as a Java AWT polygon, for graphics purposes.
     * @return This polygon as a Java AWT polygon.
     */
    public java.awt.Polygon toAWTPolygon() {
        int[] xp = new int[points.length];
        int[] yp = new int[points.length];
        for(int i = 0; i < xp.length; i++) {
            xp[i] = points[i].ix();
            yp[i] = points[i].iy();
        }
        return (new java.awt.Polygon(xp, yp, xp.length));

    }


    /**
     * Indicates whether a point lies within this Polygon. A point lying within a concavity is not
     * within the polygon (though it lies within its {@link Polygon#getConvexHull() convex hull}).
     * @param p A point.
     * @return Whether that point lies within this Polygon.
     */
    public boolean encloses(com.mmkrusniak.geom.Point p) {
//        for(Angle a: toAngles()) if(! a.containsPoint(p, 0)) return false;
//        return true;
        // We've had problems with this before from floating point errors.
        // Sometimes you can get better results with the AWT version:
         return toAWTPolygon().contains(p.toAWTPoint());
    }

    /**
     * Gives the bounds of the polygon as an AWT rectangle. Since a polygon can conceivably be oriented in any way
     * (the choice of cartesian bounds is arbitrary), consider using {@link Polygon#getPolygonalWidth()} or
     * {@link Polygon#getPolygonalHeight()}.
     * @return An AWT rectangle which encloses the polygon.
     */
    public java.awt.Rectangle getCartesianBounds() {
        return toAWTPolygon().getBounds();
    }

    /**
     * Render this polygon on an AWT graphics canvas. Very useful for debugging; consider modifying it to show
     * polygon traits which are relevant you.
     * @param g An AWT graphics instance on which to render.
     */
    public void render(Graphics g) {
        for(Line l: toLines()) l.render(g);
    }

    /**
     * Gives the minimum distance from a point to any point on the polygon (either a vertex or a point lying on a
     * side of the polygon).
     * @param p A point.
     * @return The distance from this polygon to the point.
     */
    public double distance(com.mmkrusniak.geom.Point p) {
        double min = Double.MAX_VALUE;
        for(Line l: toLines()) {
            double k = p.distance2D(l);
            if(k < min) min = k;
        }
        return min;
    }


    /**
     * Gives the width of the polygon. Note that this is not the x-width of this polygon as drawn on a screen (the
     * Cartesian width)! Rather, it is the Cartesian width when the polygon is lying on its base. In other words, it
     * is the difference in X spanned by the polygon when it is oriented to be vertically as short as possible.
     * It also happens to be the longer dimension of the smallest square which encompasses the polygon.
     * @return The polygonal width of this polygon.
     */
    public double getPolygonalWidth() {
        Polygon rotated = rotate(base().measure());
        return rotated.rightmost().x() - rotated.leftmost().x();
    }
    /**
     * Gives the height of the polygon. As with {@link Polygon#getPolygonalWidth()}, this is not the y-height of this polygon as
     * drawn on a screen (the Cartesian height)! It is the height of the polygon when oriented to be as vertically
     * short as possible. It also happens to be the shorter dimension of the smallest square which encompasses the
     * polygon.
     * @return The polygonal height of this polygon.
     */
    public double getPolygonalHeight() {
        Polygon rotated = rotate(base().measure());
        return rotated.bottommost().y() - rotated.topmost().y();
    }

    /**
     * Gives the Cartesian height of the polygon - the difference between its greatest and lowest vertex Y-coordinates.
     * @return The Cartesian height of the polygon.
     */
    public double getCartesianHeight() {
        return bottommost().y() - topmost().y();
    }

    /**
     * Gives the Cartesian width of the polygon - the difference between its greatest and lowest vertex X-coordinates.
     * @return The Cartesian width of the polygon.
     */
    public double getCartesianWidth() {
        return rightmost().x() - leftmost().x();
    }


    /**
     * Gives the perimeter of the polygon.
     * @return The perimeter of the polygon.
     */
    public double perimeter() {
        double sum = 0;
        for(Line l: toLines()) sum += l.length();
        return sum;
    }

    /**
     * Gives a deep copy of this polygon - a new polygon with equal vertices.
     * @return A duplicate of this polygon.
     */
    @Override
    public Polygon clone() {
        // Wow, the way this is implemented has got to be my least favorite part of Java.
        // Which is otherwise a language I really like.
        try {
            Polygon clone = (Polygon) super.clone();
            clone.points = points.clone();
            return clone;
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Gives the center of this polygon (as an average of coordinate values).
     * @return The center of this polygon.
     */
    public com.mmkrusniak.geom.Point getCenter() {
        double sumX = 0, sumY = 0;
        for(com.mmkrusniak.geom.Point p: points) {
            sumX += p.x();
            sumY += p.y();
        }
        return new com.mmkrusniak.geom.Point(sumX/numSides(), sumY/numSides());
    }

    /**
     * Returns the point in this polygon which is farthest from a given line.
     * @param l A line.
     * @return The point in this polygon which is farthest from that line.
     */
    public com.mmkrusniak.geom.Point farthest(Line l) {
        double max = 0;
        com.mmkrusniak.geom.Point best = null;
        for(Angle a: toAngles()){
            if(a.isConcave()) continue;
            com.mmkrusniak.geom.Point p = a.b();
            double k = p.distance2D(l);
            if(k > max) {
                max = k;
                best = p;
            }
        }
        return best;
    }
    /**
     * Returns the point in this polygon which is closest to a given line.
     * @param l A line.
     * @return The point in this polygon which is closest to that line.
     */
    public com.mmkrusniak.geom.Point closest(Line l) {
        double min = Double.MAX_VALUE;
        com.mmkrusniak.geom.Point best = null;
        for(Angle a: toAngles()){
            if(a.isConcave()) continue;
            com.mmkrusniak.geom.Point p = a.b();
            double k = p.distance2D(new Line(l, Line.INFINITE));
            if(k < min) {
                min = k;
                best = p;
            }
        }
        return best;
    }

    /**
     * Returns the point in this polygon which is closest to a given point.
     * @param p A point.
     * @return The point in this polygon which is closest to that point.
     */
    public com.mmkrusniak.geom.Point closest(com.mmkrusniak.geom.Point p) {
        double bestDist = 0;
        com.mmkrusniak.geom.Point bestPoint = null;
        for(Line l: toLines()) {
            com.mmkrusniak.geom.Point q = l.closest(p);
            if(bestPoint == null || q.distance2D(p) < bestDist) {
                bestPoint = q;
                bestDist = q.distance2D(p);
            }
        }
        return bestPoint;
    }

    /**
     * Returns the point in this polygon which is farthest a given point.
     * @param p A point.
     * @return The point in this polygon which is farthest from that point.
     */
    public com.mmkrusniak.geom.Point farthest(com.mmkrusniak.geom.Point p) {
        double bestDist = Double.MAX_VALUE;
        com.mmkrusniak.geom.Point bestPoint = null;
        for(Line l: toLines()) {
            com.mmkrusniak.geom.Point q = l.closest(p);
            if(bestPoint == null || q.distance2D(p) > bestDist) {
                bestPoint = q;
                bestDist = q.distance2D(p);
            }
        }
        return bestPoint;
    }

    /**
     * Gives the leftmost point in the polygon (that is, the one with least x-value). It is always a vertex.
     * @return The leftmost point in the polygon.
     */
    public com.mmkrusniak.geom.Point leftmost() {
        com.mmkrusniak.geom.Point best = points[0];
        for(com.mmkrusniak.geom.Point p: points) if(p.x() < best.x()) best = p;
        return best;
    }

    /**
     * Gives a point `t` units into the polygon from the leftmost point.
     * @param t Distance from the leftmost point to the returned point
     * @return A point `t` units into the polygon from the leftmost point.
     */
    public com.mmkrusniak.geom.Point leftish(int t) {
        Point p = leftmost();
        return new Point(p.x() + t, p.y());
    }

    /**
     * Gives the rightmost point in the polygon (that is, the one with greatest x-value). It is always a vertex.
     * @return The rightmost point in the polygon.
     */
    public com.mmkrusniak.geom.Point rightmost() {
        com.mmkrusniak.geom.Point best = points[0];
        for(com.mmkrusniak.geom.Point p: points) if(p.x() > best.x()) best = p;
        return best;
    }


    /**
     * Gives a point `t` units into the polygon from the rightmost point.
     * @param t Distance from the rightmost point to the returned point
     * @return A point `t` units into the polygon from the rightmost point.
     */
    public com.mmkrusniak.geom.Point rightish(int t) {
        Point p = rightmost();
        return new Point(p.x() - t, p.y());
    }

    /**
     * Gives the topmost point in the polygon (that is, the one with least y-value). It is always a vertex.
     * @return The topmost point in the polygon.
     */
    public com.mmkrusniak.geom.Point topmost() {
        com.mmkrusniak.geom.Point best = points[0];
        for(com.mmkrusniak.geom.Point p: points) if(p.y() < best.y()) best = p;
        return best;
    }

    /**
     * Gives the bottommost point in the polygon (that is, the one with greatest y-value). It is always a vertex.
     * @return The bottommost point in the polygon.
     */
    public com.mmkrusniak.geom.Point bottommost() {
        com.mmkrusniak.geom.Point best = points[0];
        for(com.mmkrusniak.geom.Point p: points) if(p.y() > best.y()) best = p;
        return best;
    }

    /**
     * Gives the topmost point in the polygon (that is, the one with least y-value) at a given x-value, or null if
     * none exists.
     * @param x An x-coordinate.
     * @return The topmost point in the polygon at that x-value.
     */
    public com.mmkrusniak.geom.Point topmost(double x) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(x, 1), new com.mmkrusniak.geom.Point(x, 2), Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point top = intersections[0];
        for(com.mmkrusniak.geom.Point p: intersections) if(p.y() < top.y()) top = p;
        return top;
    }

    /**
     * Gives the bottommost point in the polygon (that is, the one with greatest y-value) at a given x-value, or null
     * if none exists.
     * @param x An x-coordinate.
     * @return The topmost point in the polygon at that x-value.
     */
    public com.mmkrusniak.geom.Point bottommost(double x) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(x, 1), new com.mmkrusniak.geom.Point(x, 2), Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point bottom = intersections[0];
        for(com.mmkrusniak.geom.Point p: intersections) if(p.y() > bottom.y()) bottom = p;
        return bottom;
    }

    /**
     * Gives the leftmost point in the polygon (that is, the one with least x-value) at a given y-value, or null if
     * none exists.
     * @param y A y-coordinate.
     * @return The leftmost point in the polygon at that y-value.
     */
    public com.mmkrusniak.geom.Point leftmost(double y) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(1, y), new com.mmkrusniak.geom.Point(2, y), Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point b = intersections[0];
        for(com.mmkrusniak.geom.Point p: intersections) if(p.x() < b.x()) b = p;
        return b;
    }

    /**
     * Gives the rightmost point in the polygon (that is, the one with greatest x-value) at a given y-value, or null if
     * none exists.
     * @param y A y-coordinate.
     * @return The rightmost point in the polygon at that y-value.
     */
    public com.mmkrusniak.geom.Point rightmost(double y) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(1, y), new com.mmkrusniak.geom.Point(2, y), Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point b = intersections[0];
        for(com.mmkrusniak.geom.Point p: intersections) if(p.x() > b.x()) b = p;
        return b;
    }

    /**
     * Gives the point on the border of the polygon directly above the argument; if none exists, returns null.
     * @param p A point.
     * @return The point on the border of the polygon directly above the argument.
     */
    public com.mmkrusniak.geom.Point above(com.mmkrusniak.geom.Point p) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(p.x(), p.y()), new com.mmkrusniak.geom.Point(p.x(), p.y()+1), Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point best = topmost(p.x());
        if(best == null) return null;

        for(com.mmkrusniak.geom.Point k: intersections) if(k.y() < p.y() && k.y() > best.y()) best = k;
        return best;
    }

    /**
     * Gives the point on the border of the polygon directly below the argument; if none exists, returns null.
     * @param p A point.
     * @return The point on the border of the polygon directly below the argument.
     */
    public com.mmkrusniak.geom.Point below(com.mmkrusniak.geom.Point p) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(p.x(), p.y()), new com.mmkrusniak.geom.Point(p.x(), p.y()-1),
                Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point best = bottommost(p.x());
        if(best == null) return null;

        for(com.mmkrusniak.geom.Point k: intersections) if(k.y() > p.y() && k.y() < best.y()) best = k;
        return best;
    }

    /**
     * Gives the point on the border of the polygon directly left of the argument; if none exists, returns null.
     * @param p A point.
     * @return The point on the border of the polygon directly left of the argument.
     */
    public com.mmkrusniak.geom.Point left(com.mmkrusniak.geom.Point p) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(p.x(), p.y()), new com.mmkrusniak.geom.Point(p.x()+1, p.y()),
                Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point best = leftmost(p.x());
        if(best == null) return null;

        for(com.mmkrusniak.geom.Point k: intersections) if(k.x() < p.x() && k.x() > best.x()) best = k;
        return best;
    }

    /**
     * Gives the point on the border of the polygon directly right of the argument; if none exists, returns null.
     * @param p A point.
     * @return The point on the border of the polygon directly right of the argument.
     */
    public com.mmkrusniak.geom.Point right(com.mmkrusniak.geom.Point p) {
        com.mmkrusniak.geom.Point[] intersections = intersections(new Line(new com.mmkrusniak.geom.Point(p.x(), p.y()), new com.mmkrusniak.geom.Point(p.x()+1, p.y()),
                Line.INFINITE));

        if(intersections.length == 0) return null;
        com.mmkrusniak.geom.Point best = rightmost(p.x());
        if(best == null) return null;

        for(com.mmkrusniak.geom.Point k: intersections) if(k.x() > p.x() && k.x() < best.x()) best = k;
        return best;
    }


    /**
     * Calculates the area of the polygon. Only supports convex polygons.
     * @return The area of the polygon.
     */
    public double area() {
        double area = 0;
        int j = numSides()-1;

        for (int i = 0; i < numSides(); i++) {
            area = area +  (points[j].x() + points[i].x()) * (points[j].y() - points[i].y());
            j = i;
        }
        return area/2;
    }

    /**
     * Gives the area of the polygon as calculated by Reimann sum with a given granularity (horizontally). Useful
     * for calculations about traversing the area with a tool of fixed length.
     * Only supports convex polygons.
     * @param granularity Granularity with which to calculate the sum.
     * @return The Reimann-sum area of the polygon with that granularity.
     */
    public double reimannArea(double granularity) {
        double sum = 0;
        double height;
        for(double x = leftmost().x(); x < rightmost().x(); x += granularity) {
            if(x + granularity > rightmost().x()) height = Math.abs(topmost(x).y() - bottommost(x).y());
            else height = Math.max(
                    Math.abs(topmost(x).y() - bottommost(x).y()),
                    Math.abs(topmost(x + granularity).y() - bottommost(x + granularity).y())
            );
            sum += height * granularity;
        }
        return sum;
    }

    /**
     * Indicates whether the given point is a vertex of the polygon.
     * @param point A point.
     * @return Whether the point is a vertex of the polygon.
     */
    // It happens to be always inverted, but this is more semantically clear
    @SuppressWarnings("BooleanMethodIsAlwaysInverted")
    public boolean hasVertex(com.mmkrusniak.geom.Point point) {
        for(com.mmkrusniak.geom.Point p: points) if(p.equals(point)) return true;
        return false;
    }

    /**
     * Gives a list of vertices that this polygon has in common with some other polygon. It does not calculate
     * intersections between the two polygons.
     * @param other Another polygon.
     * @return The vertices in common between this polygon and the other.
     */
    public List<com.mmkrusniak.geom.Point> sharedPoints(Polygon other) {
        List<com.mmkrusniak.geom.Point> common = new ArrayList<>();
        for(com.mmkrusniak.geom.Point p: points) if(other.distance(p) < 2) common.add(p);
        for(com.mmkrusniak.geom.Point p: other.points) if(this.distance(p) < 2 && ! common.contains(p)) common.add(p);
        return common;
    }

    /**
     * Merge two polygons. They must be adjacent (as though through {@link Polygon#split(int, int) split}), sharing
     * exactly one side.
     * @param other Some other polygon.
     * @return A polygon representing these two polygons combined over one side.
     */
    public Polygon combine(Polygon other) {
        List<com.mmkrusniak.geom.Point> result = new ArrayList<>();
        List<com.mmkrusniak.geom.Point> common = sharedPoints(other);

        Polygon us = this.clone();

        if(common.size() > 2) throw new IllegalArgumentException("These polygons are too adjacent...");
        if(common.size() < 2) throw new IllegalArgumentException("These polygons are not adjacent...");


        if(! other.hasVertex(common.get(0))) other = other.addPoint(common.get(0), other.indexOf(common.get(0))+1);
        if(! other.hasVertex(common.get(1))) other = other.addPoint(common.get(1), other.indexOf(common.get(1))+1);
        if(!  this.hasVertex(common.get(0))) us = us.addPoint(common.get(0), indexOf(common.get(0))+1);
        if(!  this.hasVertex(common.get(1))) us = us.addPoint(common.get(1), indexOf(common.get(1))+1);

        int ic1 = indexOf(common.get(0));
        int ic2 = indexOf(common.get(1));
        int begin = Math.max(ic1, ic2);

        // Special case for modular arithmetic
        if(begin == numSides()-1 && (ic1 == 0 || ic2 == 0)) begin = 0;

        int i1 = begin;

        while(i1 != (begin - 1 + numSides()) % numSides()) {
            result.add(us.points[i1]);
            i1 = (i1 + 1) % numSides();
        }

        int i2 = other.indexOf(us.points[(begin - 1 + numSides()) % numSides()]);
        while(i2 != other.indexOf(us.points[begin])) {
            result.add(other.points[i2]);
            i2 = (i2 + 1) % other.numSides();
        }

        Polygon p = new Polygon(result);
        if(p.numSides() != this.numSides() + other.numSides() - 2) System.err.println("[Polygon.combine()] Points " +
                "appeared or disappeared when combining polygons - something suspicious is going on.");
        return p;
    }


    // Base is a pain to calculate, so we cache it:
    private Line cachedBase = null;
    /**
     * Gives the base of the polygon. A polygon's base is the side on which it lays for minimum vertical height; the
     * line perpendicular to that (running through its peak) is its girth. The best way to traverse a convex polygon
     * with a tool is in the direction of its base to minimize turns.
     * @return The base of the polygon.
     */
    public Line base() {
        if(cachedBase != null) return cachedBase;
        double min = Double.POSITIVE_INFINITY;
        Line best = null;

        // Only the convex outline matters to us
        for(Line l: getConvexHull().toLines()) {

            com.mmkrusniak.geom.Point p = farthest(l);
            if(p == null) {

                System.err.println("Could not find a point closest to " + l);
                System.err.println("  There are " + numSides() + " points in this polygon.");
                System.err.println("  It is " + (isProper()? "":"NOT ") + "a proper polygon.");
                System.err.println(this);
                continue;
            }

            if(p.distance2D(l) < min){
                best = l;
                min = p.distance2D(l);
            }
        }
        if(best == null) throw new ArithmeticException("Unexpectedly unable to determine the base of a polygon");
        cachedBase = best;
        return best;
    }
    /**
     * Gives the peak of the polygon. A polygon's peak is its highest point when laying on its base. Its girth is the
     * line running through the peak perpendicular to the base.
     * @return The base of the polygon.
     */
    public com.mmkrusniak.geom.Point peak() {
        return farthest(base());
    }


    /**
     * Gives the girth of the polygon. It is the line running through its peak perpendicular to its base. It's also
     * the shortest possible height of the polygon.
     * @return The girth of the polygon.
     */
    public Line girth() {
        Line b = base();
        return Line.perpendicularTo(b, farthest(b));
    }

    /**
     * Renders the polygon as a string.
     * @return The polygon rendered as a string.
     */
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder();
        for(com.mmkrusniak.geom.Point p: points) result.append("\t").append(p).append("\n");
        return result.toString();
    }

    /**
     * Gives whether the polygon is proper. A proper polygon has no sides which intersect and is oriented such that
     * its inside faces in. Most functions here behave erratically with improper polygons.
     * @return Whether this polygon is proper.
     */
    public boolean isProper() {
        for(Line n: toLines()) {
            for(Line m: toLines()) {
                if(n.intersection(m) != null
                        && ! n.a().equals(m.a())
                        && ! n.a().equals(m.b())
                        && ! n.b().equals(m.a())
                        && ! n.b().equals(m.b())) {
                    System.err.println("This polygon is not proper.");
                    System.err.println(m);
                    System.err.println(n);
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Gives whether this polygon is "simply traversable." A simply traversable polygon is never concave in the
     * direction of its base. A simply traversable polygon can be as efficiently covered as a convex polygon, even if
     * it has some concavities.
     * @return Whether this polygon is simply traversable.
     */
    public boolean isSimplyTraversable() {
        Line girthLine = girth();
        double theta = -girthLine.a().bearing(girthLine.b());
        Polygon poly = rotate(theta);

        for(Angle a: poly.toAngles()) {
            if(a.isConcave() && (! Util.within(a.a().x(), a.c().x(), a.b().x()))) return false;
        }
        return true;
    }

    public List<com.mmkrusniak.geom.Point> verticesBetween(int v1, int v2) {
        List<com.mmkrusniak.geom.Point> result = new ArrayList<>();

        for(int i = v1; i != v2; i = (i+1) % numSides()) {
            result.add(points[i]);
        }
        result.add(points[v2]);
        return result;
    }

    public List<com.mmkrusniak.geom.Point> verticesBetween(com.mmkrusniak.geom.Point v1, Point v2) {
        return verticesBetween(indexOf(v1), indexOf(v2));
    }
}