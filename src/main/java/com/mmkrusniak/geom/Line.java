package com.mmkrusniak.geom;

import java.awt.*;
import java.util.Arrays;
import java.util.Collection;

/**
 * A geometric object representing a line segment, ray, or infinite line. All lines are defined by two points, though
 * defining a line as a ray or an infinite line affects calculations involving it, and except for a few exceptions
 * noted below those details are mostly abstracted. Like all geometric objects, lines are immutable; once defined
 * they may not be changed.
 */
@SuppressWarnings("unused")
public class Line {

    private final Point a;
    private final Point b;

    private final int type;

    /** Line type corresponding to a line segment. */
    public static final int SEGMENT = 0;
    /** Line type corresponding to a ray. */
    public static final int RAY = 1;
    /** Line type corresponding to an infinite line. */
    public static final int INFINITE = 2;

    /**
     * Construct a line segment defined by the two points at which it ends.
     * @param a The first endpoint of the segment.
     * @param b The second endpoint of the segment.
     */
    public Line(Point a, Point b) {
        this.a = a;
        this.b = b;
        this.type = SEGMENT;
    }

    /**
     * Construct a line that passes through two points. It can be a segment (ends at both points), a ray (ends at the
     * first point but extends infinitely out of the second point), or an infinite line (extends infinitely out of
     * both points).
     *
     * This does mean that there are multiple ways of instantiating a ray or an infinite line - for instance, an
     * infinite line through (0, 0) and (1, 1) is the same as an infinite line through (2, 2) and (3, 3).
     * @param a The first point by which the line is defined (an endpoint if it's to be a line segment or ray).
     * @param b The second point by which the line is defined (an endpoint if t's to be a line segment).
     * @param type The type of line to construct; one of {@code SEGMENT}, {@code INFINITE}, or {@code RAY}.
     */
    public Line(Point a, Point b, int type) {
        this.a = a;
        this.b = b;
        this.type = type;
    }

    /**
     * Construct a line identical to another line, but which is a different type. For example, extends a segment to a
     * ray.
     * @param l A line.
     * @param type A line type; one of {@code SEGMENT}, {@code INFINITE}, or {@code RAY}.
     */
    public Line(Line l, int type) {
        this(l.a, l.b, type);
    }

    /**
     * Construct a line (either a ray or an infinite line) from a point and a slope. If it's a ray, the point is its
     * endpoint. Line segments can't be constructed in this way.
     * @param a A point.
     * @param m A slope for the line to have. (Remember that the coordinate system has Y facing down, so a positive
     *          slope generates a downwards-tilting line graphically speaking.)
     * @param type A line type, either {@code INFINITE} or {@code RAY}.
     * @throws IllegalArgumentException If used to instantiate a line segment.
     */
    public Line(Point a, double m, int type) {
        this(a, new Point(a.x() + 1, a.y() + m), type);
        if(type == Line.SEGMENT) throw new IllegalArgumentException("Cannot instantiate a line segment with only a " +
                "point and a slope");
    }

    /**
     * Gives an infinite line which is parallel to the given line and passes through the given point.
     * @param l A line.
     * @param p A point.
     * @return A line parallel to the given line passing through the given point.
     */
    public static Line parallelTo(Line l, Point p) {
        return new Line(p, new Point(p.x()+l.dx(), p.y()+l.dy()), INFINITE);
    }

    /**
     * Gives an infinite line which is perpendicular to the given line and passes through the given point.
     * @param l A line.
     * @param p A point.
     * @return A line perpendicular to the given line passing through the given point.
     */
    public static Line perpendicularTo(Line l, Point p) {
        double m = l.slope(); // this could be 0!
        Line k;
        if(m != 0) k = new Line(p, new Point(p.x() + 1, p.y() - 1/l.slope()), Line.INFINITE);
        else k = new Line(p, new Point(p.x(), p.y()+1), Line.INFINITE);
        Point r = new Line(l, Line.INFINITE).intersection(k);
        return new Line(p, r);
    }

    /**
     * Gives the type of this Line - infinite line, ray or segment. @return The type of this Line.
     */
    public int getType() { return type; }

    /** Gives the Cartesian X distance spanned by the points defining this line (so rays and infinite lines also have
     *  finite distance). @return The change in X on this line. */
    public double dx()      { return (b.x()  - a.x()); }
    /** Gives the Cartesian Y distance spanned by the points defining this line (so rays and infinite lines also have
     *  finite distance). @return The change in Y on this line. */
    public double dy()      { return (b.y()  - a.y()); }
    /** Gives the Cartesian Z distance spanned by the points defining this line (so rays and infinite lines also have
     *  finite distance). @return The change in Z on this line. */
    public double dz()      { return (b.z()  - a.z()); }
    /** Gives the nth Cartesian distance spanned by the points defining this line (so rays and infinite lines also have
     *  finite distance). @return The change in the nth dimension on this line. */
    public double dn(int n) { return (b.n(n) - a.n(n)); }

    /** Gives the first point defining this line. @return The first point defining this line. */
    public Point a() { return a; }
    /** Gives the second point defining this line. @return The second point defining this line. */
    public Point b() { return b; }

    /**
     * Gives the intersection of this line with another. The implementation uses Cramer's Rule.
     * @param l Another line.
     * @return The intersection of that line with this one, or null if they do not intersect.
     */
    public Point intersection(Line l) {

        // Cramer's Rule - not the best, but it works
        double det1 = Util.det(new double[][] {{l.dx(), l.dy()}, {-dx(), -dy()}});
        double det2 = Util.det(new double[][] {{l.dx(), l.dy()}, {a.x() - l.a().x(), a.y() - l.a().y()}});

        if(det1 == 0) return null;

        double t = det2/det1;
        Point p = new Point(a.x() + t*dx(), a.y() + t*dy());

        // Segment checking. A ray or "infinite" line can be created with arbitrarily far Points.
        if(type == SEGMENT) {
            if (!Util.within(a().x(), b().x(), p.x(), .5)) return null;
            if (!Util.within(a().y(), b().y(), p.y(), .5)) return null;
        } else if(type == RAY) {
            if(dx() > 0 && p.x() < a().x()) return null;
            if(dx() < 0 && p.x() > a().x()) return null;
            if(dy() > 0 && p.y() < a().y()) return null;
            if(dy() < 0 && p.y() > a().y()) return null;
        }

        if(l.type == SEGMENT) {
            if (!Util.within(l.a().x(), l.b().x(), p.x())) return null;
            if (!Util.within(l.a().y(), l.b().y(), p.y())) return null;
        } else if(l.type == RAY) {
            if(l.dx() > 0 && p.x() < l.a().x()) return null;
            if(l.dx() < 0 && p.x() > l.a().x()) return null;
            if(l.dy() > 0 && p.y() < l.a().y()) return null;
            if(l.dy() < 0 && p.y() > l.a().y()) return null;
        }

        return p;
    }

    /**
     * Formats this line as a string. The format is {@code (%.2f, %.2f, %.2f)-(%.2f, %.2f, %.2f)}. Note that this does
     * not indicate the type of line.
     * @return The line as a string.
     */
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f)-(%.2f, %.2f, %.2f)", a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
    }

    /**
     * Renders this line on a Java graphics canvas. Infinite lines and rays are drawn to extreme coordinates in lieu
     * of infinite ones.
     * @param g A Java graphics canvas.
     */
    public void render(Graphics g) {
        if(type == SEGMENT) g.drawLine(a.ix(), a.iy(), b.ix(), b.iy());
        if(type == RAY) g.drawLine(a.ix(), a.iy(), b.ix()+10000, (int) (b.iy()+10000*slope()));
        if(type == INFINITE) g.drawLine(a.ix()-10000, (int) (a.iy()-10000*slope()), b.ix()+10000, (int) (b.iy()+10000*slope()));
    }

    /**
     * Splits this line into subpoints, where each one is a distance apart on the line. There may be a remainder, so
     * one point might be somewhat closer than the others.
     * @param segLength The length apart to create the subpoints.
     * @return A list of points which all lie on the line and all of which (except one) are that length apart.
     */
    public Collection<? extends Point> toSubpoints(double segLength) {
        if(! (type == SEGMENT)) throw new ArithmeticException("you can't split an infinite ray or line into subpoints");

        int n = (int) (length()/segLength) + 1; // one extra for the remainder, but not another for the last point
        double dxi = dx()/n;
        double dyi = dy()/n;
        Point[] result = new Point[n];

        for(int i = 0; i < n; i++) {
            result[i] = new Point(a.x() + i*dxi, a.y() + i*dyi, a.z());
        }
        return Arrays.asList(result);
    }

    /**
     * Gives the midpoint of the line.
     * @return The midpoint of the line.
     */
    public Point midpoint() {
        return new Point((a.x()+b.x())/2.0, (a.y() + b.y())/2.0);
    }

    /**
     * Rotates the line around its midpoint.
     * @param theta Amount to rotate.
     * @return A line which represents this line after being rotated.
     */
    public Line rotate(double theta) {
        Point m = midpoint();
        Point a2 = new Point(a.x() - m.x(), a.y() - m.y());
        Point b2 = new Point(b.x() - m.x(), b.y() - m.y());

        a2 = a2.rotate(theta);
        b2 = b2.rotate(theta);

        a2 = new Point(a2.x() + m.x(), a2.y() + m.y());
        b2 = new Point(b2.x() + m.x(), b2.y() + m.y());

        return new Line(a2, b2);
    }

    /**
     * Gives whether a point lies on (or near) the line. The granularity used is {@link Point#GRANULARITY}.
     * @param p A point.
     * @return Whether that point lies on the line.
     */
    public boolean contains(Point p) {
        return p.distance2D(this) < 1;
    }

    /**
     * Gives whether another line is parallel to this one.
     * @param l A line.
     * @return Whether that line is parallel to this one.
     */
    public boolean isParallel(Line l) {
        return Util.approx(l.slope(), slope(), 0.2);
    }

    /**
     * Gives whether another line is parallel to this one, with a tolerance (expected as a slope).
     * @param l A line.
     * @param t A tolerance.
     * @return Whether that line is parallel to this one.
     */
    public boolean isParallel(Line l, double t) {
        return Util.approx(l.slope(), slope(), t);
    }

    /**
     * Gives whether another line is perpendicular to this one.
     * @param l A line.
     * @return Whether that line is perpendicular to this one.
     */
    public boolean isPerpendicular(Line l) { return Util.approx(l.slope(), -1.0/slope(), 0.001);}

    /**
     * Gives the length of this line.
     * @return The length of this line.
     */
    public double length() {
        return a.distance2D(b);
    }

    /**
     * Gives the length of this line, considering only the X and Y coordinates.
     * @return The two-dimensional length of this line.
     */
    public double length2D() {
        return Math.sqrt(dx()*dx() + dy()*dy());
    }

    /**
     * Gives the slope of this line.
     * @return The slope of this line.
     */
    public double slope() { return dy() / dx(); }

    /**
     * Gives the angle measure ("grade") of this line (from the first point to the second point.)
     * @return The angle measure of this line.
     */
    public double measure() { return a.bearing(b); }

    /**
     * Gives the point which lies closest to another point on this line.
     * @param p A point.
     * @return The point on this line which is closest to the specified one.
     */
    public Point closest(Point p) {
        Point r = intersection(new Line(p, -1/slope(), Line.INFINITE));
        if(r == null) {
            if(p.distance2D(a) < p.distance2D(b)) return a;
            return b;
        } else return r;
    }

    /**
     * Generates an array of Lines that form a path through a list of points in order. It does not loop, merely
     * stopping at the final point.
     * @param points A list of points.
     * @return An array of Lines that forms a path through the given list of points.
     */
    public static Line[] arrayFromPoints(Point[] points) {
        if(points.length <= 1) return new Line[]{};
        Line[] result = new Line[points.length-1];
        for(int i = 0; i < points.length-1; i++) {
            result[i] = new Line(points[i], points[i+1]);
        }
        return result;
    }
}
