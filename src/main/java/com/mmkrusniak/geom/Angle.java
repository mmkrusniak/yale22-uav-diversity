package com.mmkrusniak.geom;

import java.awt.*;

/**
 * A geometry component corresponding to a three-point angle. (That is, an angle between two intersecting lines.) Like
 * all geometry components, it is immutable.
 */
@SuppressWarnings("unused")
public class Angle {

    private final com.mmkrusniak.geom.Point a;
    private final com.mmkrusniak.geom.Point b;
    private final com.mmkrusniak.geom.Point c;

    /**
     * Initialize an angle from three points. The second point is the vertex; the first and last define lines from the
     * vertex.
     * @param a One point of the angle.
     * @param b The vertex of the angle.
     * @param c Another point of the angle.
     */
    public Angle(com.mmkrusniak.geom.Point a, com.mmkrusniak.geom.Point b, com.mmkrusniak.geom.Point c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /** Gives the first point in the angle. @return The first point in the angle.*/
    public com.mmkrusniak.geom.Point a() { return a; }
    /** Gives the second point (the vertex) in the angle. @return The vertex of the angle.*/
    public com.mmkrusniak.geom.Point b() { return b; }
    /** Gives the third point in the angle. @return The third point in the angle.*/
    public com.mmkrusniak.geom.Point c() { return c; }

    /** Gives the first line in the angle. @return The first line in the angle.*/
    public Line ab() { return new Line(a, b); }
    /** Gives the second line in the angle. @return The second point in the angle.*/
    public Line bc() { return new Line(b, c); }

    /**
     * Gives whether the given point lies within the angle. (The lines are treated as rays extending out from the
     * vertex.)
     * @param g A point.
     * @param tolerance Margin of error.
     * @return Whether the given point lies within the angle.
     */
    public boolean containsPoint(com.mmkrusniak.geom.Point g, double tolerance) {
        double angleA = b.bearing(a);
        double angleC = b.bearing(c);
        double angleG = b.bearing(g);

        if(Math.abs(angleC - angleA) > Math.PI) return ! Util.within(angleA, angleC, angleG, tolerance);
        else return Util.within(angleA, angleC, angleG, tolerance);
    }

    /**
     * Gives whether the angle is concave; that is, its measure is greater than pi (in other words, its interior is the
     * greater side).
     * @return Whether the angle is concave.
     */
    public boolean isConcave() {
        return measure() > Math.PI;
    }

    /**
     * Gives whether the angle is straight; that is, its measure is exactly pi.
     * @return Whether the angle is a straight angle.
     */
    public boolean isStraight() {
        return measure() == Math.PI;
    }

    /**
     * Gives whether the angle is straight to a certain (angular) tolerance; that is, whether the angle is within
     * that tolerance of pi.
     * @param t A tolerance.
     * @return Whether the angle is within that tolerance of straight.
     */
    public boolean isStraight(double t) {
        if(a == null || b == null || c == null) return true;
        return Util.approx(Math.PI, measure(), t) || Util.approx(0.0, measure(), t);
    }

    /**
     * Gives the measure of the angle, in radians.
     * @return The measure of the angle.
     */
    public double measure() {
        return (b.bearing(c) - b.bearing(a) + 2 * Math.PI) % (2 * Math.PI);
    }

    /**
     * Renders the angle on a Java graphics canvas.
     * @param g A Java graphics canvas.
     */
    public void render(Graphics g) {
        g.drawOval(b.ix()-5, b.iy()-5, 10, 10);
        String s = String.format("[%.2f]", measure());
        if(isStraight()) s = "|" + s;
        else s = "∠" + s;
        g.drawString(s, b.ix() + 5, b.iy() - 5);
    }

    /**
     * Generates an array of angles from an array of points. Starting with the second point and ending with the
     * second to last, the points become vertices of angles with the two points next to them forming the arms of the
     * angle. There is no wrapping; the last angle is the one with the second to last point as its vertex.
     * @param points A list of points.
     * @return The points transformed into a corresponding list of angles.
     */
    public static Angle[] arrayFromPoints(com.mmkrusniak.geom.Point[] points) {
        if(points.length < 2) return new Angle[0];
        Angle[] result = new Angle[points.length-2];
        for(int i = 0; i < result.length; i++) {
            result[i] = new Angle(points[i], points[i+1], points[i+2]);
        }
        return result;
    }

    /**
     * Rotates an angle about its vertex.
     * @param theta An amount to rotate the angle by.
     * @return The rotated angle.
     */
    public Angle rotate(double theta) {
        com.mmkrusniak.geom.Point a2 = new com.mmkrusniak.geom.Point(a.x() - b.x(), a.y() - b.y());
        com.mmkrusniak.geom.Point c2 = new com.mmkrusniak.geom.Point(c.x() - b.x(), c.y() - b.y());

        a2 = a2.rotate(theta);
        c2 = c2.rotate(theta);

        a2 = new com.mmkrusniak.geom.Point(a2.x() + b.x(), a2.y() + b.y());
        c2 = new Point(c2.x() + b.x(), c2.y() + b.y());

        return new Angle(a2, b, c2);
    }
}
