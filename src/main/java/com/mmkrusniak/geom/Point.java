package com.mmkrusniak.geom;

import com.mmkrusniak.frame.Displayable;
import com.mmkrusniak.frame.Option;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * An n-dimensional Cartesian point. (For most uses, however, only two or three dimensions are supported). Like
 * all geometry components, Point is immutable; once created a Point cannot be modified.
 *
 * The coordinate system is y-down, x-right.
 */
@SuppressWarnings("unused")
public class Point implements Displayable {

    /**
     * Precision with which to make internal calculations. For instance, with GRANULARITY=1, points which are one
     * unit apart are considered to be equal.
     */
    static final double GRANULARITY = 0.01;

    // Style arguments
    /** Render style argument which draws points as a small X. */
    public static final int CROSS = 1;
    /** Render style argument which draws points as a dot. */
    public static final int DOT = 0;

    // Internal representation of where the point is.
    private final double[] coordinates;

    /** Constant representation of the origin. */
    public static final Point ORIGIN = new Point(0, 0);

    /**
     * Initializes a Point at the specified coordinates.
     * @param x First coordinate.
     * @param y Second coordinate.
     */
    public Point(double x, double y) {
        coordinates = new double[2];
        coordinates[0] = x;
        coordinates[1] = y;
    }

    /**
     * Initializes a Point at the specified coordinates.
     * @param coordinates Any number of coordinates.
     */
    public Point(double... coordinates) {
        this.coordinates = coordinates.clone();
    }

    /**
     * Renders the point as a string. The format is an ordered set, with decimals truncated to two places.
     * @return The point rendered as a string.
     */
    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("(");

        for(int i = 0; i < coordinates.length; i++) {
            builder.append(String.format("%.2f", coordinates[i]));
            if(i != coordinates.length-1) builder.append(", ");
        }
        builder.append(")");
        return builder.toString();
    }

    /** Yields the first coordinate value. @return The first coordinate value.*/
    public double x()      { return (coordinates.length > 0)? coordinates[0] : 0; }
    /** Yields the second coordinate value. @return The second coordinate value.*/
    public double y()      { return (coordinates.length > 1)? coordinates[1] : 0; }
    /** Yields the third coordinate value. @return The third coordinate value.*/
    public double z()      { return (coordinates.length > 2)? coordinates[2] : 0; }
    /** Yields the nth coordinate value. @param n Which coordinate value to return. @return The nth coordinate value.*/
    public double n(int n) { return (coordinates.length > n)? coordinates[n] : 0; }

    /** Yields the first coordinate value as an integer. @return The first coordinate value as an integer.*/
    public int ix()      { return (int) Util.constrain(((coordinates.length > 0)? coordinates[0] : 0), Integer.MIN_VALUE, Integer.MAX_VALUE); }
    /** Yields the second coordinate value as an integer. @return The second coordinate value as an integer.*/
    public int iy()      { return (int) Util.constrain(((coordinates.length > 1)? coordinates[1] : 0), Integer.MIN_VALUE, Integer.MAX_VALUE); }
    /** Yields the third coordinate value as an integer. @return The third coordinate value as an integer.*/
    public int iz()      { return (int) Util.constrain(((coordinates.length > 2)? coordinates[2] : 0), Integer.MIN_VALUE, Integer.MAX_VALUE); }
    /** Yields the nth coordinate value as an integer. @param Which coordinate value to return. @return The nth
     * coordinate value as an integer.*/
    public int in(int n) { return (int) Util.constrain(((coordinates.length > n)? coordinates[n] : 0), Integer.MIN_VALUE, Integer.MAX_VALUE); }

    /** Gives the dimension of the point - how many coordinate values are defined. @return The dimension of the point.*/
    public int dim() { return coordinates.length; }

    /** Yields the 2D distance from one point to another. @return The first coordinate value as an integer.*/
    public double distance2D(Point other) { return Math.sqrt(sqDistance2D(other)); }

    /** Yields the 3D distance from one point to another. @return The first coordinate value as an integer.*/
    public double distance3D(Point other) { return Math.sqrt(sqDistance3D(other)); }

    /**
     * The distance of this point from a line. If the line is infinite in one or two directions, that is respected
     * and the resulting point may not necessarily lie between the two points defining the line.
     * @param l A line.
     * @return The distance of this point from the line.
     */
    public double distance2D(Line l) {
        Point a = l.a();
        Point b = l.b();
        double len = a.sqDistance2D(b);
        if (len == 0) return sqDistance2D(a);
        double t = ((x() - a.x()) * (b.x() - a.x()) + (y() - a.y()) * (b.y() - a.y())) / len;
        if(l.getType() == Line.SEGMENT) t = Util.constrain(t, 0, 1);
        if(l.getType() == Line.RAY) t = (t < 0)? 0 : t;
        return Math.sqrt(sqDistance2D(new Point((int) (a.x() + t * (b.x() - a.x())), (int) (a.y() + t * (b.y() - a.y())))));
    }

    public double distance3D(Line l) {
        Point a = l.a();
        Point b = l.b();
        double len = a.sqDistance3D(b);
        if (len == 0) return sqDistance3D(a);
        double t = ((x() - a.x()) * (b.x() - a.x()) +
                        (y() - a.y()) * (b.y() - a.y()) +
                        (z() - a.z()) * (b.z() - a.z())) / len;
        if(l.getType() == Line.SEGMENT) t = Util.constrain(t, 0, 1);
        if(l.getType() == Line.RAY) t = (t < 0)? 0 : t;
        return Math.sqrt(sqDistance3D(new Point(
                (a.x() + t * (b.x() - a.x())),
                (a.y() + t * (b.y() - a.y())),
                (a.z() + t * (b.z() - a.z())))));
    }

    /**
     * Rotates this point about the origin.
     * @param theta The amount of rotation, in radians. (Note that since y points down in this coordinate system,
     *          increasing the amount of rotation makes for a clockwise, not counterclockwise, spin.
     * @return A point which is the result of rotating this one {@code theta} degress about the origin.
     */
    public Point rotate(double theta) {
        return new Point(
                x() * Math.cos(theta) - y() * Math.sin(theta),
                x() * Math.sin(theta) + y() * Math.cos(theta),
                this.z());
    }

    /**
     * Rotates this point about another point.
     * @param c A point about which to rotate.
     * @param theta The amount of rotation, in radians. (Note that since y points down in this coordinate system,
     *          increasing the amount of rotation makes for a clockwise, not counterclockwise, spin.
     * @return A point which is the result of rotating this one {@code theta} degress about {@code c}.
     */
    public Point rotate(double theta, Point c) {
        return new Point(
                ((this.x() - c.x()) * Math.cos(theta) - (this.y() - c.y()) * Math.sin(theta)) + c.x(),
                ((this.x() - c.x()) * Math.sin(theta) + (this.y() - c.y()) * Math.cos(theta)) + c.y(),
                this.z());
    }

    /**
     * Gives the distance from this point to another, squared. It's preferable to compare squared distances rather than
     * normal ones in many cases, since taking the square root of a distance is a long and unnecessary operation in
     * that case.
     * @param other A point.
     * @return The distance from this point to another, squared.
     */
    public double sqDistance(Point other, int dim) {
        int sum = 0;
        for(int i = 0; i < dim; i++) sum += (other.n(i) - n(i))*(other.n(i) - n(i));
        return sum;
    }

    public double sqDistance2D(Point other) {
        return (other.x() - x())*(other.x() - x()) + (other.y() - y())*(other.y() - y());
    }

    public double sqDistance3D(Point other) {
        return sqDistance(other, 3);
    }

    /**
     * Gives the bearing of this point relative to the origin.
     * @return The bearing of this point relative to the origin.
     */
    public double bearing() {return ORIGIN.bearing(this); }

    /**
     * Gives the bearing from another point to this one (in radians), where zero is directly right, PI/2 is directly
     * down, etc.
     * @param p A point.
     * @return The bearing from another point to this one..
     */
    public double bearing(Point p) {
        if(p.x() == x()) return Math.PI/2 + ((y() > p.y())? Math.PI : 0);
        return (Math.atan((p.y() - y()) / (p.x() - x())) + (p.x()<x()? Math.PI:0) + 2*Math.PI) % (2 * Math.PI);
    }

    /**
     * Renders the point in the default style - a small dot.
     * @param g A graphics instance on which to draw.
     */
    public void render(Graphics g) {
        g.fillOval(ix()-6, iy()-6, 12, 12);
    }


    /**
     * Renders the point in a specific style. The style constants are within this class.
     * @param g A graphics canvas on which to render.
     * @param style A style constant which determines how the point is drawn.
     */
    public void render(Graphics g, int style) {
        if (style == CROSS) {
            ((Graphics2D) g).setStroke(new BasicStroke(3));
            g.drawLine(ix() - 4, iy() - 4, ix() + 4, iy() + 4);
            g.drawLine(ix() + 4, iy() - 4, ix() - 4, iy() + 4);
        }
        else g.fillOval(ix() - 2, iy() - 2, 4, 4);
    }


    /**
     * Gives whether a Point is "equal" to another. Functionally, points which are equal have X and Y distance no
     * greater than {@code GRANULARITY}. Z distance is currently disconsidered.
     * Returns false for non-Point objects.
     * @param p An object to compare to.
     * @return Whether the two points are approximately in the same two-dimensional location.
     */
    @Override
    public boolean equals(Object p) {
        if(! (p instanceof Point)) return false;
        if(this == p) return true;
        Point q = (Point) p;
        return Util.approx(q.x(), x(), GRANULARITY)
                && Util.approx(q.y(), y(), GRANULARITY);
    }

    @Override
    public int hashCode() {
        int result = 1;
        for(double d: coordinates) result *= (int) d;
        return result;
    }

    /**
     * Represents this Point as a Java AWT Point object. AWT points use integer positions, so using this is not
     * recommended except for graphics purposes.
     * @return This point as a java.awt.Point.
     */
    public java.awt.Point toAWTPoint() {
        return new java.awt.Point(ix(), iy());
    }

    /**
     * Gives a Polygon which forms a square around this point of a specified width around the point. (The full width
     * of the square will be twice the parameter.)
     * @param width A distance of each side of the square from the point.
     * @return A square around the point.
     */
    public Polygon toSquare(double width) {
        return new Polygon(
                new Point(x() + width, y() + width),
                new Point(x() + width, y() - width),
                new Point(x() - width, y() - width),
                new Point(x() - width, y() + width)
        );
    }

    /**
     * Generates a new Point with additional coordinate data. The point is the same to this one in the dimensions
     * that this one has, and also includes additional values as given by the parameters.
     * @param c One or more coordinate values to append to this point.
     * @return A new point that is the result of appending those values to this one.
     */
    public Point extend(double... c) {
        double[] join = new double[dim() + c.length];
        for (int i = 0; i < join.length; i++) {
            if(i < dim()) join[i] = coordinates[i];
            else join[i] = c[i-dim()];
        }
        return new Point(join);
    }

    @Override
    public JPanel toPanel(Consumer<Object> setter, Supplier<Object> getter, Object[] specs) {
        JPanel result = new JPanel();
        JButton pickPoint = new JButton(getter.get().toString());
        boolean listening = false;
        ActionListener al = new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent actionEvent) {
                setter.accept(Option.get("input", "mouse_in_area", Point.class));
                pickPoint.setText(getter.get().toString());
                Option.ignore("input", "mouse_in_area", this);
            }
        };

        pickPoint.addActionListener(e -> {
            Option.listen("input", "mouse_in_area", al);
            pickPoint.setText("Selecting...");
            SwingUtilities.getRoot(pickPoint).setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
        });

        result.addVetoableChangeListener(a -> {
            pickPoint.setText(getter.get().toString());
        });

        result.add(pickPoint);
        return result;
    }

    @Override
    public boolean isBordered() { return false; }
}
