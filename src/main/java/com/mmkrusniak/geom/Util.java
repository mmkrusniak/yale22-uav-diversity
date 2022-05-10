package com.mmkrusniak.geom;

import java.util.ArrayList;
import java.util.List;

import static com.mmkrusniak.frame.Area.ABS_ALTITUDE;
import static com.mmkrusniak.frame.Area.COORD_SCALE;

/**
 * A static collection of useful methods. Util contains some path-planning-related and other various utility functions
 * that are use-independent.
 */
@SuppressWarnings("unused")
public class Util {

    /**
     * Gives the determinant of a matrix. The matrix must be square.
     * @param mat A matrix.
     * @return The determinant of that matrix.
     * @throws ArithmeticException if the matrix is not square.
     */
    public static double det(double[][] mat) {
        if(mat.length != mat[0].length) throw new ArithmeticException("determinant of non-square matrix");
        if(mat.length == 1) return mat[0][0]; // base case

        double sum = 0;
        for(int i = 0; i < mat.length; i++) {
            sum += ((i%2==0)? 1:-1) * mat[i][0] * det(minor(mat, i, 0));
        }
        return sum;
    }

    /**
     * Calculates the minor of a matrix - that is, the matrix without row x or column y. Useful for calculating
     * determinants.
     * @param mat A matrix.
     * @param x X value of the minor.
     * @param y Y value of the minor.
     * @return The minor of the matrix with respect to x and y.
     */
    public static double[][] minor(double[][] mat, int x, int y) {

        if(x >= mat.length || y >= mat[0].length) return mat;

        double[][] result = new double[mat.length-1][mat[0].length-1];

        for(int i = 0; i < mat.length; i++) {
            for(int j = 0; j < mat.length; j++) {
                if(i < x && j < y) result[i][j] = mat[i][j];
                if(i < x && j > y) result[i][j-1] = mat[i][j];
                if(i > x && j < y) result[i-1][j] = mat[i][j];
                if(i > x && j > y) result[i-1][j-1] = mat[i][j];

            }
        }
        return result;
    }

    /**
     * Gives whether a value is bounded by two others with a tolerance.
     * @param a Lower bound.
     * @param b Upper bound.
     * @param c Value to test.
     * @param t Tolerance.
     * @return Whether c is within a and b with tolerance t.
     */
    public static boolean within(double a, double b, double c, double t) {
        return c - t < Math.max(a, b) && c + t >= Math.min(a, b);

    }

    /**
     * Gives whether a value is bounded by two others with a tolerance. Alias for {@code within(a, b, c, 0)}.
     * @param a Lower bound.
     * @param b Upper bound.
     * @param c Value to test.
     * @return Whether c is within a and b.
     */
    public static boolean within(double a, double b, double c) {
        return c <= Math.max(a, b) && c >= Math.min(a, b);
    }

    /**
     * Gives whether a value is within some tolerance of another. (It's not a complex function, but it helps
     * readability.)
     * @param a A value.
     * @param b A value.
     * @param t A tolerance.
     * @return Whether a is within t of b.
     */
    public static boolean approx(double a, double b, double t) {
        return Math.abs(a-b) < t;
    }

    // TODO: Apache utils has lots of this type of function, and it wouldn't be so hard to import that.
    /**
     * Calculates the maximum value in an array.
     * @param array An array.
     * @param <I> Some type of comparable value.
     * @return The maximum value of the array.
     */
    @SafeVarargs
    public static <I extends Comparable<I>> I max(I... array) {
        I best = array[0];
        for(int i = 1; i < array.length; i++) {
            if(array[i].compareTo(best) > 0) best = array[i];
        }
        return best;
    }

    /**
     * Calculates the minimum value in an array.
     * @param array An array.
     * @param <I> Some type of comparable value.
     * @return The minimum value of the array.
     */
    @SafeVarargs
    public static <I extends Comparable<I>> I min(I... array) {
        I best = array[0];
        for(int i = 1; i < array.length; i++) {
            if(array[i].compareTo(best) < 0) best = array[i];
        }
        return best;
    }

    /**
     * Constrants a value to be within bounds.
     * @param i A value.
     * @param a Lower bound.
     * @param b Upper bound.
     * @return {@literal i if a<i<b, a if i<a, b if i>b.}
     */
    public static double constrain(double i, double a, double b) {
        if(i > max(a, b)) i = max(a, b);
        if(i < min(a, b)) i = min(a, b);
        return i;
    }

    /**
     * Gives the values in a grid which are adjacent to a point in the grid. There is no wrapping.
     * @param grid The grid to search.
     * @param p A point on the grid.
     * @param d The distance from the point to search.
     * @param <I> Some type of object in the grid.
     * @return The points in the grid that have distance no greater than d in x or y from p.
     */
    public static <I> List<Point> adjacent(I[][] grid, Point p, int d) {
        List<Point> result = new ArrayList<>();
        if(d == 0) {
            safeAdd(result, grid, p.ix()+1, p.iy());
            safeAdd(result, grid, p.ix()-1, p.iy());
            safeAdd(result, grid, p.ix(), p.iy()+1);
            safeAdd(result, grid, p.ix(), p.iy()-1);
        }
        for (int x = -d; x <= d; x++) {
            for (int y = -d; y <= d; y++) {
                safeAdd(result, grid, p.ix()+x, p.iy()+y);
            }
        }
        return result;
    }

    /**
     * Collect a point if it is on a grid; otherwise make no changes.
     * @param result The list to collect into.
     * @param grid The grid to check by.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param <I> Some type of item in the grid.
     */
    private static <I> void safeAdd(List<Point> result, I[][] grid, int x, int y) {
        if(x < 0 || y < 0 || x > grid.length || y > grid[0].length) return;
        result.add(new Point(x, y));
    }

    /**
     * Remove all items which are null from a list, if there are any.
     * @param list A list.
     * @param <I> Some type of items in the list.
     * @return The list, which has had null elements pruned.
     */
    public static <I> List<I> removeNull(List<I> list) {
        while(list.contains(null)) list.remove(null);
        return list;
    }

    /**
     * Gives a list of all elements which equal some element in the list.
     * @param e Some element to search for.
     * @param list A list in which to search.
     * @param <E> The type of element in the list.
     * @return A list of elements in the input list which equal e.
     */
    public static <E> List<E> occurrencesOf(E e, List<E> list) {
        List<E> result = new ArrayList<>();

        while(list.contains(e)) {
            result.add(list.remove(list.lastIndexOf(e)));
        }
        return result;
    }

    /**
     * Converts a list of points to KML format, which can be viewed on Google Earth.
     * @param points A list of points.
     * @return A KML string which renders the points.
     */
    public static String toKML(List<Point> points) {
        StringBuilder result = new StringBuilder("" +
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n" +
                "  <Document>\n" +
                "    <name>Coverage path</name>\n" +
                "    <description>Automatically generated.</description>\n" +
                "    <Style id=\"thickRedLine\">\n" +
                "          <LineStyle>\n" +
                "            <color>ff0000ff</color>\n" +
                "            <width>10</width>\n" +
                "          </LineStyle>\n" +
                "        </Style>\n" +
                "    <Placemark>\n" +
                "      <name>Absolute Extruded</name>\n" +
                "      <description></description>\n" +
                "      <styleUrl>#thickRedLine</styleUrl>\n" +
                "      <LineString>\n" +
                "        <tessellate>1</tessellate>\n" +
                "        <altitudeMode>relativeToGround</altitudeMode>\n" +
                "        <coordinates>\n");
        for (Point p : points)
            result.append("            ").append(-p.x()/COORD_SCALE).append(",").append(p.y()/COORD_SCALE).append(",").append(p.z() + ABS_ALTITUDE).append("\n");
        result.append("        </coordinates>\n      </LineString>\n    </Placemark>\n  </Document>\n</kml>");
        return result.toString();
    }
}
