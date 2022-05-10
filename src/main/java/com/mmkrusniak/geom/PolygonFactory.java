package com.mmkrusniak.geom;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * PolygonFactory creates some useful polygons which define regions with important characteristics for coverage
 * problems. There is also legacy code for importing polygons from other sources, though I do not recommend using that.
 *
 * We define a hierarchy of eight types of regions, the first five of which are implemented here:
 * (1) Simple shapes with 90 degree angles that can be traversed perfectly.
 * (2) Convex shapes of fair size.
 * (3) Very thin convex shapes, such as sliver triangles.
 * (4) Concave areas that are nevertheless convex in the direction of their width.
 * (5) Concave areas that are convex in at least one direction.
 * (6) Concave areas that are convex in no direction.
 * (7) Rings with removed interiors. (Not representable in this simulation.)
 * (8) Multiple discrete areas (Not representable in this simulation.)
 *
 * For visual examples see https://drive.google.com/file/d/1d56iVcuQBUGQCgW4B1Bh0AJyxrqof_1k/view?usp=sharing.
 */
public class PolygonFactory {

    public static final byte BORDER = 1;
    public static final byte OUTSIDE = 2;
    public static final byte VISITED = 3;
    public static final byte INSIDE = 4;

    /**
     * Generates a "Type 1" polygon within the requested parameters.
     * @param minWidth Minimum width of the area.
     * @param maxWidth Maximum width of the area.
     * @return A random rectangle that fits those parameters.
     */
    public static Polygon type1(double minWidth, double maxWidth) {
        // Generating a rectangle is easy.
        double width = minWidth + Math.random()*(maxWidth*minWidth);
        double height = minWidth + Math.random()*(maxWidth*minWidth);
        return new Polygon(
                new Point(0, 0),
                new Point(0, height),
                new Point(width, height),
                new Point(width, 0)
        );
    }

    /**
     * Generate a random "Type 2" polygon, a convex polygon with the guarantee that it is not too thin.
     * @param minWidth Minimum width of the polygon.
     * @param n Number of sides of the polygon.
     * @return A random "Type 2" polygon
     */
    // Type 2 is a convex polygon with width guarantees.
    public static Polygon type2(double minWidth, int n) {
        // The convex hull of any polygon (that has a width in any direction greater than the threshold) is Type 2.
        // This method is remarkably inefficient, though; convex hull is hard to calculate and there is certainly a
        // closed way to generate this type of polygon without simply rerolling bad ones, which is what this does.
        Polygon result;
        do { result = new Polygon(n).getConvexHull(); }
        while (result.getPolygonalWidth() < minWidth);

        return result;
    }

    /**
     * Generates a random "Type 3" polygon, a convex polygon that may be very thin.
     * @param maxSize The maximum polygonal width of the polygon.
     * @param n The number of sides of the polygon.
     * @param thinness A float corresponding to how thin and stretched out the polgon should be.
     * @return A random "Type 3" polygon.
     */
    // Type 3 is a convex polygon that is very thin.
    public static Polygon type3(double maxSize, int n, double thinness) {
        // To get Type 3, we're going to take a Type 2, squish it on the X axis, and rotate the result randomly.
        Polygon template = type2(maxSize, n);
        template = template.rotate(template.base().measure()); // We'd like to squish it in the thinner direction.
        double ratio = thinness / template.getCartesianWidth();
        Point[] result = new Point[n];
        Point[] templatePoints = template.toPoints();
        for (int i = 0; i < n; i++) {
            result[i] = new Point(templatePoints[i].x()*ratio, templatePoints[i].y());
        }
        return new Polygon(result).rotate(Math.random()*Math.PI*2).getConvexHull();
    }

    /**
     * Generates a random "Type 4" polygon, which is never concave in the direction of its base. (That is, a polygon
     * which is "simply traversable".)
     * @param maxSize The maximum polygonal width of the polygon.
     * @param n The number of sides of the polygon
     * @return A random "Type 4" polygon.
     */
    public static Polygon type4(double maxSize, int n) {
        // This is a bit harder. Basically, we're going to make a crown pointing upwards and a crown pointing
        // downwards (each with any number and shape of spikes), paste them together, and rotate randomly.
        // If we're really unlucky and every single spike is very short (probability is 1/2^n), we have to retry.
        // This would be a fun algorithm to analyze, but it's not really relevant sadly.

        Polygon result = null;
        while(result == null || ! result.isSimplyTraversable()) {

            Point[] resultPoints = new Point[n];
            // Top part: Right to left
            double[] locX = new double[n / 2];
            for (int i = 0; i < locX.length; i++) locX[i] = Math.random();
            Arrays.sort(locX);
            for (int i = 0; i < n / 2; i++)
                resultPoints[i] = new Point((1 - locX[i]) * maxSize, Math.random() * maxSize / 2);

            // Bottom part: Left to right
            locX = new double[n - n / 2];
            for (int i = 0; i < locX.length; i++) locX[i] = Math.random();
            Arrays.sort(locX);
            for (int i = 0; i < locX.length; i++)
                resultPoints[i + n / 2] = new Point(locX[i] * maxSize,
                        Math.random() * maxSize / 2 + maxSize / 2);

            result = new Polygon(resultPoints);
        }
        return result.rotate(Math.random()*Math.PI*2);
    }

    /**
     * Generates a random "Type 5" polygon, which is never simply traversable (that is, it is always concave in the
     * direction of its width.)
     * @param maxSize The maximum polygonal width of the polygon.
     * @param n The number of sides of the polygon.
     * @return A random "Type 5" polygon.
     */
    // Type 5 is any proper polygon (though we have to make sure it isn't any of the other types.)
    public static Polygon type5(double maxSize, int n) {
        // There's probably a way to generate a polygon that's guaranteed to be not simply traversable.
        Polygon result = new Polygon(n, 0, maxSize);
        while(result.isSimplyTraversable()) result = new Polygon(n, 0, maxSize);
        return result;
    }


    // It's nice to be able to get polygons from images sometimes
    public static Polygon fromData(byte[][] data, double fidelity) {
        java.util.List<Point> raw = new ArrayList<>();

        int startX = 0, startY = 0;
        boolean foundBorder = false;

        // Look for a valid starting location...
        for (; startX < data.length && !foundBorder; startX++) {
            for (startY = 0; startY < data[0].length && !foundBorder; startY++) {
                if (isExactBorder(startX, startY, data)) foundBorder = true;
            }
        }
        if (!foundBorder) return null;

        Point current = new Point(startX, startY);

        while (current != null) {
            raw.add(current);
            data[current.ix()][current.iy()] = VISITED;
            current = neighborder(current, data);
        }

        // This is going to be a bit flawed, because this is not a research project about polygonization.

        // First question: How far away from the other points can we go without needing to make
        // an angle to get there?
        // 100% fidelity - no data loss is acceptable. Returns a very complicated polygon (that hasn't been discretized.)
        // 0% fidelity - the simplest polygon imaginable.
        double maxStray = Math.min(data.length, data[0].length) * (1 - fidelity);

        java.util.List<Point> result = new ArrayList<>();
        List<Point> currentSeg = new ArrayList<>();

        // The first two are always good
        currentSeg.add(raw.get(0));
        currentSeg.add(raw.get(1));
        Point currentStart = raw.get(0);

        for (int i = 2; i < raw.size(); i++) {
            // The good news is that it's easy to find the distance of a point from a line segment.
            // We're going to essentially add points to a segment until we find one that's too far away.

            Point currentEnd = raw.get(i);

            // Does adding that point to the segment make any point too far away?
            boolean acceptable = true;
            for(Point p : currentSeg) if(p.distance2D(new Line(currentStart, currentEnd)) > maxStray) acceptable = false;

            if(!acceptable) {
                // Adding that point to the segment resulted in some other point being too far from the polygon.
                // In other words, we need an angle at the *last* point.
                result.add(raw.get(i - 1));
                currentSeg.clear();

                // as before, the first two are always good
                currentSeg.add(raw.get(i - 1));
                currentSeg.add(raw.get(i));
                currentStart = raw.get(i - 1);
            } else currentSeg.add(currentEnd);
        }

        result.add(currentSeg.get(currentSeg.size()-1));
        return new Polygon(result.toArray(new Point[0]));
    }

    private static boolean isExactBorder(int x, int y, byte[][] data) {
        // only pixels with all eight neighbors are eligible
        if (x <= 0 || y <= 0 || x >= data.length - 1 || y >= data[0].length - 1) return false;

        // only gold pixels are eligible
        if (data[x][y] != BORDER) return false;


        return (data[x + 1][y] == INSIDE ||
                data[x - 1][y] == INSIDE ||
                data[x][y + 1] == INSIDE ||
                data[x][y - 1] == INSIDE ||
                data[x + 1][y + 1] == INSIDE ||
                data[x + 1][y - 1] == INSIDE ||
                data[x - 1][y + 1] == INSIDE ||
                data[x - 1][y - 1] == INSIDE);
    }
    private static Point neighborder(Point p, byte[][] data) {

        if (isExactBorder(p.ix() + 1, p.iy(), data)) return new Point(p.ix() + 1, p.iy());
        if (isExactBorder(p.ix() - 1, p.iy(), data)) return new Point(p.ix() - 1, p.iy());
        if (isExactBorder(p.ix(), p.iy() + 1, data)) return new Point(p.ix(), p.iy() + 1);
        if (isExactBorder(p.ix(), p.iy() - 1, data)) return new Point(p.ix(), p.iy() - 1);
        if (isExactBorder(p.ix() + 1, p.iy() + 1, data)) return new Point(p.ix() + 1, p.iy() + 1);
        if (isExactBorder(p.ix() + 1, p.iy() - 1, data)) return new Point(p.ix() + 1, p.iy() - 1);
        if (isExactBorder(p.ix() - 1, p.iy() + 1, data)) return new Point(p.ix() - 1, p.iy() + 1);
        if (isExactBorder(p.ix() - 1, p.iy() - 1, data)) return new Point(p.ix() - 1, p.iy() - 1);
        return null;
    }
    private static byte[][] imageToData(BufferedImage image) {
        byte[][] result = new byte[image.getWidth()][image.getHeight()];

        for (int x = 0; x < result.length; x++) {
            for (int y = 0; y < result[0].length; y++) {
                switch (image.getRGB(x, y) & 0xffffff) {
                    case 0x0000ff: {
                        result[x][y] = INSIDE;
                        break;
                    }
                    case 0x888800: {
                        result[x][y] = BORDER;
                        break;
                    }
                    case 0x008800: {
                        result[x][y] = OUTSIDE;
                        break;
                    }
                }
            }
        }
        return result;
    }

    public static List<Polygon> listFromData(byte[][] data, double fidelity) {
        List<Polygon> result = new ArrayList<>();
        Polygon p = fromData(data, fidelity);
        while(p != null) {
            result.add(p);
            p = fromData(data, fidelity);
        }
        return result;
    }
}
