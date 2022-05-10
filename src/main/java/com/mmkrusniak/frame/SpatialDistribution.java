package com.mmkrusniak.frame;

import com.mmkrusniak.geom.Point;
import com.mmkrusniak.geom.Polygon;

import javax.swing.*;
import java.util.Random;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class SpatialDistribution {
    private final Function<Point, Double> density;
    private final Supplier<Point> generator;
    private final String name;
    private static final Random rand = new Random();

    public static final String[] CATALOG = {
            "Pseudorandom",
            "Gaussian",
            "Multimodal",
            "Edge Distance",
    };

    private SpatialDistribution(Function<Point, Double> density, Supplier<Point> generator, String name) {
        this.density = density;
        this.generator = generator;
        this.name = name;
    }

    public static SpatialDistribution fromOptions(Area area) {
        switch(Option.get("dist", "spread", String.class)) {
            case "Gaussian": {
                DistMode mode = Option.get("dist", "mode", DistMode.class);
                return gaussian(area.getHull(), mode.center, mode.stdDev);
            }
            case "Multimodal": {
                DistMode[] modes = Option.get("dist", "modes", DistMode[].class);
                if(modes.length == 0) return pseudorandom(area.getHull());
                Point[] centers = new Point[modes.length];
                double[] stdDevs = new double[modes.length];
                for (int i = 0; i < centers.length; i++) {
                    System.out.println(modes[i].center);
                    centers[i] = modes[i].center;
                    stdDevs[i] = modes[i].stdDev;
                }
                return multimodal(area.getHull(), centers, stdDevs);
            }
            case "Edge Distance": return edgeDistance(area.getHull());
            default: return pseudorandom(area.getHull());
        }
    }

    public static SpatialDistribution gaussian(Polygon bounds, double stdDev) {
        return gaussian(bounds, bounds.getCenter(), stdDev);
    }
    public static SpatialDistribution gaussian(Polygon bounds, Point mean, double stdDev) {
        // Distribution where X and Y have independent Gaussian distributions
        Function<Point, Double> density = (p) -> {
            if(! bounds.encloses(p)) return 0.0;

            // Cool math thing: We don't really care about the area under the curve being 1 (in fact we'd rather
            // the maximum value be 1). And if we ignore the first half of the normal PDF equation that's exactly
            // what happens.
            double probX = Math.exp(-Math.pow((p.x()-mean.x())/stdDev, 2)/2);
            double probY = Math.exp(-Math.pow((p.y()-mean.y())/stdDev, 2)/2);
            return probX * probY;
        };
        Supplier<Point> generator = () -> {
            Point p = null;
            while(p == null || ! bounds.encloses(p)) {
                p = new Point(mean.x() + rand.nextGaussian()*stdDev, mean.y() + rand.nextGaussian()*stdDev);
            }
            return p;
        };
        return new SpatialDistribution(density, generator, "Gaussian");
    }
    public static SpatialDistribution pseudorandom(Polygon bounds) {

        Function<Point, Double> density = (p) -> {
            if(! bounds.encloses(p)) return 0.0;
            return 1.0/bounds.area();
        };
        Supplier<Point> generator = () -> {
            Point p = null;
            while(p == null || ! bounds.encloses(p)) {
                p = new Point(
                        rand.nextDouble()*bounds.getCartesianWidth(),
                        rand.nextDouble()*bounds.getCartesianHeight());
            }
            return p;
        };
        return new SpatialDistribution(density, generator, "Pseudorandom");
    }
    public static SpatialDistribution multimodal(Polygon bounds, SpatialDistribution modeDist, double[] stddevs) {

        Point[] modes = new Point[stddevs.length];
        SpatialDistribution[] dists = new SpatialDistribution[stddevs.length];
        for (int i = 0; i < modes.length; i++) {
            modes[i] = modeDist.getPoint();
            dists[i] = gaussian(bounds, modes[i], stddevs[i]);
        }

        Function<Point, Double> density = (p) -> {
            if(! bounds.encloses(p)) return 0.0;
            double sum = 0;
            for (int i = 0; i < modes.length; i++) {
                double probX = Math.exp(-Math.pow((p.x()-modes[i].x())/stddevs[i], 2)/2);
                double probY = Math.exp(-Math.pow((p.y()-modes[i].y())/stddevs[i], 2)/2);
                sum += probX * probY;
            }
            return sum / modes.length;
        };
        Supplier<Point> generator = () -> {
            SpatialDistribution dist = dists[rand.nextInt(dists.length)];
            return dist.getPoint();
        };
        return new SpatialDistribution(density, generator, "Multimodal");
    }
    public static SpatialDistribution multimodal(Polygon bounds, Point[] modes, double[] stddevs) {

        SpatialDistribution[] dists = new SpatialDistribution[stddevs.length];
        for (int i = 0; i < modes.length; i++) {
            dists[i] = gaussian(bounds, modes[i], stddevs[i]);
        }

        Function<Point, Double> density = (p) -> {
            if(! bounds.encloses(p)) return 0.0;
            double sum = 0;
            for (int i = 0; i < modes.length; i++) {
                double probX = Math.exp(-Math.pow((p.x()-modes[i].x())/stddevs[i], 2)/2);
                double probY = Math.exp(-Math.pow((p.y()-modes[i].y())/stddevs[i], 2)/2);
                sum += probX * probY;
            }
            return sum / modes.length;
        };
        Supplier<Point> generator = () -> {
            SpatialDistribution dist = dists[rand.nextInt(dists.length)];
            return dist.getPoint();
        };
        return new SpatialDistribution(density, generator, "Multimodal");
    }

    public static SpatialDistribution functional(Polygon bounds, Function<Point, Double> func, double max) {
        return functional(bounds, func, max, "Functional");
    }
    public static SpatialDistribution functional(Polygon bounds, Function<Point, Double> func, double max, String name) {

        Function<Point, Double> density = (p) -> {
            if(! bounds.encloses(p)) return 0.0;
            double result = func.apply(p) / max;
            return (result > 1)? 1 : result;
        };
        Supplier<Point> generator = () -> {
            boolean unacceptable = true;
            Point result;
            do {
                result = new Point(
                        rand.nextDouble() * bounds.getCartesianWidth(),
                        rand.nextDouble() * bounds.getCartesianHeight());
                if(rand.nextDouble() < func.apply(result) / max && bounds.encloses(result)) unacceptable = false;
            } while(unacceptable);
            return result;
        };
        return new SpatialDistribution(density, generator, name);
    }

    public static SpatialDistribution edgeDistance(Polygon bounds) {
        double max = bounds.getPolygonalWidth() + bounds.getPolygonalHeight();
        return functional(bounds, (p) -> 1.0/p.distance2D(bounds.closest(p)), 1, "Edge Distance");
    }

    public static SpatialDistribution periodic(Polygon bounds, double period) {
        Function<Point, Double> func =
                (p) -> Math.abs(Math.sin(p.x()/Math.PI*period) * Math.sin(p.y()/Math.PI*period));
        return functional(bounds, func, 1.0, "Periodic");
    }
    public static SpatialDistribution distance(Polygon bounds, Point point) {
        double max = bounds.farthest(point).sqDistance2D(point);
        Function<Point, Double> func =
                (p) -> p.sqDistance2D(point);
        return functional(bounds, func, max, "Periodic");
    }

    public Point getPoint() { return generator.get(); }
    public double getDensity(Point p) { return density.apply(p); }

    public String toString() {
        return name;
    }

    public static class DistMode implements Displayable {
        Point center;
        double stdDev;

        public DistMode(Point center, double stdDev) {
            this.center = center;
            this.stdDev = stdDev;
        }

        @Override
        public JPanel toPanel(Consumer<Object> setter, Supplier<Object> getter, Object[] specs) {
            JPanel result = new JPanel();
            result.setLayout(new BoxLayout(result, BoxLayout.LINE_AXIS));
            result.add(center.toPanel(
                    p -> { this.center = (Point) p; setter.accept(new DistMode(center, stdDev)); },
                    () -> ((DistMode) getter.get()).center,
                    null));
            result.add(Option.getComponentFor(Integer.class,
                    s -> { this.stdDev = ((Integer) s).doubleValue(); setter.accept(new DistMode(center, stdDev)); },
                    () -> (int) ((DistMode) getter.get()).stdDev,
                    new Integer[]{(int) stdDev, 1, 1000, 1}));
            return result;
        }

        @Override
        public boolean isBordered() { return true; }
    }
}
