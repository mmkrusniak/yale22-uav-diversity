package com.mmkrusniak.frame;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

public class Area {

    private com.mmkrusniak.geom.Polygon poly;
    private List<Detectable> objects;
    private final String name;
    private SpatialDistribution objectDistribution;


    public static final double COORD_SCALE = 10000;
    public static final double ABS_ALTITUDE = 228;


    public Area(com.mmkrusniak.geom.Point... points) {
        poly = new com.mmkrusniak.geom.Polygon(points);
        objects = new ArrayList<>(generateObjects());
        name = "Custom " + points.length + "-gon";
    }

    public Area(int n) {
        double[] angles = new double[n];
        double sum = 0.0;
        for (int i = 0; i < angles.length; i++) {
            angles[i] = Math.random() + 3.0;
            sum += angles[i];
        }
        for(int i = 0; i < angles.length; i++) angles[i] = angles[i]*Math.PI*2/sum;

        double rad = Option.get("area", "size", Integer.class)/2.0;
        double var = Option.get("area", "jaggedness", Double.class) * rad;
        poly = new com.mmkrusniak.geom.Polygon(n, rad, var);
        com.mmkrusniak.geom.Point[] points = poly.toPoints();

        // Shift the polygon into visible space
        double dx = poly.leftmost().x();
        double dy = poly.topmost().y();
        for (int i = 0; i < points.length; i++) points[i] = new com.mmkrusniak.geom.Point(points[i].x() - dx, points[i].y() - dy);

        poly = new com.mmkrusniak.geom.Polygon(points);
        objects = new ArrayList<>(generateObjects());
        name = "Random " + points.length + "-gon";
    }

    public Area(File file) {
        Charset charset = StandardCharsets.US_ASCII;
        StringBuilder kml = new StringBuilder();
        try (BufferedReader reader = Files.newBufferedReader(file.toPath(), charset)) {
            String line;
            while ((line = reader.readLine()) != null) {
                kml.append(line);
            }
        } catch (IOException x) {
            System.err.format("IOException: %s%n", x);
        }

        if(! kml.toString().contains("<coordinates>") && kml.toString().contains("</coordinates>")) throw new IllegalArgumentException("Invalid KML file.");
        kml = new StringBuilder(kml.toString().split("<coordinates>")[1].split("</coordinates>")[0].replace("\n", ""));

        ArrayList<com.mmkrusniak.geom.Point> result = new ArrayList<>();

        for(String s: kml.toString().split(" ")) {
            if(! s.equals("")) {
                String[] coords = s.split(",");
                if(coords.length < 2) continue;
                // for whatever reason, longitude comes first (it is the x coordinate I guess)
                // altitude is included, but we disregard it here
                result.add(new com.mmkrusniak.geom.Point(-COORD_SCALE*Double.parseDouble(coords[0]), COORD_SCALE*Double.parseDouble(coords[1])));
            }
        }
        poly = new com.mmkrusniak.geom.Polygon(result.toArray(new com.mmkrusniak.geom.Point[0]));
        objects = new ArrayList<>(generateObjects());
        name = "Imported area [" + file.getName() + "]";
    }

    private Paint cachedPaint;
    private Paint getPaint() {
        if(cachedPaint == null) {
            if (Option.get("graphics", "pdf", Boolean.class)) {
                BufferedImage source = new BufferedImage((int) getWidth(), (int) getHeight(), BufferedImage.TYPE_INT_RGB);
                for (int i = 0; i < getWidth(); i++) {
                    for (int j = 0; j < getHeight(); j++) {
                        double density = objectDistribution.getDensity(new com.mmkrusniak.geom.Point(i, j));
                        double d = Math.sqrt(density);
                        source.setRGB(i, j, new Color(
                                110 - (int) (d * 80.0),
                                180 - (int) (d * 180.0),
                                220 - (int) (d * 35.0)).getRGB());
                    }
                }
                cachedPaint = new TexturePaint(source, poly.toAWTPolygon().getBounds2D());
            } else {

                cachedPaint = new Color(110, 180, 0);
            }
        }
        return cachedPaint;
    }

    private List<Detectable> generateObjects() {
        int n = Option.get("objects", "n", Integer.class);
        List<Detectable> result = new ArrayList<>();
        com.mmkrusniak.geom.Point[] positions = new com.mmkrusniak.geom.Point[n];
        if(objectDistribution == null) objectDistribution = SpatialDistribution.fromOptions(this);
        for (int i = 0; i < n; i++) {
            positions[i] = objectDistribution.getPoint();
        }

        for(int i = 0; i < n; i++) {
            boolean truth = i < n/2; // exactly half of the objects are true positives; the others are false positives
            Detectable d = new Detectable(
                    positions[i].x(),
                    positions[i].y(),
                    0,
                    Math.random()*.2 + 1.0,
                    truth
            );
            result.add(d);
        }
        return result;
    }
    public void redistribute() {objects = generateObjects(); }
    public void recreateDistribution() {
        objectDistribution = null;
        cachedPaint = null;
        objects = generateObjects();
    }
    public void setDistribution(SpatialDistribution s) {
        objectDistribution = s;
        cachedPaint = null;
        objects = generateObjects();
    }

    public com.mmkrusniak.geom.Polygon getHull() {
        return poly;
    }
    public double getWidth() { return poly.getCartesianBounds().getWidth(); }
    public double getHeight() { return poly.getCartesianBounds().getHeight(); }
    public String getName() { return name; }

    public boolean contains(double x, double y) {
        return poly.encloses(new com.mmkrusniak.geom.Point(x, y));
    }

    public boolean contains(com.mmkrusniak.geom.Point p) {
        return poly.encloses(p);
    }

    public com.mmkrusniak.geom.Point getStart(int id) {
        return poly.leftish(5);
    }

    public com.mmkrusniak.geom.Point getDest(int id) {
        return poly.rightish(5);
    }

    public void render(Graphics g1) {
        Graphics2D g = (Graphics2D) g1;
        java.awt.Polygon ap = poly.toAWTPolygon();
        g.setPaint(getPaint());
        g.fillPolygon(ap);
        BasicStroke bs = new BasicStroke(5);
        g.setStroke(bs);
        g.setColor(new Color(241,181,45));
        g.drawPolygon(ap);

        for(Detectable d: objects) d.render(g);

        g.drawLine((int) getWidth()-110, (int) getHeight()-10, (int) getWidth()-10, (int) (getHeight()-10));
        g.drawString("100 meters", (int) getWidth()-110, (int) getHeight()-20);
    }

    public List<Detectable> getDetectables() {
        return new ArrayList<>(objects);
    }

    public List<Detectable> getDetectables(com.mmkrusniak.geom.Polygon p, double h) {
        List<Detectable> result = new ArrayList<>();
        for(Detectable d: objects) {
            if(p.encloses(d)) {
                Detectable n = d.atHeight(h);
                if(n.confidence() > 0) result.add(n);
            }
        }
        return result;
    }

    public double getDensity(com.mmkrusniak.geom.Point point) { return objectDistribution.getDensity(point); }
}
